#!/usr/bin/env python3
"""A small stand-in for ``elodin monte-carlo run`` on hosts where the CLI can't start.

The macOS ``elodin`` binary links against Homebrew's ffmpeg; without it the
CLI won't load, but the Python SDK (which runs the sims) works fine. This
runner implements the subset of the campaign contract this project uses, with
the same on-disk layout, so ``sim/``, the specs, ``campaign.toml`` and the
hooks run unchanged under the real CLI:

- the plan is materialised by ``elodin.monte_carlo.sample`` (the CLI's sampler,
  including ``[meta_sweep]`` grids),
- each run gets a context JSON in ``ELODIN_MONTE_CARLO_CONTEXT`` and its named
  ports in ``ELODIN_MC_PORT_<NAME>`` from a per-worker slot,
- hooks are invoked through ``python -m elodin.monte_carlo.run_hook``,
- outputs are ``plan.csv``, ``runs/<id>/``, ``results.csv`` (with hook scalars),
  ``summary.json`` and ``post_campaign/``.

Usage mirrors the CLI::

    python harness/mc.py run sim/main.py --campaign campaigns/campaign.toml \\
        --spec campaigns/clean.toml --out dbs/clean --workers 8
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
import tomllib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from elodin.monte_carlo.sample import materialize

RESERVED_HOOK_KEYS = {"pass", "valid"}


def parse_duration(text: str | None) -> float | None:
    if not text:
        return None
    text = str(text).strip()
    for suffix, scale in (("ms", 0.001), ("s", 1.0), ("m", 60.0), ("h", 3600.0)):
        if text.endswith(suffix):
            return float(text[: -len(suffix)]) * scale
    return float(text)


def read_plan(path: Path) -> list[dict]:
    rows = []
    with path.open(newline="") as f:
        for idx, record in enumerate(csv.DictReader(f)):
            row = {"run_id": record.get("run_id") or f"run_{idx:07}", "params": {}, "meta": {}}
            if record.get("seed"):
                row["seed"] = int(record["seed"])
            for key, value in record.items():
                if value in ("", None):
                    continue
                try:
                    parsed = json.loads(value)
                except json.JSONDecodeError:
                    parsed = value
                if key.startswith("param."):
                    row["params"][key[6:]] = parsed
                elif key.startswith("meta."):
                    row["meta"][key[5:]] = parsed
            rows.append(row)
    return rows


class Campaign:
    def __init__(self, args: argparse.Namespace) -> None:
        self.sim = Path(args.sim).resolve()
        self.config = tomllib.loads(Path(args.campaign).read_text()) if args.campaign else {}
        # Kept relative, as the CLI does: run contexts then carry paths relative
        # to the campaign's working directory, exactly like the real thing.
        self.out = Path(args.out)
        self.timeout = parse_duration(self.config.get("timeout"))
        resources = self.config.get("resources", {})
        self.db_port = int(resources.get("db_port", 2240))
        self.stride = int(resources.get("port_stride", 40))
        self.ports = {name: int(base) for name, base in resources.get("ports", {}).items()}
        self.hooks = {k: Path(v).resolve() for k, v in self.config.get("hooks", {}).items()}
        retention = self.config.get("retention", {})
        self.keep_db = retention.get("keep_run_db", "on-fail")
        self.prune_on_pass = retention.get("prune_on_pass", [])
        self.prune_on_fail = retention.get("prune_on_fail", [])
        self.env = {k: str(v) for k, v in self.config.get("env", {}).items()}
        self.workers = args.workers or int(self.config.get("workers", 0)) or max(1, (os.cpu_count() or 2) - 2)
        self.lock = threading.Lock()
        self.done = 0

    def slot(self, worker: int) -> dict:
        offset = worker * self.stride
        return {
            "worker_id": worker,
            "db_port": self.db_port + offset,
            "ports": {name: base + offset for name, base in self.ports.items()},
        }

    def hook(self, kind: str, context: Path) -> None:
        subprocess.run(
            [sys.executable, "-m", "elodin.monte_carlo.run_hook", str(self.hooks[kind]), kind, str(context)],
            check=True,
            cwd=Path.cwd(),
        )

    def run_one(self, row: dict, slots: queue.Queue, total: int) -> dict:
        worker = slots.get()
        try:
            return self._run(row, self.slot(worker), total)
        finally:
            slots.put(worker)

    def _run(self, row: dict, slot: dict, total: int) -> dict:
        run_dir = self.out / "runs" / row["run_id"]
        if run_dir.exists():
            shutil.rmtree(run_dir)
        (run_dir / "logs").mkdir(parents=True)
        db_path = run_dir / "db"
        context = {
            "run_id": row["run_id"],
            "seed": row.get("seed"),
            "db_path": str(db_path),
            "db_addr": f"127.0.0.1:{slot['db_port']}",
            "run_dir": str(run_dir),
            "params": row["params"],
            "meta": row["meta"],
            "slots": slot,
        }
        context_path = run_dir / "context.json"
        context_path.write_text(json.dumps(context, indent=2))
        env = dict(os.environ, **self.env)
        env["ELODIN_MONTE_CARLO_CONTEXT"] = str(context_path)
        env["ELODIN_NON_INTERACTIVE"] = "1"
        for name, port in slot["ports"].items():
            env[f"ELODIN_MC_PORT_{name.upper()}"] = str(port)

        start = time.monotonic()
        status, reason = "ok", ""
        with (run_dir / "logs" / "sim.log").open("w") as log:
            try:
                proc = subprocess.run(
                    [sys.executable, str(self.sim), "run", context["db_addr"]],
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=self.timeout,
                    cwd=Path.cwd(),
                )
                if proc.returncode != 0:
                    status, reason = "failed", f"exit code {proc.returncode}"
            except subprocess.TimeoutExpired:
                status, reason = "failed", f"timed out after {self.timeout}s"
        wall_ms = int((time.monotonic() - start) * 1000)

        metric = {
            "run_id": row["run_id"],
            "status": status,
            "failure_reason": reason,
            "worker_id": slot["worker_id"],
            "wall_ms": wall_ms,
            "db_path": str(db_path),
            "run_dir": str(run_dir),
        }
        outcome = {}
        if "post_run" in self.hooks:
            hook_ctx = run_dir / "post_run_context.json"
            hook_ctx.write_text(
                json.dumps(
                    {
                        "run_id": row["run_id"],
                        "params": row["params"],
                        "meta": row["meta"],
                        "metrics": metric,
                        "db_path": str(db_path),
                        "run_dir": str(run_dir),
                    },
                    indent=2,
                )
            )
            try:
                self.hook("post_run", hook_ctx)
                outcome = json.loads((run_dir / "post_run_result.json").read_text())
            except (subprocess.CalledProcessError, OSError, json.JSONDecodeError) as exc:
                outcome = {"valid": False}
                metric["failure_reason"] = metric["failure_reason"] or f"post_run hook failed: {exc}"

        scored_pass = outcome.get("pass")
        scored_valid = outcome.get("valid")
        valid = status == "ok" and scored_valid is not False
        passed = valid and scored_pass is not False
        metric.update(
            passed=passed,
            valid=valid,
            scored_pass="" if scored_pass is None else bool(scored_pass),
            scored_valid="" if scored_valid is None else bool(scored_valid),
            scalars={k: v for k, v in outcome.items() if k not in RESERVED_HOOK_KEYS},
        )

        keep_db = self.keep_db == "always" or (self.keep_db == "on-fail" and not passed)
        if not keep_db and db_path.exists():
            shutil.rmtree(db_path, ignore_errors=True)
        for pattern in self.prune_on_pass if passed else self.prune_on_fail:
            for path in run_dir.glob(pattern):
                path.unlink() if path.is_file() else shutil.rmtree(path, ignore_errors=True)

        with self.lock:
            self.done += 1
            if self.done % max(1, total // 20) == 0 or self.done == total:
                print(f"[{self.done}/{total}] {row['run_id']} {'pass' if passed else 'flag'} ({wall_ms} ms)", flush=True)
        return metric

    def write_results(self, metrics: list[dict]) -> None:
        keys = sorted({k for m in metrics for k in m["scalars"]})
        header = ["run_id", "status", "passed", "valid", "degraded", "scored_pass", "scored_valid", "failure_reason"]
        header += ["worker_id", "wall_ms", "db_path", "result_json", *keys]
        with (self.out / "results.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for m in sorted(metrics, key=lambda m: m["run_id"]):
                base = [m["run_id"], m["status"], str(m["passed"]).lower(), str(m["valid"]).lower(), "false"]
                base += [str(m["scored_pass"]).lower(), str(m["scored_valid"]).lower(), m["failure_reason"]]
                base += [m["worker_id"], m["wall_ms"], m["db_path"], f"runs/{m['run_id']}/result.json"]
                writer.writerow(base + [_cell(m["scalars"].get(k)) for k in keys])

    def run(self, plan: list[dict]) -> int:
        self.out.mkdir(parents=True, exist_ok=True)
        slots: queue.Queue = queue.Queue()
        for worker in range(min(self.workers, len(plan))):
            slots.put(worker)
        print(f"campaign: {len(plan)} runs, {slots.qsize()} workers -> {self.out}", flush=True)
        start = time.monotonic()
        with ThreadPoolExecutor(max_workers=slots.qsize()) as pool:
            metrics = list(pool.map(lambda row: self.run_one(row, slots, len(plan)), plan))
        wall = time.monotonic() - start
        self.write_results(metrics)
        summary = {
            "total": len(metrics),
            "passed": sum(m["passed"] for m in metrics),
            "failed": sum(m["status"] != "ok" for m in metrics),
            "invalid": sum(not m["valid"] for m in metrics),
            "workers": slots.qsize(),
            "wall_time_s": round(wall, 2),
            "mean_run_wall_s": round(sum(m["wall_ms"] for m in metrics) / 1000 / max(len(metrics), 1), 3),
            "runner": "harness/mc.py",
        }
        (self.out / "summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        if "post_campaign" in self.hooks:
            context = self.out / "campaign_hook_context.json"
            context.write_text(
                json.dumps(
                    {
                        "out_dir": str(self.out),
                        "results": str(self.out / "results.csv"),
                        "summary": str(self.out / "summary.json"),
                    },
                    indent=2,
                )
            )
            self.hook("post_campaign", context)
        return 0


def _cell(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="execute a campaign")
    run.add_argument("sim")
    run.add_argument("--campaign")
    run.add_argument("--spec")
    run.add_argument("--plan")
    run.add_argument("--out", required=True)
    run.add_argument("--workers", type=int)
    run.add_argument("--limit", type=int, help="only run the first N plan rows (smoke tests)")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if args.spec:
        plan_path = out / "plan.csv"
        materialize(Path(args.spec), plan_path)
    elif args.plan:
        plan_path = Path(args.plan)
    else:
        parser.error("one of --spec or --plan is required")
    plan = read_plan(plan_path)
    if args.limit:
        plan = plan[: args.limit]
    return Campaign(args).run(plan)


if __name__ == "__main__":
    sys.exit(main())
