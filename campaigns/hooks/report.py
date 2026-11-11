"""post_campaign hook: the mutation-testing scorecard.

Every run carries a requirement declared before it ran (``faults.requirement``,
derived from the fault and the run's inputs alone, never from running
the sim). The report holds each run's outcome against that requirement:

- ``must_pass`` (clean): a fail is a **false positive**; an undecided check
  sends it to review.
- ``must_catch``: a pass is a **false negative**, the headline failure.
- ``should_catch``: a pass is a gap in the checks, reported, not a violation.
- ``may_escape``: a catch is sensitivity beyond what the spec lets the checks
  promise.
- ``must_be_equivalent``: the run must match its clean twin (same dispersion
  sample, same noise) exactly.

A seeded run whose clean twin was itself flagged is *inconclusive*: its flag
can't be credited to the fault. Writes ``post_campaign/report.md``,
``report.json``, ``runs.csv`` and ``kill_matrix.csv``.
"""

from __future__ import annotations

import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "sim"), str(ROOT)]

import numpy as np  # noqa: E402

from checks.oracles import CHECK_NAMES  # noqa: E402
from faults import CATALOG, CLEAN, EQUIVALENT, MAY, MUST, SHOULD, all_variants, parse_variant  # noqa: E402

EQUIVALENCE_TOL = 1e-6
REQUIREMENT_ORDER = [CLEAN, MUST, SHOULD, MAY, EQUIVALENT]


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def _signature(run_dir: Path) -> np.ndarray | None:
    try:
        return np.load(run_dir / "signature.npz")["signature"]
    except (OSError, KeyError, ValueError):
        return None


def _same(a: np.ndarray | None, b: np.ndarray | None) -> bool:
    if a is None or b is None or a.shape != b.shape:
        return False
    return float(np.max(np.abs(a - b), initial=0.0)) <= EQUIVALENCE_TOL


def _pct(n: int, d: int) -> str:
    return f"{100.0 * n / d:.1f}%" if d else "n/a"


def classify(out_dir: Path) -> list[dict]:
    with (out_dir / "results.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    runs = []
    for row in rows:
        run_dir = out_dir / "runs" / row["run_id"]
        ctx = _json(run_dir / "post_run_context.json")
        params = ctx.get("params", {})
        flagged = row.get("flagged") == "true" or row["valid"] != "true"
        review = row.get("review") == "true"
        runs.append(
            {
                "run_id": row["run_id"],
                "fault": str(ctx.get("meta", {}).get("fault", row.get("fault") or "none")),
                "requirement": row.get("requirement") or CLEAN,
                "sample": json.dumps({k: params[k] for k in sorted(params)}),
                "verdict": "fail" if flagged else "review" if review else "pass",
                "failed_checks": [c for c in (row.get("failed_checks") or "").split(";") if c],
                "row": row,
                "run_dir": run_dir,
            }
        )

    twins = {r["sample"]: r for r in runs if r["fault"] == "none"}
    for r in runs:
        twin = twins.get(r["sample"])
        r["matches_twin"] = (
            r["fault"] != "none" and twin is not None and _same(_signature(r["run_dir"]), _signature(twin["run_dir"]))
        )
        r["inconclusive"] = r["fault"] != "none" and twin is not None and twin["verdict"] == "fail"
    return runs


def summarise(runs: list[dict]) -> dict:
    clean = [r for r in runs if r["requirement"] == CLEAN]
    seeded = [r for r in runs if r["requirement"] != CLEAN and not r["inconclusive"]]
    inconclusive = [r for r in runs if r["inconclusive"]]

    by_req: dict[str, Counter] = defaultdict(Counter)
    for r in seeded:
        by_req[r["requirement"]][r["verdict"]] += 1
        if r["verdict"] == "pass" and r["matches_twin"]:
            by_req[r["requirement"]]["pass_identical_to_twin"] += 1
    equivalent = by_req[EQUIVALENT]

    clean_stats = {}
    for name in CHECK_NAMES:
        values = [float(v) for r in clean if (v := r["row"].get(f"v_{name}")) not in ("", None)]
        if values:
            clean_stats[name] = {"min": min(values), "max": max(values)}

    order = {v: i for i, v in enumerate(all_variants())}
    by_variant = defaultdict(list)
    for r in seeded:
        by_variant[r["fault"]].append(r)
    variants = {}
    for variant in sorted(by_variant, key=lambda v: order.get(v, 1e9)):
        rs = by_variant[variant]
        reqs = Counter(r["requirement"] for r in rs)
        caught_by_req = Counter(r["requirement"] for r in rs if r["verdict"] == "fail")
        fault = parse_variant(variant)
        variants[variant] = {
            "layer": fault.layer,
            "description": CATALOG[fault.id].description,
            "runs": len(rs),
            "requirements": dict(reqs),
            "caught": sum(caught_by_req.values()),
            "caught_by_requirement": dict(caught_by_req),
            "review": sum(r["verdict"] == "review" for r in rs),
            "identical_to_twin": sum(r["matches_twin"] for r in rs),
            "false_negatives": sum(r["requirement"] == MUST and r["verdict"] != "fail" for r in rs),
            "checks": dict(Counter(c for r in rs for c in r["failed_checks"])),
        }

    must = by_req[MUST]
    should = by_req[SHOULD]
    may = by_req[MAY]
    n = lambda c: sum(v for k, v in c.items() if k in ("pass", "fail", "review"))  # noqa: E731
    kills = Counter(c for r in seeded for c in r["failed_checks"])
    unique = Counter(r["failed_checks"][0] for r in seeded if len(r["failed_checks"]) == 1)
    scoreable = [r for r in seeded if r["requirement"] != EQUIVALENT]
    return {
        "clean_runs": len(clean),
        "false_positives": sum(r["verdict"] == "fail" for r in clean),
        "false_positive_checks": dict(Counter(c for r in clean for c in r["failed_checks"])),
        "clean_review": sum(r["verdict"] == "review" for r in clean),
        "seeded_runs": len(seeded),
        "inconclusive_runs": len(inconclusive),
        "must_catch": {"runs": n(must), "caught": must["fail"], "review": must["review"], "false_negatives": n(must) - must["fail"]},
        "should_catch": {"runs": n(should), "caught": should["fail"], "review": should["review"], "missed": n(should) - should["fail"]},
        "may_escape": {"runs": n(may), "caught": may["fail"], "identical_to_twin": may["pass_identical_to_twin"]},
        "must_be_equivalent": {
            "runs": n(equivalent),
            "confirmed": equivalent["pass_identical_to_twin"],
            "violated": n(equivalent) - equivalent["pass_identical_to_twin"],
        },
        "all_seeded_excluding_equivalent": {"runs": len(scoreable), "caught": sum(r["verdict"] == "fail" for r in scoreable)},
        "variants": variants,
        "kills_by_check": {name: kills.get(name, 0) for name in CHECK_NAMES},
        "unique_kills_by_check": {name: unique.get(name, 0) for name in CHECK_NAMES},
        "clean_check_ranges": clean_stats,
    }


def render(s: dict, title: str) -> str:
    lines = [f"# {title}", ""]
    fp = s["false_positives"]
    lines.append(
        f"- **Negative controls**: {s['clean_runs']} clean runs, **{fp} false positives**"
        + (f" ({', '.join(f'{k}: {v}' for k, v in s['false_positive_checks'].items())})" if fp else "")
        + (f", {s['clean_review']} sent to review" if s["clean_review"] else "")
    )
    if s["seeded_runs"]:
        m, sh, may, eq, al = s["must_catch"], s["should_catch"], s["may_escape"], s["must_be_equivalent"], s["all_seeded_excluding_equivalent"]
        lines += [
            f"- **must_catch**: {m['caught']} / {m['runs']} caught ({_pct(m['caught'], m['runs'])}), "
            f"**{m['false_negatives']} false negatives**" + (f" ({m['review']} of them sent to review)" if m["review"] else ""),
            f"- **should_catch**: {sh['caught']} / {sh['runs']} caught ({_pct(sh['caught'], sh['runs'])})",
            f"- **may_escape** (inside documented bands, or permitted by spec): {may['caught']} / {may['runs']} caught "
            f"({_pct(may['caught'], may['runs'])}); {may['identical_to_twin']} left the trajectory unchanged",
            f"- **must_be_equivalent**: {eq['confirmed']} / {eq['runs']} confirmed identical to their clean twin",
            f"- All seeded runs except declared equivalents: {al['caught']} / {al['runs']} caught ({_pct(al['caught'], al['runs'])})",
        ]
        if s["inconclusive_runs"]:
            lines.append(f"- Inconclusive (clean twin was itself flagged): {s['inconclusive_runs']} seeded runs, excluded above")
        lines += [
            "",
            "## By fault variant",
            "",
            "| variant | layer | requirement (runs) | caught | false neg. | flagged by |",
            "|---|---|---|---:|---:|---|",
        ]
        for name, v in s["variants"].items():
            reqs = ", ".join(f"{k} {n}" for k, n in sorted(v["requirements"].items(), key=lambda kv: REQUIREMENT_ORDER.index(kv[0])))
            checks = ", ".join(f"{k} ({n})" for k, n in sorted(v["checks"].items(), key=lambda kv: -kv[1]))
            lines.append(
                f"| `{name}` | {v['layer']} | {reqs} | {v['caught']}/{v['runs']} | {v['false_negatives'] or ''} | {checks or '-'} |"
            )
        lines += ["", "## By check", "", "| check | seeded runs flagged | sole detector |", "|---|---:|---:|"]
        for name in CHECK_NAMES:
            lines.append(f"| `{name}` | {s['kills_by_check'][name]} | {s['unique_kills_by_check'][name]} |")
    if s["clean_check_ranges"]:
        lines += ["", "## Clean-run check statistics (min .. max)", ""]
        for name, r in s["clean_check_ranges"].items():
            lines.append(f"- `{name}`: {r['min']:.6g} .. {r['max']:.6g}")
    return "\n".join(lines) + "\n"


def post_campaign(ctx):
    out_dir = Path(ctx.out_dir)
    runs = classify(out_dir)
    s = summarise(runs)
    report_dir = out_dir / "post_campaign"
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "report.json").write_text(json.dumps(s, indent=2))
    with (report_dir / "runs.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run_id", "fault", "requirement", "verdict", "matches_twin", "inconclusive", "failed_checks"])
        for r in runs:
            writer.writerow(
                [r["run_id"], r["fault"], r["requirement"], r["verdict"], r["matches_twin"], r["inconclusive"], ";".join(r["failed_checks"])]
            )
    with (report_dir / "kill_matrix.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["variant", "runs", *CHECK_NAMES])
        for name, v in s["variants"].items():
            writer.writerow([name, v["runs"], *(v["checks"].get(c, 0) for c in CHECK_NAMES)])
    text = render(s, f"Kestrel mutation campaign: {out_dir.name}")
    (report_dir / "report.md").write_text(text)
    print(text)
    return {"false_positives": s["false_positives"], "clean_runs": s["clean_runs"], "false_negatives": s["must_catch"]["false_negatives"]}
