"""post_run hook: run every check against the run's flight recording.

``flagged`` means some check found a defect; ``review`` means none did but
some could not decide; ``pass`` means every check decided and found nothing.
The checks are handed only the recording and ``result.json``. The seeded
fault (``meta.fault``) and the run's declared requirement are added to the
scalars afterwards, for the report.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "sim"), str(ROOT)]

import numpy as np  # noqa: E402

from checks.oracles import CHECK_NAMES, as_dicts, run_checks, signature  # noqa: E402
from faults import parse_variant, requirement  # noqa: E402


def _read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def _scalar(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def post_run(ctx):
    run_dir = Path(ctx.run_dir)
    meta = vars(ctx.meta) if ctx.meta is not None else {}
    fault = str(meta.get("fault", "none"))
    params = vars(ctx.params) if ctx.params is not None else {}
    required = requirement(parse_variant(fault), params)
    result = _read_json(run_dir / "result.json")
    telemetry = run_dir / "telemetry.npz"

    if not result or not telemetry.exists() or not result.get("valid", False):
        # Nothing to judge: the sim died or the SITL link failed. Reported as
        # invalid, never silently as a pass.
        return {
            "valid": False,
            "pass": False,
            "flagged": True,
            "failed_checks": "no_recording",
            "fault": fault,
            "requirement": required,
        }

    checks = run_checks(telemetry, result)
    (run_dir / "checks.json").write_text(json.dumps(as_dicts(checks), indent=2, default=float))
    np.savez_compressed(run_dir / "signature.npz", signature=signature(telemetry))

    failed = [c.name for c in checks if c.verdict == "fail"]
    undecided = [c.name for c in checks if c.verdict == "undecided"]
    out = {
        "valid": True,
        "pass": not failed and not undecided,
        "flagged": bool(failed),
        "review": bool(undecided) and not failed,
        "n_failed": len(failed),
        "failed_checks": ";".join(failed),
        "undecided_checks": ";".join(undecided),
        "fault": fault,
        "requirement": required,
    }
    by_name = {c.name: c for c in checks}
    for name in CHECK_NAMES:
        out[f"verdict_{name}"] = by_name[name].verdict
        out[f"v_{name}"] = _scalar(float(by_name[name].value))
    for key in ("landed", "flight_time_s", "touchdown_vz_mps", "touchdown_vh_mps", "miss_m", "fuel_remaining_kg"):
        out[key] = _scalar(result.get(key))

    # Retention (after the verdict, so it can't influence it): keep the full
    # recording only where a person needs to look: a clean run that wasn't a
    # clean pass, a must/should-catch fault that wasn't caught, or a declared
    # equivalent that was flagged. KESTREL_KEEP_TELEMETRY=1 keeps all.
    expected = {
        "must_pass": out["pass"],
        "must_catch": bool(failed),
        "should_catch": bool(failed),
        "may_escape": True,
        "must_be_equivalent": not failed,
    }[required]
    if expected and os.environ.get("KESTREL_KEEP_TELEMETRY") != "1":
        telemetry.unlink()
    return out
