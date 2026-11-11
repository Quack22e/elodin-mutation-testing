"""Checks: decide from a flight recording and the spec sheet whether a run is wrong.

Each check sees only the run's outputs (``telemetry.npz`` + ``result.json``)
and the documented numbers in ``sim/spec_sheet.py``. None of them know which
fault was seeded, or the run's sampled inputs. A check fails when the
recording is inconsistent with physics, with the vehicle's documented
envelope, with the interface contracts between subsystems, or with the
mission requirements.

Verdicts are three-valued: a check can find a
defect (``fail``), find none (``pass``), or be unable to decide from this
recording (``undecided``, e.g. too few unsaturated guidance cycles to compare
commanded and delivered thrust). "Could not show it is wrong" is never
promoted to "right": an undecided check is neither a catch nor a pass.

Tolerances come from the spec sheet's dispersion bands plus a numerical
margin; the few that depend on closed-loop behaviour (attitude settling,
thrust-direction tracking) were set from a separate calibration campaign
before any mutation campaign was run (see README).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

import spec_sheet as S

# Numerical floors: a correct 6-DOF integration reproduces Newton's law to
# ~1e-12 m/s^2 in float64 (see README), so anything above these is real.
NEWTON_TOL_MPS2 = 1e-6
BOOKKEEPING_TOL_KG = 1e-6
QUAT_NORM_TOL = 1e-9
# Margins for checks that compare against a dispersion band.
BAND_MARGIN = 0.002
# Closed-loop tolerances (calibrated on the dev campaign, then frozen).
THRUST_DIRECTION_TOL_DEG = 6.0
ACCEL_RATIO_BAND = (
    (1.0 - S.THRUST_DISP) * (1.0 - S.DRY_MASS_DISP) - 0.01,
    (1.0 + S.THRUST_DISP) * (1.0 + S.DRY_MASS_DISP) + 0.01,
)


@dataclass
class CheckResult:
    name: str
    ok: bool  # meaningful only when decided
    value: float  # the worst observed statistic
    limit: float  # the bound it was held to
    detail: str = ""
    decided: bool = True

    @property
    def verdict(self) -> str:
        if not self.decided:
            return "undecided"
        return "pass" if self.ok else "fail"

    @property
    def headroom(self) -> float:
        """value / limit: < 1 passes; how close a clean run came to failing."""
        return abs(self.value) / abs(self.limit) if self.limit else math.inf


class Flight:
    def __init__(self, telemetry: Path, result: dict) -> None:
        data = np.load(telemetry)
        self.ticks = data["ticks"]
        self.cols = {str(n): i for i, n in enumerate(data["tick_columns"])}
        self.guidance = data["guidance"]
        self.gcols = {str(n): i for i, n in enumerate(data["guidance_columns"])}
        self.result = result

    def __getitem__(self, name: str) -> np.ndarray:
        return self.ticks[:, self.cols[name]]

    def vec(self, *names: str) -> np.ndarray:
        return self.ticks[:, [self.cols[n] for n in names]]

    def g(self, name: str) -> np.ndarray:
        return self.guidance[:, self.gcols[name]]

    @property
    def quat(self) -> np.ndarray:
        return self.vec("qx", "qy", "qz", "qw")

    @property
    def flying(self) -> np.ndarray:
        return self["landed"] < 0.5


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vectors v (N,3) by scalar-last unit quaternions q (N,4)."""
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def body_z(q: np.ndarray) -> np.ndarray:
    return rotate(q, np.tile([0.0, 0.0, 1.0], (len(q), 1)))


def angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a / np.linalg.norm(a, axis=1, keepdims=True)
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    return np.degrees(np.arccos(np.clip(np.sum(a * b, axis=1), -1.0, 1.0)))


def quat_angle_deg(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    dot = np.abs(np.sum(p * q, axis=1)) / (np.linalg.norm(p, axis=1) * np.linalg.norm(q, axis=1))
    return np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0)))


def undecided(name: str, limit: float, why: str) -> CheckResult:
    return CheckResult(name, False, math.nan, limit, why, decided=False)


def _within(value: float, lo: float, hi: float) -> bool:
    return lo <= value <= hi


# --- physics consistency ----------------------------------------------------


def check_finite(f: Flight) -> CheckResult:
    finite = bool(np.isfinite(f.ticks).all() and np.isfinite(f.guidance).all())
    norm_err = float(np.max(np.abs(np.linalg.norm(f.quat, axis=1) - 1.0))) if finite else math.inf
    return CheckResult("finite", finite and norm_err <= QUAT_NORM_TOL, norm_err, QUAT_NORM_TOL, "max | |q|-1 |")


def check_newton(f: Flight) -> CheckResult:
    """Velocity changes must equal (thrust along body +Z)/mass + gravity, every tick."""
    k = np.arange(1, len(f.ticks))
    k = k[f.flying[k] & f.flying[k - 1]]
    if len(k) == 0:
        return undecided("newton", NEWTON_TOL_MPS2, "no free-flight ticks")
    v = f.vec("vx", "vy", "vz")
    a_meas = (v[k] - v[k - 1]) / S.SIM_DT
    # The force on tick k uses the attitude and velocity at the end of tick k-1,
    # and the thrust and mass computed on tick k.
    a_thrust = body_z(f.quat[k - 1]) * (f["thrust"][k] / f["mass"][k])[:, None]
    v_h_sq = v[k - 1, 0] ** 2 + v[k - 1, 1] ** 2
    a_grav = np.zeros_like(a_thrust)
    a_grav[:, 2] = -(S.LUNAR_G - v_h_sq / S.R_MOON_M)
    residual = np.linalg.norm(a_meas - a_thrust - a_grav, axis=1)
    worst = float(np.max(residual))
    t_worst = float(f["t"][k[np.argmax(residual)]])
    return CheckResult("newton", worst <= NEWTON_TOL_MPS2, worst, NEWTON_TOL_MPS2, f"max residual m/s^2 at t={t_worst:.2f}s")


def check_mass_bookkeeping(f: Flight) -> CheckResult:
    """Rigid-body mass minus propellant is the (constant) dry mass; propellant only goes down."""
    dry = f["mass"] - f["propellant"]
    spread = float(np.max(dry) - np.min(dry))
    rises = float(np.max(np.diff(f["propellant"]), initial=0.0))
    negative = float(np.min(f["propellant"]))
    ok = spread <= BOOKKEEPING_TOL_KG and rises <= 1e-12 and negative >= 0.0
    return CheckResult("mass_bookkeeping", ok, spread, BOOKKEEPING_TOL_KG, f"dry-mass drift kg (prop rise {rises:.2e}, min {negative:.2f})")


def check_mass_flow(f: Flight) -> CheckResult:
    """Propellant used must match the rocket equation at the documented Isp."""
    prop, thrust = f["propellant"], f["thrust"]
    k = np.arange(1, len(f.ticks))
    k = k[(prop[k] > 0.0) & f.flying[k]]
    predicted = float(np.sum(thrust[k] * S.SIM_DT / (S.ISP_S * S.G0)))
    used = float(np.sum(prop[k - 1] - prop[k]))
    lo = 1.0 / (1.0 + S.ISP_DISP) - BAND_MARGIN
    hi = 1.0 / (1.0 - S.ISP_DISP) + BAND_MARGIN
    if predicted < 1.0:
        return undecided("mass_flow", hi, "engine never burned 1 kg")
    ratio = used / predicted
    return CheckResult("mass_flow", _within(ratio, lo, hi), ratio, hi, f"used/predicted, band [{lo:.4f}, {hi:.4f}]")


# --- documented envelopes and interfaces --------------------------------------


def check_engine_model(f: Flight) -> CheckResult:
    """Thrust per unit throttle is the rated thrust (within engine scatter); throttle stays in range."""
    on = (f["thrust"] > 0.0) & f.flying
    if not on.any():
        return undecided("engine_model", S.MAX_THRUST_N, "engine never on")
    rated = f["thrust"][on] / f["throttle"][on]
    lo = S.MAX_THRUST_N * (1.0 - S.THRUST_DISP - BAND_MARGIN)
    hi = S.MAX_THRUST_N * (1.0 + S.THRUST_DISP + BAND_MARGIN)
    throttle = f["throttle"][on]
    eps = 1e-9
    ok = (
        _within(float(rated.min()), lo, hi)
        and _within(float(rated.max()), lo, hi)
        and float(throttle.min()) >= S.THROTTLE_MIN - eps
        and float(throttle.max()) <= S.THROTTLE_MAX + eps
        and float(f["thrust"].min()) >= 0.0
    )
    worst = float(rated.min()) if abs(rated.min() - S.MAX_THRUST_N) > abs(rated.max() - S.MAX_THRUST_N) else float(rated.max())
    return CheckResult("engine_model", ok, worst, hi, f"thrust/throttle N, band [{lo:.0f}, {hi:.0f}]")


def check_engine_response(f: Flight) -> CheckResult:
    """Identify the throttle time constant from the log; it must match the datasheet."""
    thr, cmd = f["throttle"], np.clip(f["throttle_cmd"], S.THROTTLE_MIN, S.THROTTLE_MAX)
    k = np.arange(1, len(f.ticks))
    k = k[(f["thrust"][k] > 0) & (f["thrust"][k - 1] > 0) & f.flying[k]]
    x = cmd[k] - thr[k - 1]
    y = thr[k] - thr[k - 1]
    use = np.abs(x) > 1e-6
    lo = S.ENGINE_TAU_S * (1.0 - S.ENGINE_TAU_DISP) * (1.0 - BAND_MARGIN)
    hi = S.ENGINE_TAU_S * (1.0 + S.ENGINE_TAU_DISP) * (1.0 + BAND_MARGIN)
    if use.sum() < 10:
        return undecided("engine_response", hi, "not enough throttle activity")
    alpha = float(np.sum(x[use] * y[use]) / np.sum(x[use] ** 2))
    tau = S.SIM_DT / alpha if alpha > 0 else math.inf
    return CheckResult("engine_response", _within(tau, lo, hi), tau, hi, f"identified tau s, band [{lo:.3f}, {hi:.3f}]")


def check_attitude_envelope(f: Flight) -> CheckResult:
    """Tilt and body rates stay inside the documented flight envelope."""
    fly = f.flying
    tilt = np.degrees(np.arccos(np.clip(body_z(f.quat)[:, 2], -1.0, 1.0)))[fly]
    rate = np.degrees(np.linalg.norm(f.vec("wx", "wy", "wz"), axis=1))[fly]
    worst_tilt = float(tilt.max(initial=0.0))
    worst_rate = float(rate.max(initial=0.0))
    ok = worst_tilt <= S.FLIGHT_TILT_MAX_DEG and worst_rate <= S.BODY_RATE_MAX_DPS
    return CheckResult("attitude_envelope", ok, worst_tilt, S.FLIGHT_TILT_MAX_DEG, f"max tilt deg (max rate {worst_rate:.1f} deg/s)")


def check_attitude_tracking(f: Flight) -> CheckResult:
    """Once settled, the vehicle points where the flight software asks."""
    settled = f.flying & (f["t"] >= S.ATTITUDE_SETTLE_S)
    track = quat_angle_deg(f.quat[settled], f.vec("sp_qx", "sp_qy", "sp_qz", "sp_qw")[settled])
    worst = float(track.max(initial=0.0))
    return CheckResult("attitude_tracking", worst <= S.ATTITUDE_TRACK_MAX_DEG, worst, S.ATTITUDE_TRACK_MAX_DEG, "max setpoint error deg")


def check_nav_residual(f: Flight) -> CheckResult:
    """The FSW's navigation solution agrees with truth to within sensor noise."""
    if len(f.guidance) == 0:
        return undecided("nav_residual", S.NAV_SIGMA_BOUND, "no guidance cycles")
    idx = np.clip(np.searchsorted(f["t"], f.g("nav_t") - 1e-9), 0, len(f.ticks) - 1)
    truth = f.vec("x", "y", "z", "vx", "vy", "vz")[idx]
    nav = np.stack([f.g(n) for n in ("nav_x", "nav_y", "nav_z", "nav_vx", "nav_vy", "nav_vz")], axis=1)
    sigma = np.array([S.POS_SIGMA_M] * 2 + [S.ALT_SIGMA_M] + [S.VEL_SIGMA_MPS] * 3)
    z = np.abs(nav - truth) / sigma
    worst = float(z.max())
    channel = ["x", "y", "z", "vx", "vy", "vz"][int(np.argmax(z.max(axis=0)))]
    return CheckResult("nav_residual", worst <= S.NAV_SIGMA_BOUND, worst, S.NAV_SIGMA_BOUND, f"max |nav-truth|/sigma on {channel}")


def check_command_latency(f: Flight) -> CheckResult:
    """A command must be based on navigation no older than one guidance cycle."""
    limit = S.GUIDANCE_PERIOD_TICKS * S.SIM_DT + 1e-9
    if len(f.guidance) == 0:
        return undecided("command_latency", limit, "no guidance cycles")
    worst = float(np.max(f.g("t") - f.g("nav_t")))
    return CheckResult("command_latency", worst <= limit, worst, limit, "max command age s")


def _delivered_vs_commanded(f: Flight) -> tuple[np.ndarray, np.ndarray]:
    """Commanded thrust acceleration per guidance cycle, and the tick it should be delivered by.

    Only settled, in-flight cycles whose throttle command is not saturated count:
    a saturated command is, by definition, not delivered as asked.
    """
    t = f.g("t")
    idx = np.clip(np.searchsorted(f["t"], t + S.GUIDANCE_PERIOD_TICKS * S.SIM_DT - 1e-9), 0, len(f.ticks) - 1)
    throttle_cmd = f.g("throttle")
    use = (
        (t >= S.ATTITUDE_SETTLE_S)
        & f.flying[idx]
        & (throttle_cmd > S.THROTTLE_MIN + 1e-6)
        & (throttle_cmd < S.THROTTLE_MAX - 1e-6)
    )
    a_cmd = np.stack([f.g("ax"), f.g("ay"), f.g("az")], axis=1)[use]
    return a_cmd, idx[use]


def check_thrust_magnitude(f: Flight) -> CheckResult:
    """Delivered thrust acceleration matches what guidance asked for."""
    lo, hi = ACCEL_RATIO_BAND
    a_cmd, k = _delivered_vs_commanded(f)
    if len(k) < 10:
        return undecided("thrust_magnitude", hi, "not enough unsaturated guidance cycles")
    delivered = f["thrust"][k] / f["mass"][k]
    ratio = float(np.median(delivered / np.linalg.norm(a_cmd, axis=1)))
    return CheckResult("thrust_magnitude", _within(ratio, lo, hi), ratio, hi, f"median delivered/commanded, band [{lo:.3f}, {hi:.3f}]")


def check_thrust_direction(f: Flight) -> CheckResult:
    """Delivered thrust points where guidance asked for it."""
    a_cmd, k = _delivered_vs_commanded(f)
    if len(k) < 10:
        return undecided("thrust_direction", THRUST_DIRECTION_TOL_DEG, "not enough unsaturated guidance cycles")
    error = float(np.percentile(angle_deg(body_z(f.quat[k]), a_cmd), 95))
    return CheckResult("thrust_direction", error <= THRUST_DIRECTION_TOL_DEG, error, THRUST_DIRECTION_TOL_DEG, "p95 direction error deg")


def check_initial_state(f: Flight) -> CheckResult:
    """The first recorded state lies inside the documented start-of-descent envelope."""
    r0 = f.ticks[0]
    c = f.cols
    tilt0 = float(np.degrees(np.arccos(np.clip(body_z(f.quat[:1])[0, 2], -1.0, 1.0))))
    dry0 = float(r0[c["mass"]] - r0[c["propellant"]])
    # One tick has elapsed by the first record: allow a tick of motion and burn.
    pos_m, vel_m, burn_m = 1.0, 0.05, 0.05
    tests = {
        "z": (r0[c["z"]], S.INIT_ALTITUDE_M, pos_m),
        "x": (r0[c["x"]], S.INIT_X_M, pos_m),
        "y": (r0[c["y"]], S.INIT_Y_M, pos_m),
        "vx": (r0[c["vx"]], S.INIT_VX_MPS, vel_m),
        "vy": (r0[c["vy"]], S.INIT_VY_MPS, vel_m),
        "vz": (r0[c["vz"]], S.INIT_VZ_MPS, vel_m),
        "tilt": (tilt0, S.INIT_TILT_DEG, 0.5),
        "dry_mass": (dry0, S.band(S.DRY_MASS_KG, S.DRY_MASS_DISP), 0.01),
        "propellant": (r0[c["propellant"]], S.band(S.PROPELLANT_KG, S.PROPELLANT_DISP), burn_m),
    }
    out = [name for name, (v, (lo, hi), m) in tests.items() if not _within(float(v), lo - m, hi + m)]
    # "Pitched back": the engine's horizontal thrust must oppose horizontal velocity.
    axis = body_z(f.quat[:1])[0]
    if axis[0] * r0[c["vx"]] + axis[1] * r0[c["vy"]] >= 0.0:
        out.append("tilt_direction")
    return CheckResult("initial_state", not out, float(len(out)), 0.5, "outside envelope: " + (",".join(out) or "none"))


# --- mission requirements -----------------------------------------------------


def check_landing(f: Flight) -> CheckResult:
    r = f.result
    fails = []
    if not r.get("landed"):
        fails.append("no touchdown")
    else:
        if r["touchdown_vz_mps"] > S.TOUCHDOWN_VZ_MAX_MPS:
            fails.append(f"vz {r['touchdown_vz_mps']:.2f}")
        if r["touchdown_vh_mps"] > S.TOUCHDOWN_VH_MAX_MPS:
            fails.append(f"vh {r['touchdown_vh_mps']:.2f}")
        if r["touchdown_tilt_deg"] > S.TOUCHDOWN_TILT_MAX_DEG:
            fails.append(f"tilt {r['touchdown_tilt_deg']:.1f}")
        if r["flight_time_s"] > S.FLIGHT_TIME_MAX_S:
            fails.append(f"time {r['flight_time_s']:.0f}s")
    if r.get("miss_m", math.inf) > S.LANDING_MISS_MAX_M:
        fails.append(f"miss {r.get('miss_m', math.nan):.1f}m")
    if r.get("fuel_remaining_kg", 0.0) < S.FUEL_RESERVE_MIN_KG:
        fails.append(f"fuel {r.get('fuel_remaining_kg', math.nan):.1f}kg")
    return CheckResult("landing", not fails, float(len(fails)), 0.5, "; ".join(fails) or "all requirements met")


CHECKS = [
    check_finite,
    check_newton,
    check_mass_bookkeeping,
    check_mass_flow,
    check_engine_model,
    check_engine_response,
    check_attitude_envelope,
    check_attitude_tracking,
    check_nav_residual,
    check_command_latency,
    check_thrust_magnitude,
    check_thrust_direction,
    check_initial_state,
    check_landing,
]
CHECK_NAMES = [fn.__name__.removeprefix("check_") for fn in CHECKS]


def run_checks(telemetry: Path, result: dict) -> list[CheckResult]:
    flight = Flight(telemetry, result)
    out = []
    for fn in CHECKS:
        try:
            out.append(fn(flight))
        except Exception as exc:  # a check that can't evaluate a recording can't decide it
            out.append(undecided(fn.__name__.removeprefix("check_"), 0.0, f"check error: {exc!r}"))
    return out


SIGNATURE_COLUMNS = ["x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw", "propellant", "thrust"]


def signature(telemetry: Path, every: int = 10) -> np.ndarray:
    """A decimated trace of the observable state, for pairing a run with its clean twin."""
    flight = Flight(telemetry, {})
    return flight.vec(*SIGNATURE_COLUMNS)[::every]


def as_dicts(results: list[CheckResult]) -> list[dict]:
    return [asdict(r) | {"verdict": r.verdict, "headroom": r.headroom} for r in results]
