"""Seeded-fault catalog: the mutants.

Each fault is a deliberate, realistic error that a person could plausibly
commit while building or configuring this system. Faults are selected per run
through the campaign plan's ``meta.fault`` column (``"none"`` for a clean
run), so a mutation campaign is an ordinary ``elodin monte-carlo`` campaign
whose ``[meta_sweep]`` crosses every fault with the same dispersion samples.

Layers:

- ``param``  - a wrong input value or unit (the "break parameters on purpose" case)
- ``model``  - a bug in the plant physics (the JAX systems in ``vehicle.py``)
- ``fsw``    - a bug in the flight software (``fsw.py``, running as the SITL process)
- ``sensor`` - a bug or failure in the navigation sensor path (``sensors.py``)

A variant id is ``<fault>`` or ``<fault>@<magnitude>``; the magnitude meaning is
fault-specific and documented in ``description``.

What each run *requires* of the checks is declared by ``requirement()``. The
expectation is derived from the fault and the run's stated inputs alone, never
by running the sim, so the checks are not graded against the system they are
checking.

Standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import spec_sheet as S

LB_PER_KG = 2.204_622_6
N_PER_LBF = 4.448_222
FT_PER_M = 3.280_84


@dataclass(frozen=True)
class Fault:
    id: str
    layer: str
    description: str
    magnitudes: tuple[float, ...] = ()


CATALOG: dict[str, Fault] = {
    f.id: f
    for f in [
        # --- param: wrong inputs and units -------------------------------
        Fault("gravity_earth", "param", "Gravity constant set to Earth's 9.81 m/s^2 instead of the Moon's"),
        Fault("gravity_scale", "param", "Gravity constant off by the given fraction", (0.005, 0.02, 0.10)),
        Fault("isp_low", "param", "Engine Isp low by the given fraction", (0.005, 0.01, 0.02, 0.05, 0.20)),
        Fault("thrust_lbf", "param", "Max thrust copied from a datasheet in lbf, used as N"),
        Fault("thrust_scale", "param", "Engine thrust off by the given fraction", (-0.02, -0.05, -0.10, 0.10)),
        Fault("dry_mass_lb", "param", "Dry mass entered in pounds, used as kilograms"),
        Fault("propellant_short", "param", "Propellant load short by the given fraction", (0.02, 0.10, 0.30, 0.60)),
        Fault("tilt_deg_as_rad", "param", "Initial tilt in degrees passed where radians are expected"),
        Fault("vz_sign", "param", "Initial vertical velocity sign flipped (climbing)"),
        Fault("mass_flow_dt", "param", "Propellant integration uses a 50 Hz step in a 100 Hz sim"),
        Fault("inertia_x10", "param", "Inertia tensor 10x too large (g*cm^2 / kg*m^2 style slip)"),
        Fault("site_offset", "param", "Landing-site coordinates off by the given metres downrange", (100.0,)),
        # --- model: plant physics bugs -----------------------------------
        Fault("thrust_world_frame", "model", "Thrust applied along world +Z instead of body +Z"),
        Fault("mass_not_updated", "model", "Propellant burns but the rigid-body mass never changes"),
        Fault("gravity_twice", "model", "Gravity effector composed into the system twice"),
        Fault("gravity_sign", "model", "Gravity applied upward"),
        Fault("rcs_torque_sign", "model", "Attitude-control torque sign flipped"),
        Fault("throttle_unclamped", "model", "Engine model no longer clamps the throttle command"),
        Fault("centrifugal_sign", "model", "Centrifugal relief term added to gravity instead of subtracted"),
        Fault("engine_lag", "model", "Engine throttle time constant multiplied by the given factor", (1.5, 10.0)),
        Fault("mdot_lunar_g", "model", "Mass flow uses local lunar g instead of g0 in T/(Isp*g)"),
        # --- fsw: flight software bugs -----------------------------------
        Fault("quat_wxyz", "fsw", "Attitude command sent scalar-first (w,x,y,z) to a scalar-last interface"),
        Fault("quat_conjugate", "fsw", "Attitude command sent as the inverse rotation"),
        Fault("fsw_gravity_earth", "fsw", "Guidance compensates for Earth gravity"),
        Fault("crossrange_sign", "fsw", "Crossrange (y) guidance acceleration sign flipped"),
        Fault("mass_estimate_dry", "fsw", "Guidance mass estimate omits remaining propellant"),
        Fault("stale_command", "fsw", "Commands applied the given number of guidance cycles late", (1.0, 2.0, 10.0)),
        # --- sensor: navigation path faults -------------------------------
        Fault("altimeter_feet", "sensor", "Altimeter driver reports feet as metres"),
        Fault("altimeter_bias", "sensor", "Altimeter reads high by the given metres", (0.5, 1.0, 2.0, 10.0, 50.0)),
        Fault("velocity_xy_swap", "sensor", "Horizontal velocity axes swapped in the nav message"),
        Fault("altimeter_stuck", "sensor", "Altimeter freezes once below the given metres", (100.0,)),
    ]
}


@dataclass(frozen=True)
class ActiveFault:
    """The fault seeded into this run (``id == "none"`` for a clean run)."""

    id: str = "none"
    magnitude: float = 0.0
    variant: str = "none"

    @property
    def layer(self) -> str:
        return CATALOG[self.id].layer if self.id in CATALOG else "none"

    def is_(self, fault_id: str) -> bool:
        return self.id == fault_id


def variant_id(fault_id: str, magnitude: float | None) -> str:
    if magnitude is None:
        return fault_id
    return f"{fault_id}@{magnitude:g}"


def parse_variant(variant: str) -> ActiveFault:
    if not variant or variant == "none":
        return ActiveFault()
    fault_id, _, mag = variant.partition("@")
    if fault_id not in CATALOG:
        raise ValueError(f"unknown fault {fault_id!r}; known: {sorted(CATALOG)}")
    fault = CATALOG[fault_id]
    if fault.magnitudes and not mag:
        raise ValueError(f"fault {fault_id!r} needs a magnitude, e.g. {variant_id(fault_id, fault.magnitudes[0])}")
    magnitude = float(mag) if mag else 0.0
    return ActiveFault(fault_id, magnitude, variant_id(fault_id, magnitude if mag else None))


def all_variants() -> list[str]:
    """Every seeded variant in catalog order (excluding the clean ``none``)."""
    out = []
    for fault in CATALOG.values():
        if fault.magnitudes:
            out.extend(variant_id(fault.id, m) for m in fault.magnitudes)
        else:
            out.append(fault.id)
    return out


def from_meta(meta: dict) -> ActiveFault:
    return parse_variant(str(meta.get("fault", "none")))


# --- Requirements -------------------------------------------------------------
#
# must_catch          The edit necessarily breaks a relation the checks test:
#                     Newton's law with the documented gravity, a documented
#                     band, an interface contract, or a requirement. A pass is
#                     a false negative.
# should_catch        A real out-of-spec defect, but whether it shows depends
#                     on closed-loop behaviour that can't be derived from the
#                     edit alone. A pass is a finding about the checks, not a
#                     contract violation.
# may_escape          The edit leaves every observable inside its documented
#                     band, or is permitted by the spec. A catch here is extra
#                     sensitivity, not a requirement.
# must_be_equivalent  Another part of the system already enforces what the
#                     edit removes; the run must behave exactly like its twin.
# must_pass           A clean run. Any flag is a false positive.

MUST, SHOULD, MAY, EQUIVALENT, CLEAN = "must_catch", "should_catch", "may_escape", "must_be_equivalent", "must_pass"
# A fault counts as outside a documented band only when it clears the band by
# more than this fraction of the nominal value (the checks' resolution).
BAND_CLEARANCE = 0.005

_ALWAYS = {
    "gravity_earth": MUST,  # gravity is not dispersed; Newton is exact
    "gravity_scale": MUST,
    "thrust_lbf": MUST,  # thrust/throttle = 1574 N vs a 7000 N +/- 2 % rating
    "dry_mass_lb": MUST,  # first recorded dry mass ~2700 kg vs 1225..1275 kg
    "vz_sign": MUST,  # climbing at +22..28 m/s vs the -28..-22 m/s envelope
    "mass_flow_dt": MUST,  # propellant used is 2x the rocket equation
    "inertia_x10": SHOULD,  # attitude loop slows; whether limits break is dynamic
    "site_offset": MUST,  # guidance lands on the wrong spot: 100 m > 30 m allowed
    "thrust_world_frame": MUST,  # starts tilted >= 15 deg, so Newton breaks at once
    "mass_not_updated": MUST,  # rigid-body mass minus propellant drifts
    "gravity_twice": MUST,
    "gravity_sign": MUST,
    "rcs_torque_sign": SHOULD,  # positive feedback; how it shows is dynamic
    "throttle_unclamped": EQUIVALENT,  # the FSW clamps to the same [0.15, 1] first
    "centrifugal_sign": MUST,  # residual 2 v_h^2 / R >= 1.8e-3 m/s^2 at start
    "mdot_lunar_g": MUST,  # mass flow 6x the rocket equation
    "quat_wxyz": MUST,  # setpoint is a different rotation than the commanded thrust
    "quat_conjugate": MUST,  # mirrored tilt: thrust direction off by 2x the tilt
    "fsw_gravity_earth": SHOULD,
    "crossrange_sign": SHOULD,
    "mass_estimate_dry": MUST,  # delivered/commanded ~ dry/(dry + propellant) < 0.95
    "altimeter_feet": MUST,  # residual 2.28 z >> 6 sigma
    "velocity_xy_swap": MUST,  # |vx - vy| >= 38 m/s vs 6 sigma = 0.3 m/s
    "altimeter_stuck": MUST,  # truth keeps descending below the frozen reading
}


def _outside(value: float, lo: float, hi: float, nominal: float) -> bool:
    clearance = BAND_CLEARANCE * abs(nominal)
    return value < lo - clearance or value > hi + clearance


def requirement(fault: ActiveFault, params: dict) -> str:
    """What the checks must do with this run, from the edit and the run's inputs alone."""
    if fault.id == "none":
        return CLEAN
    if fault.id in _ALWAYS:
        return _ALWAYS[fault.id]
    m = fault.magnitude
    if fault.id == "isp_low":
        effective = float(params["isp_s"]) * (1.0 - m)
        return MUST if _outside(effective, *S.band(S.ISP_S, S.ISP_DISP), S.ISP_S) else MAY
    if fault.id == "thrust_scale":
        effective = float(params["thrust_scale"]) * (1.0 + m)
        return MUST if _outside(effective, *S.band(1.0, S.THRUST_DISP), 1.0) else MAY
    if fault.id == "propellant_short":
        effective = float(params["propellant_kg"]) * (1.0 - m)
        return MUST if _outside(effective, *S.band(S.PROPELLANT_KG, S.PROPELLANT_DISP), S.PROPELLANT_KG) else MAY
    if fault.id == "engine_lag":
        effective = float(params["engine_tau_s"]) * m
        return MUST if _outside(effective, *S.band(S.ENGINE_TAU_S, S.ENGINE_TAU_DISP), S.ENGINE_TAU_S) else MAY
    if fault.id == "tilt_deg_as_rad":
        # The attitude is a rotation by -X rad about +Y; wrap it and see where it lands.
        angle = math.degrees((-float(params["init_tilt_deg"]) + math.pi) % (2.0 * math.pi) - math.pi)
        lo, hi = S.INIT_TILT_DEG
        in_envelope = angle < 0.0 and lo - 1.0 <= -angle <= hi + 1.0  # retrograde, in range
        return MAY if in_envelope else MUST
    if fault.id == "stale_command":
        return MAY if m <= 1.0 else MUST  # the spec permits one guidance cycle of latency
    if fault.id == "altimeter_bias":
        return MUST if m > S.NAV_SIGMA_BOUND * S.ALT_SIGMA_M else MAY
    raise ValueError(f"no requirement declared for {fault.variant!r}")
