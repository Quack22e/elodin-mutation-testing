"""Kestrel flight software: powered-descent guidance and the SITL wire format.

Two phases, chosen by altitude:

- **Approach** - ZEM/ZEV feedback guidance (the closed-form energy-optimal
  law) toward a low gate 30 m above the site, descending at 2 m/s. Time to
  go is re-estimated every cycle from the vertical channel, so the law closes
  the loop on thrust, mass, and initial-state dispersions, but it may only
  count down: an estimate from a vehicle that has slowed would otherwise keep
  pushing the gate back, and the approach creeps (found by the v1 campaign).
- **Terminal** - vertical let-down on an altitude-scheduled descent rate,
  with a bounded PD loop nulling horizontal position and drift.

The commanded thrust acceleration is lightly smoothed, tilt-limited, and
slew-rate limited in direction, then converted to an attitude setpoint (body
+Z along the thrust vector, zero yaw) and a throttle.

Standard library only: this module runs inside the SITL process.
"""

from __future__ import annotations

import math
import struct
from collections import deque
from dataclasses import dataclass

import spec_sheet as S
from faults import ActiveFault

GATE_ALT_M = 30.0
GATE_VZ_MPS = -2.0
# ZEM/ZEV feeds position noise through with gain 6/tgo^2; below ~10 s the
# commanded tilt starts chasing sensor noise, so hand over to terminal there.
TGO_MIN_S = 10.0
APPROACH_TILT_MAX_DEG = 40.0
TERMINAL_TILT_MAX_DEG = 12.0
MIN_VERTICAL_ACCEL = 0.2
# First-order smoothing of the commanded acceleration (per 20 Hz cycle),
# so attitude setpoints don't jitter with navigation noise.
COMMAND_SMOOTHING = 0.2
# Rate limit on the commanded thrust direction, so a phase change never asks
# the attitude loop for a step it can't follow.
MAX_SLEW_DPS = 8.0

# Wire format (little-endian doubles). Every reply echoes the request's
# sequence number so the bridge can run the two processes in lockstep.
STATE_FMT = "<9d"  # seq, t, x, y, z, vx, vy, vz, propellant
REPLY_FMT = "<17d"  # seq, throttle, q(x,y,z,w), accel(3), phase, nav(t,x,y,z,vx,vy,vz)
STATE_SIZE = struct.calcsize(STATE_FMT)
REPLY_SIZE = struct.calcsize(REPLY_FMT)


@dataclass(frozen=True)
class Nav:
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    propellant: float


@dataclass(frozen=True)
class Command:
    throttle: float
    quat: tuple[float, float, float, float]  # x, y, z, w (scalar last)
    accel: tuple[float, float, float]  # commanded thrust acceleration, world frame
    phase: int  # 0 approach, 1 terminal
    nav: Nav  # the navigation state this command was computed from


def _norm(v: tuple[float, ...]) -> float:
    return math.sqrt(sum(c * c for c in v))


def attitude_for_thrust(direction: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Shortest-arc quaternion rotating body +Z onto ``direction``."""
    n = _norm(direction)
    ux, uy, uz = (c / n for c in direction)
    q = (-uy, ux, 0.0, 1.0 + uz)
    qn = _norm(q)
    if qn < 1e-9:  # thrust straight down: any 180 deg flip about a horizontal axis
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / qn for c in q)  # type: ignore[return-value]


def slew_toward(current: tuple[float, ...], target: tuple[float, ...], max_rad: float) -> tuple[float, ...]:
    """Rotate unit vector ``current`` toward unit vector ``target`` by at most ``max_rad``."""
    cos_t = max(-1.0, min(1.0, sum(c * t for c, t in zip(current, target))))
    if math.acos(cos_t) <= max_rad:
        return target
    perp = tuple(t - cos_t * c for c, t in zip(current, target))
    n = _norm(perp)
    if n < 1e-12:
        return current
    return tuple(math.cos(max_rad) * c + math.sin(max_rad) * p / n for c, p in zip(current, perp))


def limit_tilt(a: list[float], max_deg: float) -> list[float]:
    a[2] = max(a[2], MIN_VERTICAL_ACCEL)
    horizontal = math.hypot(a[0], a[1])
    cap = a[2] * math.tan(math.radians(max_deg))
    if horizontal > cap:
        a[0] *= cap / horizontal
        a[1] *= cap / horizontal
    return a


class Guidance:
    def __init__(self, fault: ActiveFault = ActiveFault()) -> None:
        self.fault = fault
        self.gravity = S.G0 if fault.is_("fsw_gravity_earth") else S.LUNAR_G
        self.site = (fault.magnitude if fault.is_("site_offset") else 0.0, 0.0)
        self.phase = 0
        self.delay = int(fault.magnitude) if fault.is_("stale_command") else 0
        self.pending: deque[Command] = deque()
        self.smoothed: list[float] | None = None
        self.direction: tuple[float, ...] | None = None
        self.tgo: float | None = None

    def step(self, nav: Nav) -> Command:
        cmd = self._compute(nav)
        if self.delay:
            # Bug: commands queue up and leave the FSW `delay` cycles late.
            self.pending.append(cmd)
            if len(self.pending) > self.delay:
                return self.pending.popleft()
            return self.pending[0]
        return cmd

    def _compute(self, nav: Nav) -> Command:
        g = self.gravity
        r = (nav.x - self.site[0], nav.y - self.site[1], nav.z)
        v = (nav.vx, nav.vy, nav.vz)
        estimate = 2.0 * (r[2] - GATE_ALT_M) / max(abs(v[2]) + abs(GATE_VZ_MPS), 0.1)
        countdown = estimate if self.tgo is None else self.tgo - 1.0 / S.GUIDANCE_RATE_HZ
        self.tgo = min(estimate, countdown)
        if self.phase == 0 and (r[2] <= GATE_ALT_M + 1.0 or self.tgo <= TGO_MIN_S):
            self.phase = 1

        if self.phase == 0:
            tgo = max(self.tgo, TGO_MIN_S)
            target_r = (0.0, 0.0, GATE_ALT_M)
            target_v = (0.0, 0.0, GATE_VZ_MPS)
            grav = (0.0, 0.0, -g)
            a = []
            for i in range(3):
                zem = target_r[i] - (r[i] + v[i] * tgo + 0.5 * grav[i] * tgo * tgo)
                zev = target_v[i] - (v[i] + grav[i] * tgo)
                a.append(6.0 * zem / (tgo * tgo) - 2.0 * zev / tgo)
            tilt_max = APPROACH_TILT_MAX_DEG
        else:
            height = r[2] - S.PAD_HEIGHT_M
            vz_ref = -min(max(0.15 * height, 0.7), 2.0)
            ax = -0.08 * r[0] - 0.6 * v[0]
            ay = -0.08 * r[1] - 0.6 * v[1]
            h = math.hypot(ax, ay)
            if h > 0.4:
                ax, ay = ax * 0.4 / h, ay * 0.4 / h
            a = [ax, ay, 1.2 * (vz_ref - v[2]) + g]
            tilt_max = TERMINAL_TILT_MAX_DEG

        if self.fault.is_("crossrange_sign"):
            a[1] = -a[1]
        if self.smoothed is None:
            self.smoothed = list(a)
        self.smoothed = [s + COMMAND_SMOOTHING * (n - s) for s, n in zip(self.smoothed, a)]
        a = limit_tilt(list(self.smoothed), tilt_max)
        size = _norm(a)
        want = tuple(c / size for c in a)
        max_step = math.radians(MAX_SLEW_DPS) / S.GUIDANCE_RATE_HZ
        self.direction = want if self.direction is None else slew_toward(self.direction, want, max_step)
        a = [size * c for c in self.direction]

        mass = S.DRY_MASS_KG + (0.0 if self.fault.is_("mass_estimate_dry") else nav.propellant)
        throttle = mass * _norm(a) / S.MAX_THRUST_N
        throttle = min(max(throttle, S.THROTTLE_MIN), S.THROTTLE_MAX)

        q = attitude_for_thrust((a[0], a[1], a[2]))
        if self.fault.is_("quat_wxyz"):
            q = (q[3], q[0], q[1], q[2])
        elif self.fault.is_("quat_conjugate"):
            q = (-q[0], -q[1], -q[2], q[3])
        return Command(throttle, q, (a[0], a[1], a[2]), self.phase, nav)


def pack_state(seq: int, nav: Nav) -> bytes:
    return struct.pack(STATE_FMT, seq, nav.t, nav.x, nav.y, nav.z, nav.vx, nav.vy, nav.vz, nav.propellant)


def unpack_state(raw: bytes) -> tuple[int, Nav]:
    seq, *fields = struct.unpack(STATE_FMT, raw[:STATE_SIZE])
    return int(seq), Nav(*fields)


def pack_reply(seq: int, cmd: Command) -> bytes:
    n = cmd.nav
    return struct.pack(REPLY_FMT, seq, cmd.throttle, *cmd.quat, *cmd.accel, cmd.phase, n.t, n.x, n.y, n.z, n.vx, n.vy, n.vz)


def unpack_reply(raw: bytes) -> tuple[int, list[float]]:
    seq, *fields = struct.unpack(REPLY_FMT, raw[:REPLY_SIZE])
    return int(seq), fields
