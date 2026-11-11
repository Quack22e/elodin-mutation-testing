#!/usr/bin/env python3
"""Kestrel terminal descent: Elodin entry point.

- builds the plant from the run's sampled parameters (``vehicle.py``),
- launches the flight software as a SITL process (``controller.py``) and
  bridges navigation state to it in lockstep from ``post_step``,
- records a flight log (``telemetry.npz``) and a ``result.json`` summary
  into the run directory for the post-run checks.

Single run (editor or headless)::

    python sim/main.py run              # nominal parameters, no fault

Campaign::

    elodin monte-carlo run sim/main.py --campaign campaigns/campaign.toml \\
        --spec campaigns/mutation.toml --out dbs/mutation

Set ``KESTREL_FSW=inproc`` to run guidance inside the sim process instead of
over UDP (identical commands, no second process).
"""

from __future__ import annotations

import math
import os
import socket
import sys
import time
from pathlib import Path

import elodin as el
import numpy as np

import spec_sheet as S
from faults import from_meta
from fsw import Guidance, pack_reply, pack_state, unpack_reply
from sensors import NavSensor
from vehicle import PARAMS, build, resolve_plant

STARTUP_TIMEOUT_S = 20.0
STEP_TIMEOUT_S = 5.0
RESEND_S = 0.2


class FswTimeout(RuntimeError):
    pass


class SitlFsw:
    """Lockstep UDP bridge: every state is answered before the sim advances."""

    def __init__(self) -> None:
        self.state_addr = ("127.0.0.1", el.monte_carlo.port("state", 9103))
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_sock.bind(("127.0.0.1", el.monte_carlo.port("command", 9102)))
        self.command_sock.settimeout(RESEND_S)
        self.seq = 0

    def step(self, nav) -> list[float]:
        self.seq += 1
        payload = pack_state(self.seq, nav)
        deadline = time.monotonic() + (STARTUP_TIMEOUT_S if self.seq == 1 else STEP_TIMEOUT_S)
        while time.monotonic() < deadline:
            self.state_sock.sendto(payload, self.state_addr)
            try:
                while True:
                    seq, fields = unpack_reply(self.command_sock.recvfrom(1024)[0])
                    if seq == self.seq:
                        return fields
            except TimeoutError:
                continue
        raise FswTimeout(f"no FSW reply to state {self.seq}")


class InProcessFsw:
    def __init__(self, fault) -> None:
        self.guidance = Guidance(fault)
        self.seq = 0

    def step(self, nav) -> list[float]:
        self.seq += 1
        return unpack_reply(pack_reply(self.seq, self.guidance.step(nav)))[1]


# Flight-log layout: one row per tick, one row per guidance cycle.
TICK_COLUMNS = (
    ["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"]
    + ["mass", "propellant", "thrust", "throttle", "throttle_cmd"]
    + ["sp_qx", "sp_qy", "sp_qz", "sp_qw", "tq_x", "tq_y", "tq_z", "landed"]
    + ["td_vz", "td_vh", "td_tilt"]
)
GUIDANCE_COLUMNS = ["t", "throttle", "qx", "qy", "qz", "qw", "ax", "ay", "az", "phase"] + [
    "nav_t", "nav_x", "nav_y", "nav_z", "nav_vx", "nav_vy", "nav_vz"
]
READS = [
    "lander.world_pos",
    "lander.world_vel",
    "lander.inertia",
    "lander.propellant",
    "lander.thrust",
    "lander.throttle",
    "lander.throttle_cmd",
    "lander.attitude_setpoint",
    "lander.rcs_torque",
    "lander.landed",
    "lander.touchdown",
]

params = el.monte_carlo.params(PARAMS)
fault = from_meta(params.meta)
plant = resolve_plant(params, fault)
world, system = build(plant, fault)
in_campaign = os.environ.get("ELODIN_MONTE_CARLO_CONTEXT") is not None
run_dir = params.run_dir or os.environ.get("KESTREL_RUN_DIR")

use_sitl = os.environ.get("KESTREL_FSW", "sitl") != "inproc"
if use_sitl:
    # No `cwd`: the FSW inherits the campaign's working directory, because
    # `elodin monte-carlo run` hands out the run context as a path relative to
    # it. (controller.py puts its own directory on sys.path for imports.)
    world.recipe(
        el.s10.PyRecipe.process(
            name="Kestrel FSW",
            cmd=sys.executable,
            args=[str(Path(__file__).with_name("controller.py"))],
        )
    )

sensor = NavSensor(int(params.get("noise_seed")), fault)
fsw = None
log = np.full((S.MAX_TICKS, len(TICK_COLUMNS)), np.nan)
guidance_log = np.full((S.MAX_TICKS // S.GUIDANCE_PERIOD_TICKS + 1, len(GUIDANCE_COLUMNS)), np.nan)
rows = 0
cycles = 0
command = None
landed_tick = None
done = False
error = ""


def finish(tick: int) -> None:
    global done
    done = True
    tel = log[:rows]
    col = {name: i for i, name in enumerate(TICK_COLUMNS)}
    last = tel[-1] if rows else np.full(len(TICK_COLUMNS), np.nan)
    landed = bool(rows and last[col["landed"]] > 0.5)
    summary = {
        "valid": not error,
        "error": error,
        "landed": landed,
        "flight_time_s": landed_tick * S.SIM_DT if landed_tick is not None else tick * S.SIM_DT,
        # None, not NaN: result() only accepts finite numbers (JSON null here).
        "touchdown_vz_mps": float(abs(last[col["td_vz"]])) if landed else None,
        "touchdown_vh_mps": float(last[col["td_vh"]]) if landed else None,
        "touchdown_tilt_deg": float(last[col["td_tilt"]]) if landed else None,
        "miss_m": float(math.hypot(last[col["x"]], last[col["y"]])),
        "fuel_remaining_kg": float(last[col["propellant"]]),
    }
    if run_dir:
        np.savez_compressed(
            Path(run_dir) / "telemetry.npz",
            ticks=tel,
            tick_columns=np.array(TICK_COLUMNS),
            guidance=guidance_log[:cycles],
            guidance_columns=np.array(GUIDANCE_COLUMNS),
        )
    if in_campaign:
        el.monte_carlo.result(**summary)
    else:
        print("kestrel:", {k: round(v, 3) if isinstance(v, float) else v for k, v in summary.items()})


def post_step(tick: int, ctx: el.StepContext) -> None:
    global command, cycles, error, fsw, landed_tick, rows
    if done:
        return
    r = ctx.component_batch_operation(reads=READS)
    pos = np.asarray(r["lander.world_pos"], dtype=np.float64)
    vel = np.asarray(r["lander.world_vel"], dtype=np.float64)
    landed = float(r["lander.landed"][0]) > 0.5
    t = tick * S.SIM_DT
    log[rows] = np.concatenate(
        [
            [t],
            pos[4:7],
            pos[0:4],
            vel[3:6],
            vel[0:3],
            [float(r["lander.inertia"][-1])],
            np.asarray(r["lander.propellant"]),
            np.asarray(r["lander.thrust"]),
            np.asarray(r["lander.throttle"]),
            np.asarray(r["lander.throttle_cmd"]),
            np.asarray(r["lander.attitude_setpoint"]),
            np.asarray(r["lander.rcs_torque"]),
            [1.0 if landed else 0.0],
            np.asarray(r["lander.touchdown"]),
        ]
    )
    rows += 1

    if landed and landed_tick is None:
        landed_tick = tick
    if tick % S.GUIDANCE_PERIOD_TICKS == 0 and not landed:
        nav = sensor.measure(t, tuple(pos[4:7]), tuple(vel[3:6]), float(r["lander.propellant"][0]))
        try:
            if fsw is None:
                fsw = SitlFsw() if use_sitl else InProcessFsw(fault)
            command = fsw.step(nav)
        except FswTimeout as exc:
            error = str(exc)
            finish(tick)
            return
        guidance_log[cycles] = [t, *command]
        cycles += 1
        ctx.component_batch_operation(
            writes={
                "lander.throttle_cmd": np.array([command[0]], dtype=np.float64),
                "lander.attitude_setpoint": np.array(command[1:5], dtype=np.float64),
            }
        )

    post_touchdown = landed_tick is not None and tick - landed_tick >= S.POST_TOUCHDOWN_S * S.SIM_RATE_HZ
    if post_touchdown or rows >= S.MAX_TICKS or tick >= S.MAX_TICKS - 1:
        finish(tick)


world.run(
    system,
    simulation_rate=S.SIM_RATE_HZ,
    max_ticks=S.MAX_TICKS,
    db_path=params.db_path or os.environ.get("ELODIN_DB_PATH"),
    post_step=post_step,
    is_canceled=lambda: done,
    interactive=not in_campaign and os.environ.get("ELODIN_NON_INTERACTIVE") != "1",
    log_level="warn",
)
