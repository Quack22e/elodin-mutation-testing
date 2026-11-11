"""Navigation sensor model: truth plus white noise, plus sensor-layer faults.

Noise is drawn in a fixed order every sample, whatever the fault, so a clean
run and a faulted run of the same dispersion sample see identical noise.
"""

from __future__ import annotations

import random

import spec_sheet as S
from faults import FT_PER_M, ActiveFault
from fsw import Nav


class NavSensor:
    def __init__(self, seed: int, fault: ActiveFault = ActiveFault()) -> None:
        self.rng = random.Random(seed)
        self.fault = fault
        self.frozen_altitude: float | None = None

    def measure(self, t: float, pos: tuple[float, ...], vel: tuple[float, ...], propellant: float) -> Nav:
        gauss = self.rng.gauss
        x = pos[0] + gauss(0.0, S.POS_SIGMA_M)
        y = pos[1] + gauss(0.0, S.POS_SIGMA_M)
        z = pos[2] + gauss(0.0, S.ALT_SIGMA_M)
        vx = vel[0] + gauss(0.0, S.VEL_SIGMA_MPS)
        vy = vel[1] + gauss(0.0, S.VEL_SIGMA_MPS)
        vz = vel[2] + gauss(0.0, S.VEL_SIGMA_MPS)

        f = self.fault
        if f.is_("altimeter_feet"):
            z *= FT_PER_M
        elif f.is_("altimeter_bias"):
            z += f.magnitude
        elif f.is_("velocity_xy_swap"):
            vx, vy = vy, vx
        elif f.is_("altimeter_stuck"):
            if self.frozen_altitude is None and pos[2] < f.magnitude:
                self.frozen_altitude = z
            if self.frozen_altitude is not None:
                z = self.frozen_altitude
        return Nav(t, x, y, z, vx, vy, vz, propellant)
