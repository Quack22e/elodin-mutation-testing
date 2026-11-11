"""Kestrel lander spec sheet: the documented numbers everything else is held to.

The sim builds the vehicle from these values and the checks in ``checks/``
judge telemetry against them. The checks never look at which fault (if any)
was seeded into a run, and never read the run's sampled parameters: like a
reviewer with only the datasheet and the flight recording, they must decide
from the outputs alone whether something is wrong.

Kestrel is a fictional ~1.5 t robotic lunar lander, sized so a terminal
descent from ~1.5 km takes about two minutes of sim time.

Standard library only, so the sim, the SITL flight software, and the hooks
can all import it.
"""

from __future__ import annotations

# --- Physical constants -----------------------------------------------------
G0 = 9.80665  # standard gravity, used only in the rocket equation / Isp
LUNAR_G = 1.622  # surface gravity at the landing site (not dispersed)
R_MOON_M = 1_737_400.0  # for the flat-world centrifugal relief term

# --- Timing -----------------------------------------------------------------
SIM_RATE_HZ = 100.0
SIM_DT = 1.0 / SIM_RATE_HZ
GUIDANCE_RATE_HZ = 20.0
GUIDANCE_PERIOD_TICKS = int(round(SIM_RATE_HZ / GUIDANCE_RATE_HZ))
MAX_FLIGHT_S = 210.0
MAX_TICKS = int(MAX_FLIGHT_S * SIM_RATE_HZ)
# Recording continues briefly after touchdown so the contact is captured.
POST_TOUCHDOWN_S = 1.0

# --- Vehicle (nominal value, +/- fractional dispersion) ---------------------
DRY_MASS_KG = 1_250.0
DRY_MASS_DISP = 0.02
PROPELLANT_KG = 230.0
PROPELLANT_DISP = 0.05
ISP_S = 315.0
ISP_DISP = 0.01
MAX_THRUST_N = 7_000.0
THRUST_DISP = 0.02  # engine-to-engine thrust scatter
THROTTLE_MIN = 0.15
THROTTLE_MAX = 1.0
ENGINE_TAU_S = 0.10  # first-order throttle response time constant
ENGINE_TAU_DISP = 0.20
INERTIA_DIAG_KGM2 = (1_400.0, 1_400.0, 900.0)  # at full wet mass
RCS_TORQUE_LIMIT_NM = 500.0
PAD_HEIGHT_M = 1.5  # vehicle origin height above the surface at rest

# --- Initial-state envelope (min, max) at the start of terminal descent ----
INIT_ALTITUDE_M = (1_400.0, 1_600.0)
INIT_X_M = (-1_900.0, -1_700.0)  # downrange position; the site is the origin
INIT_Y_M = (-40.0, 40.0)
INIT_VX_MPS = (40.0, 50.0)
INIT_VY_MPS = (-1.5, 1.5)
INIT_VZ_MPS = (-28.0, -22.0)
INIT_TILT_DEG = (15.0, 25.0)  # pitched back, thrust retrograde

# --- Navigation sensors (1-sigma white noise) -------------------------------
ALT_SIGMA_M = 0.5
POS_SIGMA_M = 1.0
VEL_SIGMA_MPS = 0.05
NAV_SIGMA_BOUND = 6.0  # residuals beyond 6 sigma are not noise

# --- Mission requirements ---------------------------------------------------
TOUCHDOWN_VZ_MAX_MPS = 2.0
TOUCHDOWN_VH_MAX_MPS = 0.6
TOUCHDOWN_TILT_MAX_DEG = 8.0
LANDING_MISS_MAX_M = 30.0
FUEL_RESERVE_MIN_KG = 5.0
FLIGHT_TIME_MAX_S = 200.0
FLIGHT_TILT_MAX_DEG = 45.0  # guidance never asks for more than 40 deg
BODY_RATE_MAX_DPS = 20.0
ATTITUDE_TRACK_MAX_DEG = 10.0  # setpoint vs. actual, after the initial slew
ATTITUDE_SETTLE_S = 8.0


def band(nominal: float, frac: float) -> tuple[float, float]:
    """The (min, max) range for a nominal value with a fractional dispersion."""
    return nominal * (1.0 - frac), nominal * (1.0 + frac)
