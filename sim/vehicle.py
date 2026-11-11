"""Kestrel plant: a 6-DOF lander in Elodin, with param and model faults.

All physics lives in JAX systems composed into ``el.six_dof``: lunar gravity
(with the flat-world centrifugal relief), a throttleable body-axis engine with
first-order response, propellant burn-down that rewrites the rigid-body mass,
an RCS attitude loop toward the flight software's attitude setpoint, and a
ground-contact latch that records touchdown state on the contact tick.

``resolve_plant`` turns the sampled campaign parameters into the effective
plant configuration, applying any ``param``-layer fault on the way;
``build`` then wires ``model``-layer faults directly into the systems.
"""

# No `from __future__ import annotations`: Elodin reads the component types
# from the system signatures at runtime.
import math
import typing as ty
from dataclasses import dataclass

import elodin as el
import jax
import jax.numpy as jnp

import spec_sheet as S
from faults import LB_PER_KG, N_PER_LBF, ActiveFault


def _ranged(default: float, bounds: tuple[float, float]) -> el.monte_carlo.Param:
    return el.monte_carlo.Param(float, default=default, min=bounds[0], max=bounds[1])


def _dispersed(nominal: float, frac: float) -> el.monte_carlo.Param:
    return _ranged(nominal, S.band(nominal, frac))


def _mid(bounds: tuple[float, float]) -> float:
    return 0.5 * (bounds[0] + bounds[1])


# Single runs use the defaults; campaigns sample the same ranges (spec.toml).
PARAMS = el.monte_carlo.params_spec(
    dry_mass_kg=_dispersed(S.DRY_MASS_KG, S.DRY_MASS_DISP),
    propellant_kg=_dispersed(S.PROPELLANT_KG, S.PROPELLANT_DISP),
    isp_s=_dispersed(S.ISP_S, S.ISP_DISP),
    thrust_scale=_dispersed(1.0, S.THRUST_DISP),
    engine_tau_s=_dispersed(S.ENGINE_TAU_S, S.ENGINE_TAU_DISP),
    init_altitude_m=_ranged(_mid(S.INIT_ALTITUDE_M), S.INIT_ALTITUDE_M),
    init_x_m=_ranged(_mid(S.INIT_X_M), S.INIT_X_M),
    init_y_m=_ranged(_mid(S.INIT_Y_M), S.INIT_Y_M),
    init_vx_mps=_ranged(_mid(S.INIT_VX_MPS), S.INIT_VX_MPS),
    init_vy_mps=_ranged(_mid(S.INIT_VY_MPS), S.INIT_VY_MPS),
    init_vz_mps=_ranged(_mid(S.INIT_VZ_MPS), S.INIT_VZ_MPS),
    init_tilt_deg=_ranged(_mid(S.INIT_TILT_DEG), S.INIT_TILT_DEG),
    # Seeds the navigation-sensor noise. It is sampled like any other
    # dispersion so a clean run and a faulted run of the same sample see the
    # same noise realisation.
    noise_seed=_ranged(0.0, (0.0, 2.0**31)),
)


@dataclass(frozen=True)
class PlantConfig:
    gravity: float
    dry_mass: float
    propellant: float
    isp: float
    max_thrust: float
    thrust_scale: float
    engine_tau: float
    inertia_diag: tuple[float, float, float]
    init_pos: tuple[float, float, float]
    init_vel: tuple[float, float, float]
    init_tilt_rad: float
    mass_flow_dt: float


def resolve_plant(params: el.monte_carlo.Params, fault: ActiveFault) -> PlantConfig:
    """Effective plant inputs for this run, after any param-layer fault."""

    def p(name: str) -> float:
        return float(params.get(name))

    gravity = S.LUNAR_G
    dry_mass = p("dry_mass_kg")
    propellant = p("propellant_kg")
    isp = p("isp_s")
    max_thrust = S.MAX_THRUST_N
    thrust_scale = p("thrust_scale")
    inertia = S.INERTIA_DIAG_KGM2
    vz = p("init_vz_mps")
    tilt_rad = math.radians(p("init_tilt_deg"))
    mass_flow_dt = S.SIM_DT

    m = fault.magnitude
    if fault.is_("gravity_earth"):
        gravity = S.G0
    elif fault.is_("gravity_scale"):
        gravity *= 1.0 + m
    elif fault.is_("isp_low"):
        isp *= 1.0 - m
    elif fault.is_("thrust_lbf"):
        max_thrust = S.MAX_THRUST_N / N_PER_LBF
    elif fault.is_("thrust_scale"):
        thrust_scale *= 1.0 + m
    elif fault.is_("dry_mass_lb"):
        dry_mass *= LB_PER_KG
    elif fault.is_("propellant_short"):
        propellant *= 1.0 - m
    elif fault.is_("tilt_deg_as_rad"):
        tilt_rad = p("init_tilt_deg")
    elif fault.is_("vz_sign"):
        vz = -vz
    elif fault.is_("mass_flow_dt"):
        mass_flow_dt = 1.0 / 50.0
    elif fault.is_("inertia_x10"):
        inertia = tuple(10.0 * i for i in inertia)

    return PlantConfig(
        gravity=gravity,
        dry_mass=dry_mass,
        propellant=propellant,
        isp=isp,
        max_thrust=max_thrust,
        thrust_scale=thrust_scale,
        engine_tau=p("engine_tau_s"),
        inertia_diag=inertia,
        init_pos=(p("init_x_m"), p("init_y_m"), p("init_altitude_m")),
        init_vel=(p("init_vx_mps"), p("init_vy_mps"), vz),
        init_tilt_rad=tilt_rad,
        mass_flow_dt=mass_flow_dt,
    )


def _f64(n: int) -> el.ComponentType:
    return el.ComponentType(el.PrimitiveType.F64, (n,))


Throttle = ty.Annotated[jax.Array, el.Component("throttle", _f64(1))]
ThrottleCmd = ty.Annotated[
    jax.Array,
    el.Component("throttle_cmd", _f64(1), metadata={"external_control": "true"}),
]
AttitudeSetpoint = ty.Annotated[
    el.Quaternion,
    el.Component("attitude_setpoint", metadata={"external_control": "true"}),
]
Thrust = ty.Annotated[jax.Array, el.Component("thrust", _f64(1))]
Propellant = ty.Annotated[jax.Array, el.Component("propellant", _f64(1))]
RcsTorque = ty.Annotated[jax.Array, el.Component("rcs_torque", _f64(3))]
Landed = ty.Annotated[jax.Array, el.Component("landed", _f64(1))]
# Touchdown state latched on the contact tick, before contact zeroes velocity.
Touchdown = ty.Annotated[jax.Array, el.Component("touchdown", _f64(3))]  # vz, vh, tilt_deg

Z_HAT = jnp.array([0.0, 0.0, 1.0], dtype=jnp.float64)
INITIAL_THROTTLE = 0.4


def initial_attitude(tilt_rad: float) -> el.Quaternion:
    # Rotating body +Z about +Y by -tilt points the engine back along -X,
    # i.e. retrograde for the vehicle's +X downrange velocity.
    return el.Quaternion.from_axis_angle(jnp.array([0.0, 1.0, 0.0]), -tilt_rad)


def build(cfg: PlantConfig, fault: ActiveFault) -> tuple[el.World, el.System]:
    world = el.World()
    wet_mass = cfg.dry_mass + cfg.propellant
    base_inertia = jnp.array(cfg.inertia_diag, dtype=jnp.float64)
    q0 = initial_attitude(cfg.init_tilt_rad)

    world.spawn(
        [
            el.Body(
                world_pos=el.SpatialTransform(angular=q0, linear=jnp.array(cfg.init_pos, dtype=jnp.float64)),
                world_vel=el.SpatialMotion(
                    angular=jnp.zeros(3, dtype=jnp.float64),
                    linear=jnp.array(cfg.init_vel, dtype=jnp.float64),
                ),
                inertia=el.SpatialInertia(wet_mass, base_inertia),
            ),
            el.C(Throttle, jnp.array([INITIAL_THROTTLE], dtype=jnp.float64)),
            el.C(ThrottleCmd, jnp.array([INITIAL_THROTTLE], dtype=jnp.float64)),
            el.C(AttitudeSetpoint, q0),
            el.C(Thrust, jnp.array([0.0], dtype=jnp.float64)),
            el.C(Propellant, jnp.array([cfg.propellant], dtype=jnp.float64)),
            el.C(RcsTorque, jnp.zeros(3, dtype=jnp.float64)),
            el.C(Landed, jnp.array([0.0], dtype=jnp.float64)),
            el.C(Touchdown, jnp.zeros(3, dtype=jnp.float64)),
        ],
        name="lander",
    )

    tau = cfg.engine_tau * (fault.magnitude if fault.is_("engine_lag") else 1.0)
    alpha = min(S.SIM_DT / tau, 1.0)
    clamp_throttle = not fault.is_("throttle_unclamped")
    thrust_full = cfg.max_thrust * cfg.thrust_scale

    @el.map
    def engine(throttle: Throttle, cmd: ThrottleCmd, prop: Propellant, landed: Landed) -> tuple[Throttle, Thrust]:
        target = jnp.clip(cmd[0], S.THROTTLE_MIN, S.THROTTLE_MAX) if clamp_throttle else cmd[0]
        actual = throttle[0] + (target - throttle[0]) * alpha
        on = jnp.logical_and(prop[0] > 0.0, landed[0] < 0.5)
        actual = jnp.where(on, actual, 0.0)
        return jnp.array([actual]), jnp.array([actual * thrust_full])

    flow_g = S.LUNAR_G if fault.is_("mdot_lunar_g") else S.G0
    update_mass = not fault.is_("mass_not_updated")

    @el.map
    def mass_props(thrust: Thrust, prop: Propellant) -> tuple[Propellant, el.Inertia]:
        burned = thrust[0] / (cfg.isp * flow_g) * cfg.mass_flow_dt
        next_prop = jnp.maximum(prop[0] - burned, 0.0)
        mass = cfg.dry_mass + next_prop if update_mass else wet_mass
        return jnp.array([next_prop]), el.SpatialInertia(mass, base_inertia * (mass / wet_mass))

    # The RCS loop is designed against the spec-sheet inertia (not the plant's).
    design_inertia = jnp.array(S.INERTIA_DIAG_KGM2, dtype=jnp.float64)
    omega_n, zeta = 2.5, 0.9
    k_p = design_inertia * omega_n**2
    k_d = design_inertia * 2.0 * zeta * omega_n
    torque_limit = jnp.full(3, S.RCS_TORQUE_LIMIT_NM)
    torque_sign = -1.0 if fault.is_("rcs_torque_sign") else 1.0

    @el.map
    def attitude_control(pos: el.WorldPos, vel: el.WorldVel, setpoint: AttitudeSetpoint, landed: Landed) -> RcsTorque:
        q_err = (pos.angular().inverse() * setpoint).vector()
        hemisphere = jnp.where(q_err[3] >= 0.0, 1.0, -1.0)
        body_rate = pos.angular().inverse() @ vel.angular()
        torque = hemisphere * 2.0 * q_err[:3] * k_p - body_rate * k_d
        torque = torque_sign * jnp.clip(torque, -torque_limit, torque_limit)
        return jnp.where(landed[0] > 0.5, jnp.zeros(3), torque)

    g_sign = -1.0 if fault.is_("gravity_sign") else 1.0
    relief_sign = -1.0 if fault.is_("centrifugal_sign") else 1.0

    def gravity_force(force: el.Force, inertia: el.Inertia, vel: el.WorldVel) -> el.Force:
        # Flat-world stand-in for orbital motion: horizontal speed v relieves
        # v^2 / R of gravity.
        v_h_sq = jnp.sum(vel.linear()[:2] ** 2)
        g_eff = g_sign * (cfg.gravity - relief_sign * v_h_sq / S.R_MOON_M)
        return force + el.SpatialForce(linear=-Z_HAT * g_eff * inertia.mass())

    lunar_gravity = el.map(gravity_force)

    @el.map
    def main_thrust(thrust: Thrust, force: el.Force, pos: el.WorldPos) -> el.Force:
        body = Z_HAT * thrust[0]
        world_dir = body if fault.is_("thrust_world_frame") else pos.angular() @ body
        return force + el.SpatialForce(linear=world_dir)

    @el.map
    def rcs(torque: RcsTorque, force: el.Force, pos: el.WorldPos) -> el.Force:
        return force + el.SpatialForce(torque=pos.angular() @ torque)

    @el.map
    def ground_contact(
        pos: el.WorldPos, vel: el.WorldVel, landed: Landed, touchdown: Touchdown
    ) -> tuple[el.WorldPos, el.WorldVel, Landed, Touchdown]:
        contact = pos.linear()[2] <= S.PAD_HEIGHT_M
        was_landed = landed[0] > 0.5
        landed_now = jnp.logical_or(was_landed, contact)
        first_contact = jnp.logical_and(~was_landed, contact)
        body_up = pos.angular() @ Z_HAT
        tilt_deg = jnp.rad2deg(jnp.arccos(jnp.clip(body_up[2], -1.0, 1.0)))
        state = jnp.array([vel.linear()[2], jnp.linalg.norm(vel.linear()[:2]), tilt_deg])
        latched = jnp.where(first_contact, state, touchdown)
        linear_pos = jnp.where(landed_now, pos.linear().at[2].set(S.PAD_HEIGHT_M), pos.linear())
        return (
            el.SpatialTransform(linear=linear_pos, angular=pos.angular()),
            el.SpatialMotion(
                linear=jnp.where(landed_now, jnp.zeros(3), vel.linear()),
                angular=jnp.where(landed_now, jnp.zeros(3), vel.angular()),
            ),
            jnp.array([jnp.where(landed_now, 1.0, 0.0)]),
            latched,
        )

    effectors = lunar_gravity | main_thrust | rcs
    if fault.is_("gravity_twice"):
        effectors = el.map(gravity_force) | effectors

    return world, (
        engine
        | mass_props
        | attitude_control
        | el.six_dof(sys=effectors, integrator=el.Integrator.SemiImplicit)
        | ground_contact
    )

