# Kestrel mutation campaign: mutation

- Clean runs: **40**, false positives: **1** (landing: 1)
- Seeded runs: **2000** (50 fault variants), equivalent: 39, scoreable: 1961
- Caught: **1731 / 1961 = 88.3%** (of which 0 crashed the run), escaped: 230
- Variants: killed 41, partial 9

## By fault variant

| variant | layer | caught | equiv. | escaped | status | flagged by |
|---|---|---:|---:|---:|---|---|
| `gravity_earth` | param | 40/40 | 0 | 0 | killed | newton (40), thrust_magnitude (40), thrust_direction (40), landing (40) |
| `gravity_scale@0.005` | param | 40/40 | 0 | 0 | killed | newton (40) |
| `gravity_scale@0.02` | param | 40/40 | 0 | 0 | killed | newton (40) |
| `gravity_scale@0.1` | param | 40/40 | 0 | 0 | killed | newton (40), landing (22) |
| `isp_low@0.005` | param | 6/40 | 0 | 34 | partial | mass_flow (6), landing (1) |
| `isp_low@0.01` | param | 16/40 | 0 | 24 | partial | mass_flow (16), landing (1) |
| `isp_low@0.02` | param | 36/40 | 0 | 4 | partial | mass_flow (36), landing (1) |
| `isp_low@0.05` | param | 40/40 | 0 | 0 | killed | mass_flow (40), landing (1) |
| `isp_low@0.2` | param | 40/40 | 0 | 0 | killed | mass_flow (40), landing (1) |
| `thrust_lbf` | param | 40/40 | 0 | 0 | killed | engine_model (40), thrust_magnitude (40), landing (40) |
| `thrust_scale@-0.02` | param | 18/40 | 0 | 22 | partial | engine_model (18), thrust_magnitude (1) |
| `thrust_scale@-0.05` | param | 40/40 | 0 | 0 | killed | engine_model (40), thrust_magnitude (21), landing (5) |
| `thrust_scale@-0.1` | param | 40/40 | 0 | 0 | killed | engine_model (40), thrust_magnitude (40), landing (30) |
| `thrust_scale@0.1` | param | 40/40 | 0 | 0 | killed | engine_model (40), thrust_magnitude (40), landing (40) |
| `dry_mass_lb` | param | 40/40 | 0 | 0 | killed | thrust_magnitude (40), initial_state (40), landing (40) |
| `propellant_short@0.02` | param | 9/40 | 0 | 31 | partial | initial_state (8), landing (1) |
| `propellant_short@0.1` | param | 40/40 | 0 | 0 | killed | initial_state (40), landing (1) |
| `propellant_short@0.3` | param | 40/40 | 0 | 0 | killed | initial_state (40), landing (1) |
| `propellant_short@0.6` | param | 40/40 | 0 | 0 | killed | initial_state (40), landing (40) |
| `tilt_deg_as_rad` | param | 39/40 | 0 | 1 | partial | initial_state (37), attitude_envelope (36), attitude_tracking (14), landing (6) |
| `vz_sign` | param | 40/40 | 0 | 0 | killed | initial_state (40), landing (40) |
| `mass_flow_dt` | param | 40/40 | 0 | 0 | killed | mass_flow (40), landing (17) |
| `inertia_x10` | param | 40/40 | 0 | 0 | killed | attitude_tracking (40), landing (39), thrust_direction (38) |
| `site_offset@100` | param | 40/40 | 0 | 0 | killed | landing (40) |
| `thrust_world_frame` | model | 40/40 | 0 | 0 | killed | newton (40), landing (40) |
| `mass_not_updated` | model | 40/40 | 0 | 0 | killed | mass_bookkeeping (40), thrust_magnitude (11) |
| `gravity_twice` | model | 40/40 | 0 | 0 | killed | newton (40), landing (40) |
| `gravity_sign` | model | 40/40 | 0 | 0 | killed | newton (40), landing (40) |
| `rcs_torque_sign` | model | 40/40 | 0 | 0 | killed | attitude_envelope (40), attitude_tracking (40), thrust_direction (40), landing (40) |
| `throttle_unclamped` | model | 1/1 | 39 | 0 | killed | landing (1) |
| `centrifugal_sign` | model | 40/40 | 0 | 0 | killed | newton (40), landing (1) |
| `engine_lag@1.5` | model | 40/40 | 0 | 0 | killed | engine_response (40), landing (1) |
| `engine_lag@10` | model | 40/40 | 0 | 0 | killed | engine_response (40), landing (1) |
| `mdot_lunar_g` | model | 40/40 | 0 | 0 | killed | mass_flow (40), landing (40) |
| `quat_wxyz` | fsw | 40/40 | 0 | 0 | killed | attitude_envelope (40), attitude_tracking (40), thrust_direction (40), landing (40) |
| `quat_conjugate` | fsw | 40/40 | 0 | 0 | killed | attitude_envelope (40), thrust_direction (40), landing (40) |
| `fsw_gravity_earth` | fsw | 40/40 | 0 | 0 | killed | landing (40) |
| `crossrange_sign` | fsw | 40/40 | 0 | 0 | killed | landing (40) |
| `mass_estimate_dry` | fsw | 40/40 | 0 | 0 | killed | thrust_magnitude (40), landing (36) |
| `stale_command@1` | fsw | 1/40 | 0 | 39 | partial | landing (1) |
| `stale_command@2` | fsw | 40/40 | 0 | 0 | killed | command_latency (40), landing (1) |
| `stale_command@10` | fsw | 40/40 | 0 | 0 | killed | command_latency (40), landing (1) |
| `altimeter_feet` | sensor | 40/40 | 0 | 0 | killed | nav_residual (40), landing (40) |
| `altimeter_bias@0.5` | sensor | 1/40 | 0 | 39 | partial | landing (1) |
| `altimeter_bias@1` | sensor | 4/40 | 0 | 36 | partial | nav_residual (3), landing (1) |
| `altimeter_bias@2` | sensor | 40/40 | 0 | 0 | killed | nav_residual (40), landing (1) |
| `altimeter_bias@10` | sensor | 40/40 | 0 | 0 | killed | nav_residual (40), landing (1) |
| `altimeter_bias@50` | sensor | 40/40 | 0 | 0 | killed | nav_residual (40), landing (32) |
| `velocity_xy_swap` | sensor | 40/40 | 0 | 0 | killed | attitude_envelope (40), nav_residual (40), landing (40) |
| `altimeter_stuck@100` | sensor | 40/40 | 0 | 0 | killed | nav_residual (40), landing (35) |

## By check

| check | seeded runs flagged | sole detector |
|---|---:|---:|
| `finite` | 0 | 0 |
| `newton` | 320 | 137 |
| `mass_bookkeeping` | 40 | 29 |
| `mass_flow` | 218 | 156 |
| `engine_model` | 178 | 36 |
| `engine_response` | 80 | 78 |
| `attitude_envelope` | 196 | 2 |
| `attitude_tracking` | 134 | 1 |
| `nav_residual` | 243 | 94 |
| `command_latency` | 80 | 78 |
| `thrust_magnitude` | 273 | 4 |
| `thrust_direction` | 198 | 0 |
| `initial_state` | 245 | 89 |
| `landing` | 961 | 125 |

## Clean-run check statistics (min .. max)

- `finite`: 2.22045e-16 .. 3.33067e-16
- `newton`: 3.85399e-13 .. 3.96316e-13
- `mass_bookkeeping`: 0 .. 4.54747e-13
- `mass_flow`: 0.990338 .. 1.00998
- `engine_model`: 6861.75 .. 7134.64
- `engine_response`: 0.0809117 .. 0.119613
- `attitude_envelope`: 16.3998 .. 26.7602
- `attitude_tracking`: 2.67114 .. 4.76786
- `nav_residual`: 3.78105 .. 5.15233
- `command_latency`: 0 .. 0
- `thrust_magnitude`: 0.969463 .. 1.03564
- `thrust_direction`: 1.1254 .. 2.08481
- `initial_state`: 0 .. 0
- `landing`: 0 .. 1
