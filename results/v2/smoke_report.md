# Kestrel mutation campaign: smoke

- **Negative controls**: 1 clean runs, **0 false positives**
- **must_catch**: 37 / 37 caught (100.0%), **0 false negatives**
- **should_catch**: 4 / 4 caught (100.0%)
- **may_escape** (inside documented bands, or permitted by spec): 2 / 8 caught (25.0%); 0 left the trajectory unchanged
- **must_be_equivalent**: 1 / 1 confirmed identical to their clean twin
- All seeded runs except declared equivalents: 43 / 49 caught (87.8%)

## By fault variant

| variant | layer | requirement (runs) | caught | false neg. | flagged by |
|---|---|---|---:|---:|---|
| `gravity_earth` | param | must_catch 1 | 1/1 |  | newton (1), landing (1) |
| `gravity_scale@0.005` | param | must_catch 1 | 1/1 |  | newton (1) |
| `gravity_scale@0.02` | param | must_catch 1 | 1/1 |  | newton (1) |
| `gravity_scale@0.1` | param | must_catch 1 | 1/1 |  | newton (1), landing (1) |
| `isp_low@0.005` | param | may_escape 1 | 0/1 |  | - |
| `isp_low@0.01` | param | may_escape 1 | 0/1 |  | - |
| `isp_low@0.02` | param | may_escape 1 | 1/1 |  | mass_flow (1) |
| `isp_low@0.05` | param | must_catch 1 | 1/1 |  | mass_flow (1) |
| `isp_low@0.2` | param | must_catch 1 | 1/1 |  | mass_flow (1) |
| `thrust_lbf` | param | must_catch 1 | 1/1 |  | engine_model (1), thrust_magnitude (1), landing (1) |
| `thrust_scale@-0.02` | param | must_catch 1 | 1/1 |  | engine_model (1) |
| `thrust_scale@-0.05` | param | must_catch 1 | 1/1 |  | engine_model (1), thrust_magnitude (1) |
| `thrust_scale@-0.1` | param | must_catch 1 | 1/1 |  | engine_model (1), thrust_magnitude (1), landing (1) |
| `thrust_scale@0.1` | param | must_catch 1 | 1/1 |  | engine_model (1), thrust_magnitude (1) |
| `dry_mass_lb` | param | must_catch 1 | 1/1 |  | thrust_magnitude (1), initial_state (1), landing (1) |
| `propellant_short@0.02` | param | may_escape 1 | 0/1 |  | - |
| `propellant_short@0.1` | param | must_catch 1 | 1/1 |  | initial_state (1) |
| `propellant_short@0.3` | param | must_catch 1 | 1/1 |  | initial_state (1) |
| `propellant_short@0.6` | param | must_catch 1 | 1/1 |  | initial_state (1), landing (1) |
| `tilt_deg_as_rad` | param | must_catch 1 | 1/1 |  | attitude_envelope (1), initial_state (1) |
| `vz_sign` | param | must_catch 1 | 1/1 |  | initial_state (1) |
| `mass_flow_dt` | param | must_catch 1 | 1/1 |  | mass_flow (1) |
| `inertia_x10` | param | should_catch 1 | 1/1 |  | attitude_tracking (1), thrust_direction (1), landing (1) |
| `site_offset@100` | param | must_catch 1 | 1/1 |  | landing (1) |
| `thrust_world_frame` | model | must_catch 1 | 1/1 |  | newton (1), landing (1) |
| `mass_not_updated` | model | must_catch 1 | 1/1 |  | mass_bookkeeping (1), thrust_magnitude (1) |
| `gravity_twice` | model | must_catch 1 | 1/1 |  | newton (1), landing (1) |
| `gravity_sign` | model | must_catch 1 | 1/1 |  | newton (1), landing (1) |
| `rcs_torque_sign` | model | should_catch 1 | 1/1 |  | attitude_envelope (1), attitude_tracking (1), thrust_direction (1), landing (1) |
| `throttle_unclamped` | model | must_be_equivalent 1 | 0/1 |  | - |
| `centrifugal_sign` | model | must_catch 1 | 1/1 |  | newton (1) |
| `engine_lag@1.5` | model | must_catch 1 | 1/1 |  | engine_response (1) |
| `engine_lag@10` | model | must_catch 1 | 1/1 |  | engine_response (1) |
| `mdot_lunar_g` | model | must_catch 1 | 1/1 |  | mass_flow (1), landing (1) |
| `quat_wxyz` | fsw | must_catch 1 | 1/1 |  | attitude_envelope (1), attitude_tracking (1), thrust_direction (1), landing (1) |
| `quat_conjugate` | fsw | must_catch 1 | 1/1 |  | attitude_envelope (1), thrust_direction (1), landing (1) |
| `fsw_gravity_earth` | fsw | should_catch 1 | 1/1 |  | landing (1) |
| `crossrange_sign` | fsw | should_catch 1 | 1/1 |  | landing (1) |
| `mass_estimate_dry` | fsw | must_catch 1 | 1/1 |  | thrust_magnitude (1), landing (1) |
| `stale_command@1` | fsw | may_escape 1 | 0/1 |  | - |
| `stale_command@2` | fsw | must_catch 1 | 1/1 |  | command_latency (1) |
| `stale_command@10` | fsw | must_catch 1 | 1/1 |  | command_latency (1) |
| `altimeter_feet` | sensor | must_catch 1 | 1/1 |  | nav_residual (1), landing (1) |
| `altimeter_bias@0.5` | sensor | may_escape 1 | 0/1 |  | - |
| `altimeter_bias@1` | sensor | may_escape 1 | 0/1 |  | - |
| `altimeter_bias@2` | sensor | may_escape 1 | 1/1 |  | nav_residual (1) |
| `altimeter_bias@10` | sensor | must_catch 1 | 1/1 |  | nav_residual (1) |
| `altimeter_bias@50` | sensor | must_catch 1 | 1/1 |  | nav_residual (1), landing (1) |
| `velocity_xy_swap` | sensor | must_catch 1 | 1/1 |  | attitude_envelope (1), nav_residual (1), landing (1) |
| `altimeter_stuck@100` | sensor | must_catch 1 | 1/1 |  | nav_residual (1), landing (1) |

## By check

| check | seeded runs flagged | sole detector |
|---|---:|---:|
| `finite` | 0 | 0 |
| `newton` | 8 | 3 |
| `mass_bookkeeping` | 1 | 0 |
| `mass_flow` | 5 | 4 |
| `engine_model` | 5 | 1 |
| `engine_response` | 2 | 2 |
| `attitude_envelope` | 5 | 0 |
| `attitude_tracking` | 3 | 0 |
| `nav_residual` | 6 | 2 |
| `command_latency` | 2 | 2 |
| `thrust_magnitude` | 7 | 0 |
| `thrust_direction` | 4 | 0 |
| `initial_state` | 6 | 3 |
| `landing` | 22 | 3 |

## Clean-run check statistics (min .. max)

- `finite`: 2.22045e-16 .. 2.22045e-16
- `newton`: 3.92982e-13 .. 3.92982e-13
- `mass_bookkeeping`: 0 .. 0
- `mass_flow`: 0.994175 .. 0.994175
- `engine_model`: 6885.39 .. 6885.39
- `engine_response`: 0.102391 .. 0.102391
- `attitude_envelope`: 21.7305 .. 21.7305
- `attitude_tracking`: 4.07326 .. 4.07326
- `nav_residual`: 4.22118 .. 4.22118
- `command_latency`: 0 .. 0
- `thrust_magnitude`: 0.985363 .. 0.985363
- `thrust_direction`: 1.75048 .. 1.75048
- `initial_state`: 0 .. 0
- `landing`: 0 .. 0
