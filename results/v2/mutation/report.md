# Kestrel mutation campaign: mutation

- **Negative controls**: 40 clean runs, **0 false positives**
- **must_catch**: 1500 / 1500 caught (100.0%), **0 false negatives**
- **should_catch**: 160 / 160 caught (100.0%)
- **may_escape** (inside documented bands, or permitted by spec): 66 / 300 caught (22.0%); 0 left the trajectory unchanged
- **must_be_equivalent**: 40 / 40 confirmed identical to their clean twin
- All seeded runs except declared equivalents: 1726 / 1960 caught (88.1%)

## By fault variant

| variant | layer | requirement (runs) | caught | false neg. | flagged by |
|---|---|---|---:|---:|---|
| `gravity_earth` | param | must_catch 40 | 40/40 |  | newton (40), landing (40) |
| `gravity_scale@0.005` | param | must_catch 40 | 40/40 |  | newton (40) |
| `gravity_scale@0.02` | param | must_catch 40 | 40/40 |  | newton (40) |
| `gravity_scale@0.1` | param | must_catch 40 | 40/40 |  | newton (40), landing (22) |
| `isp_low@0.005` | param | may_escape 40 | 6/40 |  | mass_flow (6) |
| `isp_low@0.01` | param | must_catch 10, may_escape 30 | 16/40 |  | mass_flow (16) |
| `isp_low@0.02` | param | must_catch 30, may_escape 10 | 36/40 |  | mass_flow (36) |
| `isp_low@0.05` | param | must_catch 40 | 40/40 |  | mass_flow (40) |
| `isp_low@0.2` | param | must_catch 40 | 40/40 |  | mass_flow (40) |
| `thrust_lbf` | param | must_catch 40 | 40/40 |  | engine_model (40), thrust_magnitude (40), landing (40) |
| `thrust_scale@-0.02` | param | must_catch 15, may_escape 25 | 18/40 |  | engine_model (18), thrust_magnitude (1) |
| `thrust_scale@-0.05` | param | must_catch 40 | 40/40 |  | engine_model (40), thrust_magnitude (21), landing (6) |
| `thrust_scale@-0.1` | param | must_catch 40 | 40/40 |  | engine_model (40), thrust_magnitude (40), landing (31) |
| `thrust_scale@0.1` | param | must_catch 40 | 40/40 |  | engine_model (40), thrust_magnitude (40) |
| `dry_mass_lb` | param | must_catch 40 | 40/40 |  | thrust_magnitude (40), initial_state (40), landing (40) |
| `propellant_short@0.02` | param | must_catch 6, may_escape 34 | 8/40 |  | initial_state (8) |
| `propellant_short@0.1` | param | must_catch 40 | 40/40 |  | initial_state (40) |
| `propellant_short@0.3` | param | must_catch 40 | 40/40 |  | initial_state (40) |
| `propellant_short@0.6` | param | must_catch 40 | 40/40 |  | initial_state (40), landing (40) |
| `tilt_deg_as_rad` | param | must_catch 39, may_escape 1 | 39/40 |  | initial_state (39), attitude_envelope (36), attitude_tracking (14), landing (8) |
| `vz_sign` | param | must_catch 40 | 40/40 |  | initial_state (40), landing (11) |
| `mass_flow_dt` | param | must_catch 40 | 40/40 |  | mass_flow (40), landing (11) |
| `inertia_x10` | param | should_catch 40 | 40/40 |  | attitude_tracking (40), thrust_direction (40), landing (39) |
| `site_offset@100` | param | must_catch 40 | 40/40 |  | landing (40) |
| `thrust_world_frame` | model | must_catch 40 | 40/40 |  | newton (40), landing (40) |
| `mass_not_updated` | model | must_catch 40 | 40/40 |  | mass_bookkeeping (40), thrust_magnitude (11) |
| `gravity_twice` | model | must_catch 40 | 40/40 |  | newton (40), landing (40) |
| `gravity_sign` | model | must_catch 40 | 40/40 |  | newton (40), landing (40) |
| `rcs_torque_sign` | model | should_catch 40 | 40/40 |  | attitude_envelope (40), attitude_tracking (40), thrust_direction (40), landing (40) |
| `throttle_unclamped` | model | must_be_equivalent 40 | 0/40 |  | - |
| `centrifugal_sign` | model | must_catch 40 | 40/40 |  | newton (40) |
| `engine_lag@1.5` | model | must_catch 40 | 40/40 |  | engine_response (40) |
| `engine_lag@10` | model | must_catch 40 | 40/40 |  | engine_response (40) |
| `mdot_lunar_g` | model | must_catch 40 | 40/40 |  | mass_flow (40), landing (40) |
| `quat_wxyz` | fsw | must_catch 40 | 40/40 |  | attitude_envelope (40), attitude_tracking (40), thrust_direction (40), landing (40) |
| `quat_conjugate` | fsw | must_catch 40 | 40/40 |  | attitude_envelope (40), thrust_direction (40), landing (40) |
| `fsw_gravity_earth` | fsw | should_catch 40 | 40/40 |  | landing (40) |
| `crossrange_sign` | fsw | should_catch 40 | 40/40 |  | landing (40) |
| `mass_estimate_dry` | fsw | must_catch 40 | 40/40 |  | thrust_magnitude (40), landing (36) |
| `stale_command@1` | fsw | may_escape 40 | 0/40 |  | - |
| `stale_command@2` | fsw | must_catch 40 | 40/40 |  | command_latency (40) |
| `stale_command@10` | fsw | must_catch 40 | 40/40 |  | command_latency (40) |
| `altimeter_feet` | sensor | must_catch 40 | 40/40 |  | nav_residual (40), landing (40) |
| `altimeter_bias@0.5` | sensor | may_escape 40 | 0/40 |  | - |
| `altimeter_bias@1` | sensor | may_escape 40 | 3/40 |  | nav_residual (3) |
| `altimeter_bias@2` | sensor | may_escape 40 | 40/40 |  | nav_residual (40) |
| `altimeter_bias@10` | sensor | must_catch 40 | 40/40 |  | nav_residual (40) |
| `altimeter_bias@50` | sensor | must_catch 40 | 40/40 |  | nav_residual (40), landing (23) |
| `velocity_xy_swap` | sensor | must_catch 40 | 40/40 |  | attitude_envelope (40), nav_residual (40), landing (40) |
| `altimeter_stuck@100` | sensor | must_catch 40 | 40/40 |  | nav_residual (40), landing (20) |

## By check

| check | seeded runs flagged | sole detector |
|---|---:|---:|
| `finite` | 0 | 0 |
| `newton` | 320 | 138 |
| `mass_bookkeeping` | 40 | 29 |
| `mass_flow` | 218 | 167 |
| `engine_model` | 178 | 36 |
| `engine_response` | 80 | 80 |
| `attitude_envelope` | 196 | 0 |
| `attitude_tracking` | 134 | 0 |
| `nav_residual` | 243 | 120 |
| `command_latency` | 80 | 80 |
| `thrust_magnitude` | 233 | 4 |
| `thrust_direction` | 160 | 0 |
| `initial_state` | 247 | 120 |
| `landing` | 847 | 120 |

## Clean-run check statistics (min .. max)

- `finite`: 2.22045e-16 .. 3.33067e-16
- `newton`: 3.85119e-13 .. 3.96014e-13
- `mass_bookkeeping`: 0 .. 4.54747e-13
- `mass_flow`: 0.990338 .. 1.00998
- `engine_model`: 6861.75 .. 7134.64
- `engine_response`: 0.0809117 .. 0.119613
- `attitude_envelope`: 16.3843 .. 26.7973
- `attitude_tracking`: 2.69238 .. 4.83919
- `nav_residual`: 3.78105 .. 5.15233
- `command_latency`: 0 .. 0
- `thrust_magnitude`: 0.969508 .. 1.03516
- `thrust_direction`: 1.46276 .. 2.09879
- `initial_state`: 0 .. 0
- `landing`: 0 .. 0
