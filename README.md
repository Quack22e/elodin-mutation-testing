# Breaking an Elodin sim on purpose:

The email suggestion was to *build your own project with Elodin, start from the
`monte-carlo` example, look at `apollo-lander`, then point a mutation-testing
habit at a sim: break parameters on purpose and see whether your checks catch
it.*

This repo does that. **Kestrel** is a 6-DOF lunar-lander sim built with
[Elodin](https://github.com/elodin-sys/elodin). Its flight software runs as a
separate software-in-the-loop (SITL) process, and every campaign runs through
the official `elodin monte-carlo run`. I intentionally seeded 50 realistic faults including 
wrong units, wrong constants, physics bugs, flight-software bugs and
sensor faults. Each was flown across 40 dispersed flights, and 14 independent
checks decided which flights were broken.

| | Caught by the checks |
|---|---:|
| **Faults that must be caught** (1,500 broken flights) | **1,500 (100 %)** |
| **All broken flights** (1,960) | **1,726 (88 %)** |
| **False alarms** on clean flights (240) | **0** |
| Broken *parameters* only: must-catch / all (780 / 960 flights) | 780 (100 %) / 843 (88 %) |

- **"Must be caught" had been determined before the run, only from the fault.** It
  applies when the fault pushes something outside its documented range. Isp
  5 % low must be caught, because no legitimate engine is actually that far off. Isp
  0.5 % low need not be: real engines scatter ±1 %, and a fault hidden inside
  that band leaves no evidence in any of the outputs.
- **Everything that slipped through is a fault smaller than normal scatter**,
  plus one flight whose degrees-as-radians tilt happened to wrap back into
  the valid range (also declared before the run). Detection rises with fault
  size: Isp 0.5 % low was caught 6 / 40 times, 5 % low 40 / 40.
- **40 more broken flights carried a quantifiably harmless fault.** Removing a
  duplicate throttle clamp left every one of them bit-identical to its clean
  twin.
- **Both runners agree exactly.** Elodin's own runner and a stand-in runner
  (`harness/mc.py`) give bit-identical trajectories and identical verdicts on
  all of the 2,280 flights.

## Broken parameters

Each row is one parameter broken the way a person plausibly would break it,
flown across the same 40 dispersed flights.

| What was broken | Caught | Caught by |
|---|---:|---|
| Gravity set to Earth's 9.81 instead of the Moon's 1.622 | 40 / 40 | `newton` |
| Gravity off by 0.5 % / 2 % / 10 % | 40 / 40 / 40 | `newton` (a clean flight satisfies Newton's law to 4e-13 m/s²) |
| Engine Isp low by 0.5 % / 1 % / 2 % / 5 % / 20 % | 6 / 16 / 36 / 40 / 40 | `mass_flow` |
| Max thrust copied from a lbf datasheet, used as N | 40 / 40 | `engine_model`, `thrust_magnitude` |
| Thrust off by −2 % / −5 % / −10 % / +10 % | 18 / 40 / 40 / 40 | `engine_model` |
| Dry mass entered in pounds | 40 / 40 | `initial_state`, `thrust_magnitude` |
| Propellant load short by 2 % / 10 % / 30 % / 60 % | 8 / 40 / 40 / 40 | `initial_state` |
| Initial tilt in degrees passed as radians | 39 / 40 | `initial_state`, `attitude_envelope` |
| Initial vertical velocity sign flipped | 40 / 40 | `initial_state` |
| Propellant integrated at 50 Hz in a 100 Hz sim | 40 / 40 | `mass_flow` |
| Inertia 10× too large | 40 / 40 | `attitude_tracking`, `thrust_direction` |
| Landing-site coordinates 100 m off | 40 / 40 | `landing` |

How to read the partial rows:
- **Isp low, thrust −2 %, propellant short 2 %** trace the detection floor.
  Isp legitimately varies ±1 % between engines. A 0.5 % fault is caught only
  when it lands a flight's Isp outside that band (6 / 40). By 5 % it's caught
  every time. Every flight whose broken value left the band was caught; every
  miss is a flight where the band was not left.
- **Degrees as radians (39 / 40):** the one escape is a flight whose tilt, read
  as radians and wrapped around the circle, happens to land back inside the
  valid start envelope, pointing the right way. That was computed and declared
  *may escape* before the run.

## Beyond parameters

The same experiment, with bugs in other layers:

| Layer | Examples | Must-catch caught |
|---|---|---:|
| Plant physics (`model`) | thrust in world frame instead of body frame; mass never updated as propellant burns; gravity applied twice or upward; RCS torque sign; centrifugal-relief sign; engine lag ×1.5 or ×10; mass flow using lunar g instead of g0 | 320 / 320 |
| Flight software (`fsw`) | quaternion sent scalar-first or conjugated; guidance assumes Earth gravity; crossrange sign; mass estimate omits propellant; commands 2 or 10 cycles late | 200 / 200 |
| Sensors (`sensor`) | altimeter in feet; altimeter biased 10 m or 50 m; vx/vy swapped; altimeter stuck below 100 m | 200 / 200 |

Two results in that layer are worth noting:
- **Below sensor noise, nothing is promised.** An altimeter biased 0.5 m / 1 m /
  2 m was caught 0 / 3 / 40 times, against 0.5 m (1σ) noise and a 6σ limit.
- **Latency the spec permits is not flagged.** One guidance cycle of command
  latency is allowed by the spec and caught 0 / 40 times. Two cycles was
  caught 40 / 40.

## What breaking things uncovered

1. **A real bug in the unbroken flight software.** In the first frozen run, a
   *clean* control flight failed its landing requirement. Guidance
   re-estimated time-to-go from the current descent rate, so a vehicle that
   slowed down kept pushing its own deadline back. It crept down at 0.8 m/s,
   65 m up, and was still airborne at the 210 s cap. The checks were right but
   the design was unfortunately wrong. Time-to-go now only counts down, and the slowest
   flight dropped from 169 s to 150 s. That first run is kept in `results/v1/`.
2. **A run-contract bug that only the official runner exposed.** The first
   `elodin monte-carlo run` marked every flight invalid. The CLI passes each
   run's context as a path relative to the campaign directory, while the SITL
   process was launched from `sim/`. Fixed, and the stand-in runner now writes
   relative paths too, so it can't hide that again.
3. **Elodin v0.20.0 on macOS needs Homebrew's `ffmpeg@8` at runtime.** The
   `elodin` binary links `/opt/homebrew/opt/ffmpeg@8/lib` and won't start
   without it. `dist-workspace.toml` lists `ffmpeg@8` as a Homebrew build
   dependency, but the quickstart guide fails to mention it.

## How it works

```text
campaigns/*.toml ──► elodin monte-carlo run ──► sim/main.py ──────────────────► checks/oracles.py ──► verdict
  params.* (dispersions)                         ├─ vehicle.py  6-DOF plant       (post_run hook)       per flight
  meta.fault (which bug)                         ├─ sensors.py  nav sensor + noise
                                                 ├─ controller.py ⇄ fsw.py  SITL flight software (UDP, lockstep)
                                                 └─ flight recorder ──► telemetry.npz, result.json
```

**From the Elodin examples.**
- *From `monte-carlo`:* parameters declared with `el.monte_carlo.params_spec`;
  a separate flight-software process launched with `world.recipe` and bridged
  over UDP from `post_step`; per-run `el.monte_carlo.result(...)`; post-run
  and post-campaign hooks.
- *From `apollo-lander`:* the 6-DOF structure (`el.six_dof` with gravity, a
  body-axis throttleable engine and RCS torque as JAX systems); propellant
  burn-down rewriting the rigid-body mass; touchdown state latched on the
  contact tick.
- *Kestrel itself is original:* a fictional ~1.5 t lander flying ZEM/ZEV
  guidance from ~1.5 km to touchdown in about two minutes.

**How faults get in.** Each run's fault comes from the campaign plan's
`meta.fault` column. `campaigns/mutation.toml` uses Elodin's
`[meta_sweep]` to cross all 50 fault variants with the same 40 dispersion
samples, plus `fault = "none"`. Every broken flight therefore has a clean twin
with identical dispersions and identical sensor noise. Faults live in the
layer they belong to:
- **param**: wrong inputs, in `vehicle.resolve_plant`.
- **model**: physics bugs, in the JAX systems.
- **fsw**: flight-software bugs, in the SITL process.
- **sensor**: navigation faults, in `sensors.py`.

**The checks** (`checks/oracles.py`) see only the flight recording and the
spec sheet (`sim/spec_sheet.py`). They never see which fault was seeded or
the flight's sampled inputs.

| Check | Asserts |
|---|---|
| `newton` | every tick's Δv equals (body-axis thrust) / mass + gravity, to 1e-6 m/s² |
| `mass_bookkeeping`, `mass_flow` | rigid-body mass − propellant is constant; propellant used matches the rocket equation within the Isp band |
| `engine_model`, `engine_response` | thrust per unit throttle within the rated ±2 %; throttle time constant, *identified from the log by least squares*, within spec |
| `attitude_envelope`, `attitude_tracking` | tilt ≤ 45°, rates ≤ 20°/s; vehicle within 10° of the commanded attitude |
| `nav_residual`, `command_latency` | flight-software navigation agrees with truth within 6σ; commands at most one cycle old |
| `thrust_magnitude`, `thrust_direction` | delivered thrust matches what guidance commanded, in size and direction |
| `initial_state` | first recorded state inside the documented start envelope, pitched back |
| `landing`, `finite` | touchdown ≤ 2 m/s vertical and ≤ 0.6 m/s horizontal, tilt ≤ 8°, miss ≤ 30 m, fuel ≥ 5 kg, time ≤ 200 s; no NaNs, unit quaternions |

**Keeping the numbers honest.** These are the testing habits I use in
[Aptio](https://github.com/Quack22e/APTIO-README):
- **Declared requirements.** `faults.requirement()` labels every flight
  `must_catch`, `should_catch`, `may_escape`, `must_be_equivalent` or
  `must_pass` from the fault and the flight's inputs alone, without running
  the sim. `should_catch` covers real defects whose visibility depends on
  closed-loop dynamics (inertia ×10, RCS sign, crossrange sign, FSW gravity);
  all 160 were caught.
- **Negative controls and a holdout.** The 40-flight dev set was used to
  develop the flight software. The 200-flight clean holdout was never used for
  tuning. Check tolerances come from the spec sheet and were set before the
  first run.
- **Three-valued verdicts.** A check can fail, pass, or *be unable to decide*.
  "Couldn't decide" should never and does not ever count as a catch.
- **Versions kept, not overwritten.** v1 (`results/v1/`) has its flaw in it;
  v2 is in `results/v2/` (stand-in runner) and `results/v2-cli/` (Elodin's
  runner).

**Cross-check against my Aptio verifier.** I also ran my Aptio verifier, a
general-purpose checker for numerical claims in financial work, unmodified on
the same flights. It isn't included here because Aptio is private.

- **Its result:** 0 false alarms on the 240 clean flights, and 855 of the
  1,500 must-catch faults (57 %), none of which the checks above missed.
- **Why it caught fewer:** it only tests whether reported numbers add up. So
  it catches wrong constants such as gravity, Isp or thrust, but not wrong
  inputs or behaviour bugs, where the arithmetic stays consistent.

Details are available on request.

## Run it

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

One flight, headless, nominal parameters:

```bash
ELODIN_NON_INTERACTIVE=1 .venv/bin/python sim/main.py run
```

The clean holdout (~2.5 min) and the full mutation campaign (~23 min on 8
workers). Run these from the repo root; the CLI picks up `.venv` by itself:

```bash
elodin monte-carlo run sim/main.py --campaign campaigns/campaign.toml --spec campaigns/clean.toml --out dbs/clean
```

```bash
elodin monte-carlo run sim/main.py --campaign campaigns/campaign.toml --spec campaigns/mutation.toml --out dbs/mutation
```

Each campaign writes `results.csv` (every check's verdict per flight) and
`post_campaign/report.md` (the scorecard). Other options:
- **macOS:** install the runtime dependency first, with `brew install ffmpeg@8`.
- **No CLI:** `harness/mc.py run ...` takes the same arguments and gives the
  same results.
- **Adding a fault:** add it to `sim/faults.py` with its declared requirement,
  implement it in its layer, then run `python harness/make_specs.py`.

## Layout

```text
sim/        main.py (Elodin entry, SITL bridge, flight recorder), vehicle.py (6-DOF plant), fsw.py + controller.py
            (SITL flight software), sensors.py, faults.py (the 50 seeded faults), spec_sheet.py (documented numbers)
checks/     oracles.py: the 14 checks
campaigns/  campaign.toml; dev / clean / mutation / smoke specs; hooks/score.py (post_run), hooks/report.py (post_campaign)
harness/    mc.py (stand-in runner), make_specs.py (regenerates the specs from the spec sheet and fault catalog)
results/    v1/, v2/, v2-cli/ reports
```

## Limitations

- Every fault here is one I knew how to make. The results cover these 50, not
  defect classes absent from `sim/faults.py`.
- The checks and the sim share the spec sheet, so an error *in the spec sheet*
  (a wrong documented gravity) is invisible to both by construction.
- The physics is deliberately simple: a flat Moon, no slosh, no gimbal, and
  cold-gas RCS without propellant.
- There's no editor schematic yet, so `elodin editor sim/main.py` shows raw
  components.
- `apollo-lander` was studied for its structure but not run. Pointing this
  harness at it is the obvious next step.
