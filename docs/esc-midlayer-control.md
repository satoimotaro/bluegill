# ESC-side middle-layer control — RPM governor, ramp limiter, temperature (design)

Status: **design / feasibility (2026-07-15)**. Not scheduled; **do not develop yet** — this doc
exists so other workers can see the intent and the source grounding before any branch is cut.
All firmware here is **bench-pending** (needs the low-KV motor + thermal watch, like every BlueGill
drive change). Grounded in the pinned `vendor/bluejay` (tag v0.21.0) and the current BlueGill
overlays in `src/` — file:symbol citations below are verified, not assumed.

## Goal (the user's ask)

Move the **middle layer** of motor control off the host/Pico and **onto the ESC firmware itself**,
so `pico-esc-tool` becomes a thin driver/library and the ESC is "easy to use": you command it and it
regulates. Three features, keeping **raw throttle/DShot input** as the default:

1. **Closed-loop RPM governor** — command an RPM setpoint; the ESC holds it (PID/PI on its own eRPM).
2. **Ramp / slew limiter** — smoothly bound how fast commanded speed changes, so no rapid step causes
   a mechanical/thrust impact.
3. **Temperature check / limiting** — read board temp and limit power at a threshold; report it.

**Design headline:** on this EFM8 hardware, all three are *in-family* — two already have working
firmware machinery to extend, and the third (the RPM governor) has the feedback, the actuator, and a
dormant BLHeli ancestor to model. The one hard boundary is **regime**: the on-ESC RPM governor works
only where the ESC can measure speed (6-step/BEMF, ≳300 RPM). Below that, speed stays open-loop (sine
S1/S2) or encoder-closed on the Pico — unchanged from the existing decisions.

## What already exists — do NOT reinvent

| Need | Already in firmware | Where (verified) |
|---|---|---|
| **Speed feedback** (no encoder) | `Comm_Period4x` — Timer2 ticks over last 4 commutations = measured eRPM; already derived + telemetered | `vendor/.../Bluejay.asm:256-257`, `Modules/DShot.asm:242-272`, `Flag_High_Rpm` `Bluejay.asm:213` |
| **Actuator** (duty ceiling) | `Pwm_Limit` (+ `Pwm_Limit_By_Rpm`, `Pwm_Limit_Beg`); applied duty = min of the limiters | `Bluejay.asm:267-269`, `Modules/Isrs.asm:350-354` |
| **"Regulate a limit toward a setpoint" template** | Temp protection **ramps `Pwm_Limit` up/down by 1** toward `Temp_Prot_Limit` each scheduler tick — an integrating limiter, exactly the shape an RPM PI takes | `Modules/Scheduler.asm:179-200` |
| **"Compute a limit from an eRPM comparison" template** | BlueGill **eRPM cap** (0x2C) folds a measured-eRPM-vs-cap test into `Pwm_Limit` via `min()` — the one-sided version of a governor | `src/.../Settings.asm:270`, `docs/drive-methods-and-position-control.md` #2 |
| **Temperature limiting** | `Pgm_Enable_Temp_Prot` (0=off, 1..7 = 80..140 °C); `Temp_Prot_Limit`; Scheduler enforces it | `vendor/.../Bluejay.asm:163`, `Settings.asm:128-162`, `Scheduler.asm:112-200` |
| **Temp reported** | EDT temperature frame `0x02` is emitted (and drives the PWM-limit throttling) | `docs/telemetry.md`, `Modules/Scheduler.asm` |
| **Low-RPM power shaping** | `Pgm_Rpm_Power_Slope` (soft power-vs-rpm limit) | `vendor/.../Bluejay.asm:156, 306` |
| **A slew limiter** (to generalize) | S1 `Pgm_Sine_Ramp` (0x31) slews the sine speed accumulator, clamped to not overshoot target | `src/Modules/SineMode.asm:248-258` |
| **Prior art in the lineage** | Dormant `_Pgm_Gov_Setup_Target` / `_Pgm_Gov_Range` — BLHeli's ancestral **heli RPM governor** (underscore = reserved/removed) | `vendor/.../Bluejay.asm:314-320` |

Takeaway: **temperature (feature 3) is ~done** — it is exposure/tuning, not a build. **Ramp (feature
2)** generalizes an existing mechanism. **The RPM governor (feature 1)** is the only substantial new
firmware, and it wires together pieces that already exist.

## Feature 1 — On-ESC closed-loop RPM governor (the core new work)

**Concept.** In a new opt-in **RPM mode**, the DShot throttle maps to an **eRPM setpoint** (exactly as
S1 sine mode already remaps throttle→speed, `docs/sine-drive-design.md`). A slow PI loop compares the
setpoint to the measured `Comm_Period4x` and trims the duty each scheduler tick, using the same
integrating-limiter shape as temp protection.

- **Feedback:** `Comm_Period4x` → eRPM. No encoder, no new sensing. Convert the setpoint to a target
  comm-period once (compare periods directly to avoid a divide on the 8051).
- **Actuator:** produce a governor duty and fold it into the existing `min(Pwm_Limit, Pwm_Limit_By_Rpm,
  …)` chain (`Isrs.asm:350`) — so the governor never overrides the temp/eRPM-cap/demag safeties; it
  only ever commands *less* than they allow. Anti-windup: clamp the integrator to the duty range.
- **Loop rate:** the scheduler tick (as temp protection uses). Motor + water load time-constants are
  ≫ ms, so a slow governor is right and cheap on the 8-bit core.
- **Regime (the hard boundary — state it loudly):** valid **only in 6-step run mode**, where
  `Comm_Period4x` tracks a real BEMF period — i.e. the committed **~300–6000 RPM** band
  (`docs/low-speed-tuning.md`). In **sine mode `Comm_Period4x` is stale by design** (`sine_run` never
  updates it — `docs/sine-drive-design.md`), so the governor **must be inhibited** whenever
  `Pgm_Sine_Mode != 0`. Below the 6-step floor, speed is open-loop V/f (sine) or encoder-closed on the
  Pico — no change to those.
- **Failsafe:** on desync/stall/demag (the `Status 0x0E` conditions) or loss of a valid period, drop
  the governor and revert to open-loop throttle = the commanded setpoint's feed-forward duty, so a
  bad measurement can never wind the duty up. Reuse the existing demag/stall detection.
- **Feed-forward seed:** initialise the governor duty from the throttle→duty map (or the host's
  known throttle→RPM linearisation from `autocal.py`) so it starts near-right and the PI only trims —
  faster settling, less hunt.

**Why on the ESC and not the Pico:** the feedback (`Comm_Period4x`) is *already inside* the ESC at
commutation rate; closing the speed loop there removes the 1-wire round-trip latency, makes each of
the 8 thrusters self-regulating (no per-ESC PI on the Pico), and lets `pico-esc-tool` just say "hold
N RPM." This is the "thin library" the user wants.

## Feature 2 — Ramp / slew limiter (generalize the sine ramp to 6-step)

A general **commanded-value slew limiter** on the throttle/RPM setpoint in run mode: bound the
per-tick change so accel/decel is smooth and cannot slam thrust. Model it on `Pgm_Sine_Ramp`
(`SineMode.asm:248` — advance toward target by ≤ ramp/tick, never overshoot), but applied to the
6-step Rcp/limit path. Separate up-rate and down-rate is desirable (fast stop, gentle start). This is
already anticipated as a future param — "firmware **slew limiter**" in `docs/drive-methods` #2 and the
"future accel-slew limiter" in `docs/position-control.md`. Keep raw (un-ramped) behaviour the default.

## Feature 3 — Temperature (expose + tune + confirm report)

Mostly present. Work is: (a) confirm the `0x02` temp frame streams under our EDT setup (it does —
`docs/telemetry.md`); (b) optionally expose the `Pgm_Enable_Temp_Prot` threshold and a **softer roll-off
curve** for continuous underwater load rather than the coarse 80–140 °C steps; (c) document the soak
protocol (already in `low-speed-tuning.md` §"S1 bench ramp", which runs with `temperature_protection:
1`). No governor without this active — the RPM governor and sine hold both lack a current trip, so the
temp limiter is the backstop (`docs/sine-drive-design.md` FET-safety).

## Proposed EEPROM overlay (append-only, opt-in, safety-clamped)

Follow the established BlueGill pattern exactly (`src/Modules/Settings.asm` decode: `0xFF` stale →
default/off, then a decode-clamp; `src/Bluejay.asm` DEFAULT_/Eep_/Pgm_ triples). Next free offset is
**0x32**. Illustrative (names/sizes to finalise at build time):

| Offset | Param | Meaning | Default |
|---|---|---|---|
| 0x32 | `Pgm_Rpm_Mode` | 0=off (raw throttle, stock), 1=closed-loop RPM governor | 0 |
| 0x33 | `Pgm_Rpm_Kp` | governor proportional gain (fixed-point) | timid |
| 0x34 | `Pgm_Rpm_Ki` | governor integral gain (fixed-point) | timid |
| 0x35 | `Pgm_Rpm_Ramp_Up` | run-mode setpoint slew, up (LSB/tick) | gentle |
| 0x36 | `Pgm_Rpm_Ramp_Dn` | run-mode setpoint slew, down (LSB/tick) | faster |

- Bump `EEPROM_LAYOUT_REVISION` 225 → **226** (new fork marker) and
  `EEPROM_B2_PARAMETERS_COUNT` 35 → 40. `pico-esc-tool`'s layout/MCU **compat guard** already keys on
  the layout tag, so a new rev is refused against an old-rev ESC until reflashed — intended.
- Every new param **decode-clamped** so a stale/hostile EEPROM can't command an unsafe gain or
  disable the safeties (mirror `SINE_*_CLAMP`).
- All default-off ⇒ a flashed BlueGill behaves exactly as today until the host opts in. Raw input
  (feature-preserved) is just `Pgm_Rpm_Mode=0`.

## Host / `pico-esc-tool` library implications

- **Thinner host.** In RPM mode the host sends a **DShot throttle that the ESC interprets as an RPM
  setpoint** — no Pico-side speed PI. `esctool` gets an `rpm <i> <target>` that writes the setpoint;
  telemetry still returns measured eRPM to confirm tracking.
- **Supersedes the Pico-side RPM loop for the 6-step regime.** The earlier plan
  `~/.claude/plans/dreamy-giggling-marble.md` put RPM filtering + a PI on the Pico. If the governor
  moves to the ESC, that plan's on-Pico PI becomes redundant **in the 6-step band**; the Pico keeps:
  (a) the **8-ESC PIO split** (still needed — orthogonal), (b) telemetry decode/stream, (c) the
  **outer position/velocity loop** (`posctl.py`, #4B), which now commands **RPM** to a self-regulating
  ESC — a cleaner cascade than commanding raw throttle. *That plan is not edited here (separate doc, per
  request); reconcile when this is scheduled.*
- **Encoder decision unchanged.** `docs/position-control.md` (encoder/position loop stays on the Pico,
  ESC stays sensorless) **still holds** — an RPM *speed* governor needs no encoder and does not touch
  position. This refines the inner loop from open-loop to closed-loop-speed; it is **not** a reversal.

## Who owns speed at which RPM (regime map)

| RPM band | Speed control | Where | This doc |
|---|---|---|---|
| 0 / hold / < ~185 | open-loop V/f + hold, or encoder position loop | ESC (sine S1/S2) + Pico (#4B) | governor **inhibited** (period stale) |
| ~185–300 | marginal 6-step | ESC | governor optional, guard desync |
| **~300–6000** | **closed-loop RPM governor** | **ESC (this doc, feature 1)** | primary target band |
| position moves / holding | outer position loop commands RPM/thrust | Pico (`posctl.py`) | consumes the governor |

## Prior art (surveyed 2026-07-15, sourced)

**Gating fact:** our silicon is **EFM8BB21 (8051)**. Only **BLHeli_S, Bluejay, JESC** run on it —
**BLHeli_32 and AM32 are ARM-only**. So a governor either gets **built into Bluejay's assembly**, or
it comes from **switching thruster ESCs to ARM hardware**. No third option on EFM8.

| Firmware | Silicon | Closed-loop RPM governor | Ramp/slew | Temp limit | Forkable? |
|---|---|---|---|---|---|
| **Bluejay** (our base) | EFM8 8051 asm | **No** (measures eRPM, doesn't regulate) | startup/rampup only | **Yes** + EDT report | **Yes (open asm)** |
| BLHeli_S | EFM8 8051 asm | No ([bitdump #288](https://github.com/bitdump/BLHeli/issues/288), never built) | startup only | Yes (no EDT) | yes, no telem |
| JESC | EFM8 8051 asm | No | startup only | yes | **No** (closed/licensed) |
| BLHeli_32 | **ARM** | **Yes** (Closed-Loop mode + P-gain) | yes | yes | **No** (closed, legacy/EOL) |
| **AM32** | **ARM** | **Yes, native C** (`use_speed_control_loop`, `speedPid` Kp/Ki/Kd, `drive_by_rpm`, `FIXED_SPEED_MODE`, temp limit, sine startup) | yes | yes | **Yes (open C)** |

- **BLHeli heritage governor.** The reserved `_Pgm_Gov_Setup_Target`/`_Pgm_Gov_Range`
  (`Bluejay.asm:314-320`) are the disabled remains of BLHeli's **main/tail RPM governor** (heli
  head-speed hold) — closed-loop RPM on this exact 8051 lineage is proven, just removed for FPV. We
  revive it for thrusters, not invent it.
- **AM32 is the reference implementation of exactly this feature** — on ARM. If the 8051 governor
  cost (below) proves too high, adopting ARM/AM32 ESCs gets on-ESC RPM PID + RPM-input + ramp + temp
  **off-the-shelf**. That is a **hardware pivot** (different ESCs, revalidate everything), recorded
  here as the honest alternative; the current decision is to stay on EFM8/Bluejay. Read AM32's
  `Src/main.c` speed-PID as a design reference for the loop shape.

## Effort & the real risk (EFM8 8051, sourced assessment)

- **Temp limit = ~free** (reuse `Pgm_Enable_Temp_Prot` + Scheduler ramp). **Ramp/slew = small** (days;
  generalize `Pgm_Sine_Ramp` + existing startup rampup). **RPM PI governor = the bulk** — weeks of
  hand-written fixed-point assembly.
- **The dominant risk is not the PID math — it is ISR timing.** The commutation / zero-cross path is
  cycle-critical; a governor tick must slot into the scheduler **without perturbing commutation
  timing**, or it corrupts sync. Budget it like the sine work did (cycles/update in the sim first).
- **Flash/RAM are tight** — Bluejay is already near budget on BB21; new code + the sine module compete
  for space. Confirm the image still fits (`build/` size) before committing scope.
- No FPU / byte-only MUL·DIV → hand-coded fixed-point with overflow care; no unit tests → slow,
  bench-heavy iteration. **Feasible** (Bluejay/BlueGill already prove non-trivial features land here),
  but low-iteration and hardware-gated for tuning.

## FET safety & bench discipline (non-negotiable, per project convention)

- No current sense on this board (`docs/sine-drive-design.md`): the governor sets duty by voltage only,
  so **temp protection must be enabled** and amplitude/duty clamps enforced whenever the governor or
  ramp is active. The governor folds into `min(…)` so it can only *reduce* duty vs the safeties.
- Every new param decode-clamped; opt-in default-off; keep the stock 6-step path bit-identical when
  `Pgm_Rpm_Mode=0`.
- Bring-up follows the `low-speed-tuning.md` soak protocol: low gains first, watch the `tele` temp
  frame, one lever at a time, baseline-vs-delta measured — never folklore.

## Validatable now, before any hardware (sim-first, like S1)

- Extend `tools/sim/` with a **governor model**: throttle→RPM-setpoint map, PI on a simulated
  `Comm_Period4x`, and step/settle/overshoot vs Kp/Ki — pick timid safe defaults on paper.
- Verify the setpoint→target-comm-period conversion is divide-free / fits the 8-bit budget (as the
  sine model verified its cycle budget).
- Confirm the `min()` folding and desync-failsafe logic against the vendor `Isrs.asm`/status paths.

## Suggested phasing (when scheduled — its own orchestrate loop)

1. **T1** — temperature: confirm frame + expose/soften threshold param + doc soak. (small)
2. **R1** — run-mode ramp/slew limiter (generalize `Pgm_Sine_Ramp`). (small–medium)
3. **G1** — 6-step RPM governor: sim model → params → PI folded into `min()` → desync failsafe →
   bench soak on the low-KV motor. (the real work; needs hardware to tune Kp/Ki)
4. **Host** — `esctool rpm` + reconcile the Pico plan (governor supersedes the Pico inner PI in-band).

## Open questions
- Governor loop rate: scheduler-tick vs a dedicated slower tick — measure hunting vs 8-bit budget.
- Setpoint units on the wire: reuse the S1 throttle→speed contract, or a distinct RPM-mode mapping?
- Threshold/hysteresis for governor↔open-loop failsafe and the sine↔6-step crossover interaction.
- Is a softer temp roll-off actually needed for continuous submerged load, or is 80–140 °C enough?
