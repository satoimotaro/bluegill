# Low-speed tuning — BlueGill

The core R&D problem: make a sensorless BLHeli-S ESC run a **low-KV, high-inductance
underwater thruster** smoothly at low RPM under continuous hydrodynamic load, where BEMF
is weak and zero-cross detection is marginal. This doc lists the levers, the eRPM math,
the revised speed envelope, and the harness protocol used to measure each change.

> All lever/param names and default values are taken from the pinned vendor at
> `vendor/bluejay` (tag v0.21.0). Iteration 1 changes **no** assembly — the first
> experiments are pure `esctool.py set` on existing params. Code overlays come later.

## eRPM math

    electrical_RPM = mechanical_RPM * pole_pairs
    pole_pairs     = pole_count / 2
    commutation period ≈ 60 / (eRPM * 6)  seconds   (6 commutations / electrical rev)

BlueJay tracks speed as `Comm_Period4x` (Timer2 ticks across the last 4 commutations,
24 MHz timebase). For a 14-pole (7 pole-pair) motor:

| mech RPM | eRPM | comm period | note |
|---|---|---|---|
| 100 | 700 | ~14 ms | at/below sensorless ZC limit; near the run-mode floor |
| 300 | 2 100 | ~4.8 ms | bottom of the committed working range |
| 2 000 | 14 000 | ~0.71 ms | efficiency sweet spot |
| 4 000 | 28 000 | ~0.36 ms | top of the sweet spot |
| 6 000 | 42 000 | ~0.24 ms | top of the working range |

Record the real motor's `MOTOR_POLES`/`MOTOR_KV` in
`targets/littlebee-spring-30a/target.env` and recompute this table for it.

## Revised speed envelope (per checkpoint 2026-07-13)

- **Working range: ~300–6000 RPM** — the range we design for control authority.
- **Efficiency sweet spot: 2000–4000 RPM** — the committed band; tuning is validated here
  first.
- **Stretch / R&D: 60–240 RPM sensorless slow rotation** — a late phase; may be physically
  unreachable sensorless on this hardware. The honest fallback for very slow, precise
  motion is the **controller-side encoder path** (see `docs/position-control.md`), not the
  ESC. Do not block the working-range tuning on the 60–240 RPM stretch.

## The levers

Reference locations are in `vendor/bluejay/src/…` at the pinned tag. "EEPROM param" =
tunable at runtime today via `esctool.py set`; "code" = needs a `src/` overlay (later
iteration).

| Lever | Where | Default | Low-speed direction | Kind |
|---|---|---|---|---|
| Commutation timing advance | `Pgm_Comm_Timing` (`Bluejay.asm` `DEFAULT_PGM_COMM_TIMING=4`), applied in `Modules/Timing.asm` `calc_new_wait_times` | 4 (MediumHigh) | try **1–2** (Low/MedLow); high-inductance low-KV motors want minimal advance. Demag-metric auto-advance stays as the safety net. | EEPROM |
| Run-mode comm-period floor | `Bluejay.asm` `run6_check_speed`: exits run mode when `Comm_Period4x_H > 0F0h` (annotated "~1330 erpm"); bidir-braking termination at `40h` ("~5000 erpm") | `0F0h` | **raise** the floor so run mode tolerates slower commutation before dropping to restart | code |
| Period averaging | `Modules/Timing.asm` `calc_next_comm_period` (/4 /8 /16 tiers by RPM) | RPM-scaled | push more **/16 averaging** at long periods to stabilise the estimate against water-load ripple | code |
| ZC blanking window | `Modules/Timing.asm` `wait_before_zc_scan` (startup timeout = `Comm_Period4x/2`) | — | **lengthen** blanking at long comm periods (PWM switching noise dominates weak BEMF) | code |
| ZC sampling | `Modules/Timing.asm` `wait_for_comp_out_low/high` (startup ≈ one PWM period of samples) | — | **more samples** at low RPM — extend the startup "many samples" strategy into low-RPM run mode | code |
| Demag compensation | `Pgm_Demag_Comp` (`DEFAULT_PGM_DEMAG_COMP=2`), `wait_for_comm` sliding 8-sample metric | 2 (Low) | keep the demag cut as protection; retune the threshold only with data | EEPROM/code |
| Startup / min duty | `Pgm_Startup_Power_Min` (`=21`, min duty) and `Pgm_Startup_Power_Max` (`=5`) | 21 / 5 | tune first as pure params; add appended `Pgm_Min_Run_Duty` only if a distinct **run-mode** floor proves necessary | EEPROM (+code later) |
| Low-RPM power slope | `Pgm_Rpm_Power_Slope` (`=9`) | 9 | limits power at low rpm; adjust with sweeps | EEPROM |
| Braking / damping | `Pgm_Brake_On_Stop` (`=0`) + `Pgm_Braking_Strength` (`=255`, complementary PWM) | 0 / 255 | evaluate for low-speed smoothness under hydrodynamic load | EEPROM |
| PWM frequency | compile-time 24/48/96 kHz (which hex you flash) | 24 (BlueGill default) | 24 kHz gives most duty resolution at tiny duties; A/B against 48 kHz | build-time |

## Harness sweep protocol

Per lever, one change at a time, measured on the pico-esc-tool bench (flash → set params →
arm bidir → throttle staircase → telemetry). Planned helper: `host/rpm_sweep.py` (throttle
staircase → CSV) — controller-side, plan step 6, not built in iteration 1.

> **Flashing wipes your tuning.** `esctool.py flash` writes the app AND the `0x1A00–0x1BFF`
> config page, so every flash **resets all EEPROM params to the firmware defaults** ("firmware
> default config applied"). Only the bootloader (`≥0x1C00`) is preserved. So the order is
> always **flash first, then re-apply your `esctool.py set` params** — never assume params
> survive a reflash. For param-only experiments (no firmware change), skip the flash and just
> `set`. See `targets/littlebee-spring-30a/README.md` for the full flash behaviour.

1. Flash the target hex (`--force` only if the guard reports `layout MISMATCH` on a stock
   ESC's first flash): `esctool.py flash <i> dist/BlueGill_*.hex --yes`.
2. **Re-apply the params under test** for this run (`esctool.py set <i> Pgm_...=...`),
   because step 1 reset them to defaults.
3. `esctool.py arm <i> bidir` (bidir DShot so eRPM/EDT telemetry streams).
4. Throttle staircase across the working range (e.g. steps that target ~300, 500, 1000,
   2000, 3000, 4000, 6000 RPM). Hold each step long enough to settle (≥2 s).
5. Log eRPM at 50–100 Hz per step (`tele`). Also log EDT **status** (demag/desync/stall)
   and **temperature** frames.
6. Compute per-step: mean eRPM, eRPM std-dev, min sustainable RPM, desync/stall counts.

### Metrics (what "better" means)

- **Min sustainable RPM** goes down (can hold slower without stall/desync).
- **eRPM std-dev in 2000–4000 RPM** goes down (smoother under load).
- **Zero desync/stall** across the working band.
- Compare stock BlueJay vs BlueGill on the same motor/load.

### Results template

| date | fw / param change | PWM | target RPM | mean eRPM | eRPM σ | min sustainable RPM | desync | stall | temp °C | notes |
|---|---|---|---|---|---|---|---|---|---|---|
| | baseline v0.21.0 | 24 | | | | | | | | reference |
| | | | | | | | | | | |

Fill one row per (firmware/param, PWM, target-RPM) point. Keep the baseline row so every
tuning claim is a measured delta, not folklore.

## S1 forced-commutation stepper mode — bench ramp (HIGHEST FET RISK)

S1 (`Pgm_Sine_Mode`, params 0x2E–0x31; see `docs/sine-drive-design.md` "S1 as-built") drives DC
stall current through one winding at hold with **no current sense and zero airflow**. Bring it up
**cautiously and monotonically**, temperature-protection **enabled** the whole time
(`host/profiles/posctl_930kv_sine.yaml` sets `temperature_protection: 1`):

1. **Hold, minimum amp.** Flash sine on with the timid defaults (hold 8 ≈ 3 %, amp_max 20 ≈ 8 %,
   ramp 16). Command thrust 0 (bidir), confirm the rotor **holds with palpable detent torque** and
   soak ~60 s while watching the `tele` temperature frame. Establish an acceptable soak-temp ceiling
   before raising `sine_hold_amp` at all — raise it only one small step at a time.
2. **Slow rotation.** Command a small thrust and confirm **smooth rotation well under 185 RPM**
   (measure the enc slope; ~50 RPM should be easy). V/f amp rises with speed but stays ≤ amp_max and
   ≤ the temp-governed `Pwm_Limit`.
3. **Reversal.** Command a direction change through zero and confirm the jump is **≤ 1 detent**
   (~8.6°) — S1 decelerates to a stop, flips, and re-seeds before stepping the other way.
4. **Servo.** Run `posctl.py move --deg 90` with the sine profile and confirm **no multi-rev fling**
   and a stiff hold. Only then explore higher `sine_amp_max`.

If tele temperature climbs toward the ceiling at any step, cut thrust and disarm — hold current has
no hardware trip beyond the coarse temp governor, which is exactly why the defaults are timid and
`sine_amp_max` is decode-clamped to ≤ 60.
