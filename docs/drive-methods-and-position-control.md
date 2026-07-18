# Drive methods & position control — feasibility and roadmap (BlueGill / pico-esc-tool)

Status: design/feasibility (2026-07-14). Grounds four candidate directions (#1–#4) in the
actual hardware. Verdicts drive what we build now vs. what waits for the low-KV (300 KV)
motor and/or a hardware modification.

## The hardware that constrains everything

| | ESC (LittleBee, **EFM8BB21**) | Controller (**RP2040 / Pico W**) |
|---|---|---|
| Core | 8-bit 8051, ~25 MHz | 32-bit dual-core, 133 MHz |
| Math | integer only, tiny RAM (~1.25 KB) | fast 32-bit, **no hardware FPU** |
| Motor sense | **BEMF comparator only**; ADC reads **temperature only — NO current sensing** | ADC present but weak/buggy; no motor connection by default |
| Drive | 6-step **trapezoidal**, sensorless, complementary (damped) FETs | one signal wire out per ESC (DShot) |
| Interface in | **one signal wire** (DShot/PWM), commutation done internally | USB-CDC / Wi-Fi to host |

Two facts decide the hard cases: **true FOC needs per-phase current feedback the ESC does not
have**, and **FOC math (Clarke/Park/SVPWM at ~10–20 kHz) does not fit an 8-bit 8051**. An 8-bit
MCU *can* do open-loop **sinusoidal** PWM (no current, no encoder), which is the standard cure for
6-step torque ripple at low speed. Source-verified in `vendor/bluejay/src` (Scheduler.asm ADC =
temp only; Commutation.asm = 6-step Set_Pwm_Phase_A/B/C; no shunt/current path).

## #1 — Host auto-calibration tool — ✅ DONE (host-side)
Implemented: `pico-esc-tool/host/autocal.py`. Per-thruster characterization + auto-tuning
(direction, cold-start bisect, min sustainable RPM, throttle→RPM linearization, startup-power
bisect, timing/demag sweep) over the keep-alive DShot drive; `SimEscHost` `--dry-run` runs with no
hardware; raises `CalibrationError` rather than emitting an unverified value; always disarms.
Bench-run pending on real thrusters. No firmware dependency.

## #2 — Detailed ESC calibration settings — ✅ DONE (first BlueGill firmware params)
Implemented as whole-file overlays on Bluejay v0.21.0 (branch `mas/bluegill-b0`), all default-off:
- **Direct commutation angle** (EEPROM 0x2B): value 1..17 = 0..30° advance in 1.875° steps
  (0 = use the 1..5 preset). Generalizes `Pgm_Comm_Timing` — set the advance directly.
- **eRPM cap** (0x2C, units 1000 eRPM, ≤136k): hard in-ESC ceiling folded into the Pwm_Limit
  governor via `min()`. Out-of-water overspeed containment the host loop is too slow to guarantee.
- **Low-speed damping** (0x2D): braking-strength override below a low-RPM threshold.

Behavior is bench-pending. Later B-phase params (not yet built): run-mode **min duty** + `run6`
**comm-period floor** to break below the ~185 RPM DShot-minimum floor; firmware **slew limiter**;
explicit **arm/disarm** default.

## #3 — Higher-frequency / sinusoidal drive on the ESC (encoderless, currentless) — ⚠️ PARTIAL
- **True sensorless FOC on the EFM8: NO.** No current sense; FOC math too heavy for the 8-bit core.
- **Open-loop sinusoidal / scalar (V-f, "gimbal") drive: FEASIBLE on 8-bit**, and it is the right
  tool for ultra-low-speed smoothness and low-speed position holding (rotor follows the commanded
  sine angle synchronously, like a stepper). This **replaces** BLHeli-S's 6-step commutation core
  with a 3-phase sine-PWM generator driven from a precomputed sine table + a commanded electrical
  angle — a **major new firmware module, not a thin Bluejay patch**.
- **Architecture (hybrid, the known-good approach):** open-loop sine below ~a few-hundred RPM →
  hand off to Bluejay's BEMF 6-step above a crossover RPM (open-loop sine loses efficiency/sync at
  speed under load). Startup uses the sine ramp instead of blind 6-step; BEMF comparator can still
  detect sync for the handoff.
- **Cost / risk:** the largest firmware effort in the project; own phase; needs the low-KV motor to
  tune the crossover and current/heat under load (no current sense → open-loop torque is set by
  V and angle only, so thermal headroom must be watched). Prototype the sine generator + table now;
  validate on hardware later.
- **Effort tiers:** (a) sine table + 3-phase PWM writer + angle integrator (open-loop V-f) — the
  core; (b) startup-from-sine + BEMF-based crossover to 6-step — the hard part; (c) sync/desync
  handling at the boundary.

## #4 — Encoder position control (encoder on the Pico), ESC as half-bridge driver — ⚠️ MIXED
The blocker: a stock ESC takes one signal wire and commutates internally. Three routes:
- **A) True "ESC as inverter" (hardware mod).** Tap the EFM8's 6 gate-driver inputs, hold the EFM8
  in reset, drive the FETs directly from the RP2040 running SimpleFOC-style FOC with the encoder
  (+ add current shunts→Pico ADC for real FOC). **Feasible** (SimpleFOC runs on RP2040; RP2040 has
  PIO + speed) but a **hardware-hacking project per ESC**, and it bypasses BLHeli-S/EFM8 entirely —
  at that point a purpose-built 3-phase gate-driver board is cleaner than modifying LittleBees.
  RP2040 has no FPU (fixed-point FOC) and a weak ADC (external ADC for current sense preferred).
- **B) Cascade control — NO hardware mod (RECOMMENDED near-term).** Pico reads the encoder → outer
  **position/velocity PID** → throttle via DShot → the ESC does its normal sensorless commutation
  as the inner speed actuator. **Real closed-loop position control, works today, testable on the
  930 KV motor.** Limit: the inner loop is sensorless 6-step — weak below ~185 RPM and **no
  zero-speed holding torque** (rotor free at standstill). Good for position *moves* at moderate
  speed, not stiff servo *holding*.
- **C) Rotor angle over the 1-wire at FOC rate** — bandwidth won't allow it; rejected.

**#3 + #4 belong together:** true low-speed position control (incl. slow crawl and holding) needs
the smooth low-speed torque that only sinusoidal drive (#3) gives. The realistic path:
1. **#4B cascade position control now** (Pico + host, no HW mod, no FOC) — gets position moves working and is a testbed for the encoder + control loop. Validates on the 930 KV motor.
2. **#3 open-loop sine firmware** — unlocks sub-185 RPM smoothness and low-speed holding; pair it as the inner drive under #4B's position loop for slow/hold regimes.
3. **#4A FOC-as-inverter** — only if #3+#4B prove insufficient for the servo goal; it is a separate hardware+RP2040-FOC track (documented, parked). Zero-speed *holding torque* ultimately wants either sine open-loop hold (#3) or full FOC (#4A).

## #5 — (parked) not pursued.

## Recommended sequencing
| Phase | Work | Needs low-KV motor / HW? |
|---|---|---|
| now | #1 tool (done), #2 params (done) — bench-validate when hardware returns | bench only |
| next | **#4B cascade encoder position control** (Pico + host) — design + prototype | encoder; 930 KV ok |
| then | **#3 open-loop sine drive** firmware module (sine table → 3-phase PWM → angle; then BEMF crossover) | low-KV motor to tune |
| combine | #3 as the low-speed inner drive under #4B's position loop (slow-crawl + hold) | low-KV motor |
| future | #4A FOC-as-inverter (hardware mod + RP2040 FOC) — only if needed | HW mod + shunts |

References: SimpleFOC on RP2040; BLHeli_S is trapezoidal/damped-light; 8-bit MCUs do sinusoidal
PWM (not FOC) without current sensors; 6-step has high torque ripple at low speed.
