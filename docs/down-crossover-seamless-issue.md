# Issue: seamless BEMF→sine DOWN crossover (down-handoff phase seed)

Status: OPEN · Branch: `mas/bluegill-down-crossover` (off `docs/350kv-noload-crossover` @ 7d34fb6)
Local-only repo (no remote) → issue tracked here; merge target `docs/350kv-noload-crossover`.

## Problem
The S3 sine↔6-step crossover is built and the UP-handoff (forced-sine → BEMF) is bench-proven, but
the DOWN-handoff (BEMF-6-step → forced-sine on decel) stalls the motor with an active `cross_dn`
(bench repro: `cross_dn=239` stalls to 0 at ~300 rpm). Root cause: `src/Modules/SineMode.asm:144`
unconditionally forces `Sine_Sector=1` on every handoff, and the `Flag_Sine_Handoff` branch
(:156-171) seeds only speed (`Sine_Inc`), never *phase* → the forced field snaps to an arbitrary
electrical sector regardless of rotor position → stall. Shipped workaround `cross_dn=255` merely
*disables* the down-handoff (sine fallback) — not seamless.

## Goal
Move startup + regime handling fully INSIDE the ESC so the Pico/host sees a UNIFORM
throttle→mechanical-rpm response with no visible mode seams. Concretely:
- (A) Seamless DOWN crossover — seed the true electrical phase at the handoff instant.
- (B) Reliable soft-start (measure-first; scoped, likely no new mechanism).
- (C) FF continuity — already done (`cross_rescale_duty`); verify-only.

## Design
Reviewed design + panel synthesis:
`~/.claude/orchestrator/runs/20260726-032355-design-esc-side-startup-crossover-unific/`
(PLAN.md + iterations/1/synthesis.md). Key insight: at the down-handoff the 6-step electrical
state is deterministic (always program-state 1), so NO BEMF catch is needed — seed one sector
constant `SINE_DN_SEED_SECTOR=6` (bench-scannable) + a dynamic `Sine_Inc` from the live
`Comm_Period4x` + a debounce/sanity gate. Net ~+21 B vs ~25 free; trim ladder available.

## Acceptance
- Down-sweep at `sine_mode=2 cross_up=34 cross_dn=239`: mech rpm monotonic through the handoff,
  no stall-to-0, controllable to neutral stop; 10/10 both directions.
- Full triangular sweep: both seams ≤15% transient, <300 ms settle; steady cmd→rpm monotonic.
- Soft-start: peak ≤120% of commanded steady, no cogging.
- Existing safeties (stall-retry→3→beep, thermal PWM limit, eRPM cap) still fire with crossover active.
- `sine_mode=0` baseline byte-/behaviorally-identical; app CSEG ≤0x19FD in ALL variants.
- Residual ~175-600 rpm no-steady-state band is a plant property (out of scope), host tolerates.

## ⚠ Hardware results 2026-07-26 (Subtask 2 hypothesis FALSIFIED)
Bench: F2838 350KV + small prop, AS5600, 11.1V. Flashed the seed build, drove via host.
- **Motor DOES enter 6-step with DWELL** (not a fast ramp): thrust 600 → 1368 mech (tele live 1395),
  780 → 3224 mech. So the "350KV can't 6-step at no-load" note is too pessimistic WITH a small prop —
  the up-handoff catches fine when power is held (xover_debug's continuous ramp outruns the rotor → its
  "stall" was a ramp-rate artifact, not a real limit). Also: after every flash, `startup_power_max`
  resets to 5 which is too low to even start forced sine — must restore ~25 (see [[esc-flash-resets-eeprom]]).
- **Down-handoff (cross_dn=239) STILL STALLS with the seed.** Descending from 6-step, the motor drops to
  0 rpm right at the ~187 mech handoff. Tested `SINE_DN_SEED_SECTOR` = **6 and 3** (180° apart) and
  **sine_mode = 1 (S1) and 2 (S2)** — ALL FOUR combinations stall IDENTICALLY.
- **Un-seeded fallback (cross_dn=255) does NOT stall** — descends jittery ~30-160 mech, keeps rotating.
- **CONCLUSION: the sector seed is NOT the fix.** The sector value and S1/S2 path don't change the
  outcome, so the stall is not a phase-snap. The seeded down-handoff (which routes through `motor_start`
  → `switch_power_off` → seeded `sine_run`) stalls where the naive stock min-speed exit (cross_dn=255,
  Sine_Inc=0, no motor_start) keeps spinning. **Prime suspects: the `motor_start` power-off gap on the
  down transition, and/or the dynamic `Sine_Inc` seed running the field too fast at re-entry.** Redesign
  needed — the down-handoff likely must transition WITHOUT motor_start's power-off (stay energised),
  mirroring why the up-catch works. Sector seed may still be necessary but is insufficient alone.

## Isolation experiment 2026-07-26 (Sine_Inc RULED OUT → motor_start is the cause)
Rebuilt with `Sine_Inc=0` in the down-handoff branch (like the working cross_dn=255 fallback) while
KEEPING the sector seed. Result: **STILL STALLS** identically. So the dynamic Inc seed is NOT the
culprit. Combined with the sector/mode scan, the stall is **the `motor_start` power-off gap**:
run6_check_speed does `setb Flag_Sine_Handoff; ljmp motor_start`, and `motor_start`'s `switch_power_off`
cuts the FETs mid-spin at ~187 mech; the seeded `sine_run` re-energise cannot recover the coasting
rotor. The stock cross_dn=255 descent keeps rotating because it never routes through motor_start.

**REDESIGN DIRECTION:** the down-handoff must transition 6-step→sine WITHOUT `motor_start`'s
`switch_power_off` — re-energise the seeded sine sector directly from the live 6-step drive (stay
energised across the seam), mirroring how the up-catch avoids a power-off. This is a control-flow
change at the run6_check_speed→sine_run seam, not a constant. The sector seed (=6) and the Inc handling
are still needed once the power-off is removed, but are insufficient while it remains.

## Subtasks (highest-risk-first)
1. Baseline build/MAP. ✅ (A_H_30_24 top=0x19D7, 38 B free)
2. Minimal down-seed experiment (sector seed + dynamic Inc seed; static Settings path kept) → HW sector scan on S2.
3. Down-side debounce + near-stall sanity gate; jam test.
4. Flash reconciliation: Settings.asm seed-store trim (WITH the decode_cross_guard Temp4 fix) + per-variant MAP gate.
5. Goal C verification sweeps.
6. Goal B bench discrimination (B0) → B1 only if proven.
7. sine_mode=0 regression + diff review (all deltas sine-gated).
8. Docs + commit.
