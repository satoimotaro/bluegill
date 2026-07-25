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

## Subtasks (highest-risk-first)
1. Baseline build/MAP. ✅ (A_H_30_24 top=0x19D7, 38 B free)
2. Minimal down-seed experiment (sector seed + dynamic Inc seed; static Settings path kept) → HW sector scan on S2.
3. Down-side debounce + near-stall sanity gate; jam test.
4. Flash reconciliation: Settings.asm seed-store trim (WITH the decode_cross_guard Temp4 fix) + per-variant MAP gate.
5. Goal C verification sweeps.
6. Goal B bench discrimination (B0) → B1 only if proven.
7. sine_mode=0 regression + diff review (all deltas sine-gated).
8. Docs + commit.
