# Bluejay Sensorless 6-Step Commutation — Direction-Asymmetry Analysis

Scope: pure code analysis of the **vendor Bluejay** 6-step machinery
(`ESC-firmware/vendor/bluejay/src/`), cross-referenced against the **BlueGill
overlay** (`ESC-firmware/src/`). Goal: pin down *how* sensorless commutation
works and *where* a forward-vs-reverse asymmetry could originate, for the
BlueGill sine→6-step crossover that locks BEMF in forward but slips in reverse.

All `file:line` citations are to the vendor tree unless prefixed `overlay:`.
The crossover hands off to the **stock** run loop, so the root analysis is the
vendor code; the overlay is discussed in §6/§7 because it already contains a
targeted reverse fix.

---

## 1. The 6-step commutation sequence

### 1.1 The six states and the FET table

The canonical table is documented in `Modules/Commutation.asm:44-58`:

```
; Step   AA' BB' CC'
; 1 C->A 01  00  10
; 2 B->A 01  10  00
; 3 B->C 00  10  01
; 4 A->C 10  00  01
; 5 A->B 10  01  00
; 6 C->B 00  01  10
```

Each step drives two phases (one high-side "Com" FET, one low-side PWM-switched
"Pwm" FET) and leaves the third phase **floating**. The floating phase is the
one whose BEMF is measured by the comparator against the virtual star point
`V_Mux`.

The run-loop header comments (`Bluejay.asm:825-826, 839-840, 853-854, 866-867,
879-880, 892-893`) give the floating/measured phase per run state:

| Run | Driven (fwd) | Floating / comparator phase | Waited edge |
|-----|--------------|-----------------------------|-------------|
| run1 | B(p-on)+C(n-pwm) | A | comp **high** (A rising) |
| run2 | A(p-on)+C(n-pwm) | B | comp **low**  (B falling) |
| run3 | A(p-on)+B(n-pwm) | C | comp **high** (C rising) |
| run4 | C(p-on)+B(n-pwm) | A | comp **low**  (A falling) |
| run5 | C(p-on)+A(n-pwm) | B | comp **high** (B rising) |
| run6 | B(p-on)+A(n-pwm) | C | comp **low**  (C falling) |

### 1.2 The comm routines and their `_rev` variants

Each `commN_commN+1` routine (`Modules/Commutation.asm:62-185`) does three
things atomically (`clr IE_EA` … `setb IE_EA`): turns off the departing FET,
sets the new PWM phase (reapplying power after any demag cut), turns on the new
Com FET, and finally sets the **comparator mux to the phase that will float in
the next run state**. Example (forward, `Commutation.asm:62-71`):

```
comm1_comm2:                            ; C->A
    jb   Flag_Motor_Dir_Rev, comm1_comm2_rev
    clr  IE_EA
    B_Com_Fet_Off
    A_Com_Fet_On
    Set_Pwm_Phase_C                     ; Reapply power after a demag cut
    setb IE_EA
    Set_Comparator_Phase_B
    ret
```

The `_rev` variant is a strict **A↔C phase relabel** (`Commutation.asm:73-80`):

```
comm1_comm2_rev:                        ; A->C
    clr  IE_EA
    B_Com_Fet_Off
    C_Com_Fet_On                        ; was A_Com_Fet_On
    Set_Pwm_Phase_A                     ; was Set_Pwm_Phase_C
    setb IE_EA
    Set_Comparator_Phase_B              ; B is unchanged
    ret
```

Applying the A↔C swap to every routine yields the comparator-phase mapping
(each `commN` sets the phase used by `run(N+1)`):

| Routine | Comparator phase FWD | Comparator phase REV |
|---------|:--------------------:|:--------------------:|
| comm6_comm1 (→run1) | A | **C** |
| comm1_comm2 (→run2) | B | B |
| comm2_comm3 (→run3) | C | **A** |
| comm3_comm4 (→run4) | A | **C** |
| comm4_comm5 (→run5) | B | B |
| comm5_comm6 (→run6) | C | **A** |

So in reverse, every measured phase is the A↔C mirror of forward; **B is
invariant**. The direction branch is the *only* difference — the routine
structure (FET-off order, EN/PWM "Turn off pwm FET" sequencing for the odd
routines) is identical.

### 1.3 Comparator phase and edge polarity selection

`Set_Comparator_Phase_X` (`Layouts/Base.inc:131-156`) just points the comparator
mux at the phase-X terminal vs `V_Mux`; it carries **no polarity**. Polarity
(which edge = zero cross) is chosen entirely by which of two routines the run
loop calls:

- `wait_for_comp_out_high` — `mov B,#40h` (want comparator = high),
  `Timing.asm:593`
- `wait_for_comp_out_low`  — `mov B,#00h` (want comparator = low),
  `Timing.asm:587`

Critically, the run loop calls a **fixed** high/low per run state
(`Bluejay.asm:829,842,856,869,882,895`) and **does not** flip that choice on
`Flag_Motor_Dir_Rev`. The only run-time inversion of B is
`Flag_Dir_Change_Brake` (`Timing.asm:589-591, 595-596`), used during
bidirectional braking, not during normal reverse running.

> The reversal is therefore realized purely as: (a) A↔C swap of the *driven*
> phases and *measured* phase inside the comm routines, with (b) the edge
> polarity left fixed. §6 shows why that is nevertheless a correct mirror.

---

## 2. Zero-cross detection and commutation timing

### 2.1 `Comm_Period4x` measurement — `calc_next_comm_period`

Called immediately after each commutation (`Bluejay.asm:834` etc.).
`calc_next_comm_period` (`Timing.asm:53-252`) latches Timer2
(`Timing.asm:55-65`), on 48 MHz parts divides the raw count by 2
(`Timing.asm:68-73`), subtracts the previous timestamp to get *this*
commutation's duration, and folds it into `Comm_Period4x` (a running sum of ~4
commutation periods) with a speed-dependent averaging fraction
(`calc_next_comm_div_16_4 / _div_8_2 / _div_4_1`, `Timing.asm:161-217`).
`Comm_Period4x` is a **magnitude** (elapsed time); it contains no sign or
direction information.

`calc_next_comm_15deg` (`Timing.asm:226-252`) derives the 15° timer base
(`Comm_Period4x / 16`) minus a fixed reduction, stored in Temp4/Temp3.

### 2.2 Advance vs commutation split — `calc_new_wait_times`

The ZC→commutation window is nominally 30° (from the detected zero cross to the
next commutation). It is split into two Timer3 reload values:

- **`Wt_Adv_Start`** — the *advance* portion (fires before the ideal point when
  advanced).
- **`Wt_Comm_Start`** — the *commutation* portion.
- **`Wt_Zc_Scan_Start`** (7.5°) — blanking before scanning for the next ZC.
- **`Wt_Zc_Tout_Start`** (15°) — ZC scan timeout.

`calc_new_wait_times` (`Timing.asm:340-472`) reads the timing preset
`Pgm_Comm_Timing` into Temp8 (`Timing.asm:341-343`), builds the 15° and 7.5°
magnitudes, then:

- timing **normal** (preset 3) → `store_times_decrease` (`Timing.asm:457-461`):
  `Wt_Comm_Start = 15°`, `Wt_Adv_Start = ~0°`.
- preset 2/4 (even) → 22.5°/7.5° split (`Timing.asm:416-429`).
- preset 1/5 (odd, `adjust_timing_two_steps`, `Timing.asm:431-443`) → 30°/0°
  split, i.e. the full window becomes advance.
- higher-than-normal presets take `store_times_increase`
  (`Timing.asm:450-455`), swapping which of Temp1/2 vs Temp3/4 goes to
  advance vs comm.

### 2.3 How advance is realized in time

`setup_comm_wait` (`Timing.asm:745-762`) loads Timer3 with `Wt_Comm_Start`
and pre-loads the reload register with `Wt_Adv_Start`, so after the commutation
wait elapses Timer3 auto-reloads the advance portion. `wait_for_comm`
(`Timing.asm:803-866`) waits the `Wt_Comm_Start` delay; `wait_advance_timing`
(`Timing.asm:325-335`) waits the advance delay, then reloads
`Wt_Zc_Tout_Start`. The net effect: a larger `Wt_Adv_Start` / smaller
`Wt_Comm_Start` moves the commutation instant **earlier relative to the detected
zero cross** = more advance.

This is a **scalar time offset**. It is computed from `Pgm_Comm_Timing`
(a constant preset) and `Comm_Period4x` (a magnitude). **There is no
`Flag_Motor_Dir_Rev` anywhere in `Timing.asm`** (confirmed: the only direction
bit referenced is `Flag_Dir_Change_Brake`, and only for the braking-polarity
flip and the run-exit guard, `Timing.asm:589,595,776`). Advance is thus applied
identically in both directions, and because the comm sequence is mirrored (§6),
"earlier in time" = "advanced in the direction of rotation" for both.

### 2.4 `wait_for_comp_out_high/low`

`comp_start`…`comp_exit` (`Timing.asm:602-736`) polls the comparator, requiring
a direction-independent number of consecutive matching reads (`Temp3`), scaled
by RPM and phase (`Timing.asm:604-634`), with a Timer3 zero-cross timeout
(`comp_check_timeout`, `Timing.asm:636-649`). The desired level `B` was set by
the high/low entry point. Nothing here reads the motor direction.

---

## 3. Startup / BEMF acquisition

- `motor_start` sets `Flag_Startup_Phase` and `Flag_Initial_Run_Phase`
  (`Bluejay.asm:805-806`), zeroes `Startup_Cnt` (`:807`), primes two blind
  commutations via `comm5_comm6`/`comm6_comm1` and repeated
  `initialize_timing`/`calc_next_comm_period` (`Bluejay.asm:811-817`) to set a
  virtual commutation point at 7.5 ms (~1330 erpm, `initialize_timing`,
  `Timing.asm:37-41`).
- During startup, `calc_next_comm_period` branches to `calc_next_comm_startup`
  (`Timing.asm:75, 94-139`), which uses a two-commutation difference to reduce
  offset sensitivity, and `calc_new_wait_times` forces `Temp8 = 3` (normal
  timing, `Timing.asm:362-365`) and sets **very short** comm/scan/timeout delays
  to widen the zero-cross capture window (`Timing.asm:463-471`).
- `comp_start_check_startup_phase` uses many comparator samples (~one PWM
  period, `Timing.asm:611-615`), and `Startup_Cnt == 0` forces a timeout on the
  first commutation (`Timing.asm:661-662`).
- Lock progression: `evaluate_comparator_integrity` →
  `eval_comp_startup: inc Startup_Cnt` (`Timing.asm:790-794`). In the run6
  tail, `Startup_Cnt >= 24` (`Bluejay.asm:914-916`) clears
  `Flag_Startup_Phase`. Then `Initial_Run_Rot_Cntd` (init 12,
  `Bluejay.asm:808`) counts down 12 rotations (`Bluejay.asm:930-935`) before
  `Flag_Initial_Run_Phase` is cleared (`Bluejay.asm:942-943`) and
  `Flag_Motor_Started` set. During initial-run, demag detection is disabled
  (`Timing.asm:608-609`) and averaging is slowed (`Timing.asm:158`).

None of the startup logic branches on direction; the same `Startup_Cnt`/rotation
thresholds and capture-widening apply to forward and reverse.

---

## 4. Demag detection and compensation

### 4.1 The metric — `wait_for_comm`

`Flag_Demag_Detected` is set as the default at the start of every comparator
scan (`comp_init`, `Timing.asm:599`) and only *cleared* once a valid,
non-demag comparator reading is confirmed (`Timing.asm:664, 668`). It is also
force-cleared during initial-run (`Timing.asm:609`).

`wait_for_comm` (`Timing.asm:803-850`) maintains `Demag_Detected_Metric` as a
sliding average: `metric = (metric*7 + [8 or 9])/8`, incremented when
`Flag_Demag_Detected` is set (`Timing.asm:804-822`), floored at 120
(`:824-826`). If it exceeds `Demag_Pwr_Off_Thresh` (`:838-841`) it raises
`Flag_Desync_Notify` and cuts all PWM FETs (`All_Pwm_Fets_Off`,
`Set_All_Pwm_Phases_Off`, `:848-849`) — a hard power cut to retain sync during
demag.

### 4.2 `demag_compensation` (Off/Low/High)

`demag_compensation` maps to `Demag_Pwr_Off_Thresh` (the cut threshold) at
settings-load; Low/High lower the threshold so power is cut sooner. It changes
*when* the metric trips the cut, not the direction logic.

### 4.3 **`adjust_comm_timing` — the demag→advance feedback (KEY)**

This is the one place a *symmetric-in-code* mechanism produces a
*direction-dependent effect*. In `calc_new_wait_times`, once past startup
(`Timing.asm:362`):

```
adjust_comm_timing:                              ; Timing.asm:367-387
    clr  C
    mov  A, Demag_Detected_Metric       ; Check demag metric
    subb A, #130
    jc   load_comm_timing_done          ; metric < 130 -> leave preset alone
    inc  Temp8                          ; metric >= 130 -> +1 timing step (more advance)
    subb A, #30
    jc   adjust_comm_timing_limit_to_max
    inc  Temp8                          ; metric >= 160 -> +1 more step
adjust_comm_timing_limit_to_max:
    clr  C
    mov  A, Temp8
    subb A, #6
    jc   load_comm_timing_done
    mov  Temp8, #5                      ; clamp to preset 5 (max advance)
```

So the effective commutation-timing preset (Temp8) is **auto-advanced by up to
two steps** whenever the demag metric climbs to 130/160. Higher advance fires
commutation earlier — which, if the motor is already marginal, produces *more*
demag, raising the metric further: a **positive-feedback ratchet toward
over-advance**. This loop is entirely direction-agnostic in code
(`Demag_Detected_Metric` and `Pgm_Comm_Timing` carry no sign), but it engages in
whichever direction becomes marginal first.

---

## 5. Direction handling — every read/write of the direction bits

Bit definitions (`Bluejay.asm:199-222`):
`Flag_Startup_Phase`/`Flag_Initial_Run_Phase` (Flags0.0/.1),
`Flag_Motor_Dir_Rev` (Flags0.2), `Flag_Dir_Change_Brake` (Flags1.5),
`Flag_Pgm_Bidir` (Flags2.2), `Flag_Pgm_Dir_Rev`, `Flag_Rcp_Dir_Rev`
(Flags2.6).

- **Input decode** (`DShot.asm:105-136`): DShot cmd 7/8 set/clear
  `Flag_Pgm_Dir_Rev`; cmd 9/10 clear/set `Flag_Pgm_Bidir`.
- **Bidir throttle decode** (`Isrs.asm:231-256`): throttle sign sets carry;
  `Flag_Pgm_Dir_Rev` XORs it (`cpl C`, `:247-248`); result → `Flag_Rcp_Dir_Rev`
  (`:251`).
- **Motor start** (`Bluejay.asm:792-798`): `Flag_Motor_Dir_Rev := Flag_Pgm_Dir_Rev`;
  in bidir, overridden by `Flag_Rcp_Dir_Rev`.
- **Comm routines** (`Commutation.asm`): `jb Flag_Motor_Dir_Rev, ..._rev` at the
  top of each of the six routines — the only per-commutation use.
- **Comparator polarity** (`Timing.asm:587-596`): inverted **only** when
  `Flag_Dir_Change_Brake` set.
- **Run6 bidir handling** (`Bluejay.asm:990-1024`): on a commanded reversal
  (`Flag_Motor_Dir_Rev` ≠ `Flag_Rcp_Dir_Rev`), sets `Flag_Dir_Change_Brake`
  and jumps to `run4` to brake (`:1002-1006`); when braked below ~5000 erpm
  (`Comm_Period4x_H < 40h`, `:1011-1014`) it clears the brake flag, copies
  `Flag_Rcp_Dir_Rev → Flag_Motor_Dir_Rev` (`:1017-1019`), re-enters
  initial-run for 18 rotations (`:1020-1021`).
- **Run exit guards** (`Timing.asm:776`): no run-mode exit while braking.

Forward and reverse traverse the **identical** `run1→run6` loop, identical
high/low calls, identical timing math. The *only* difference reaching the motor
is the `_rev` A↔C relabel inside the comm routines (plus the transient
brake-polarity flip during a bidir reversal).

---

## 6. THE KEY QUESTION — is the 6-step BEMF loop direction-symmetric?

### 6.1 Phase + edge logic is a proven mirror (SYMMETRIC)

Build the zero-cross sequence each direction expects, from §1.2 (comparator
phase) × the fixed run-loop edge (§1.1):

- **Forward:** run1 A↑, run2 B↓, run3 C↑, run4 A↓, run5 B↑, run6 C↓
  → sequence **A↑ B↓ C↑ A↓ B↑ C↓** (the natural forward BEMF order).
- **Reverse:** run1 C↑, run2 B↓, run3 A↑, run4 C↓, run5 B↑, run6 A↓
  → sequence **C↑ B↓ A↑ C↓ B↑ A↓**.

Reversing mechanical rotation replays each phase's BEMF backward in time, which
(a) reverses the *order* of the crossings and (b) inverts each *slope*
(rising↔falling). Apply both operations to the forward sequence:

```
forward:                 A↑ B↓ C↑ A↓ B↑ C↓
reverse order:           C↓ B↑ A↓ C↑ B↓ A↑
invert each slope:       C↑ B↓ A↑ C↓ B↑ A↓   ==  firmware reverse sequence ✓
```

The firmware's reverse expectation **exactly matches** the true reversed-rotor
BEMF. The A↔C drive swap simultaneously reverses torque (current B→C becomes
B→A, etc.) and remaps the measured phase so the *fixed* high/low edges stay
correct. **Conclusion: the comm-routine / comparator-phase / edge-polarity /
drive-swap logic is an exact mirror — direction-symmetric.**

### 6.2 Timing advance is symmetric

`calc_new_wait_times` / `wait_advance_timing` / `setup_comm_wait` derive the
advance from `Pgm_Comm_Timing` (constant) and `Comm_Period4x` (magnitude), with
no direction input (§2.3). The same scalar offset shifts commutation the same
number of degrees earlier in both directions. Because the sequence is mirrored,
"earlier in time" is "advanced in the rotation sense" for both — **symmetric**.

### 6.3 Demag detection / blanking is symmetric in code

`Flag_Demag_Detected`, `Demag_Detected_Metric`, `Demag_Pwr_Off_Thresh`,
`Wt_Zc_Scan_Start` blanking — none branch on `Flag_Motor_Dir_Rev` (§4).
Symmetric as written.

### 6.4 Startup / run6_bidir are symmetric

Startup thresholds (§3) and the bidir brake→reverse handoff (§5) use the same
constants and code path for both directions.

### 6.5 Verdict

**The stock Bluejay 6-step BEMF loop is direction-symmetric.** No instruction
in the phase/edge/timing/demag/startup logic applies an effect in a fixed
(non-mirrored) sense with respect to rotation. There is **no** hidden
"forward-only" advance, edge, or comparator bias.

Therefore a *pure-firmware* forward-vs-reverse phase bug is ruled out. The
reverse slip cannot come from the comm routines, the comparator-phase mapping,
the fixed high/low edges, or the advance arithmetic — those are exact mirrors.

**Where the asymmetry actually lives:** the only mechanism that converts a
*physical* forward/reverse difference into a *runaway* is the demag-metric
auto-advance in `adjust_comm_timing` (§4.3, `Timing.asm:367-387`). It is
symmetric code but a **nonlinear positive-feedback loop**: whichever direction
is physically marginal first (reverse, under an asymmetric thruster/prop load —
a marine propeller has very different hydrodynamic load and thus BEMF
loading forward vs reverse) accrues demag, which ratchets Temp8 up by 1–2 steps
toward preset-5 max advance, which fires commutation earlier, which deepens
demag → the 6-step loop over-commutates and slips continuously in reverse while
forward stays inside the stable basin. The zero-cross *detection* is symmetric;
the *timing feedback that rides on top of it* diverges in the marginal
direction. Nothing in the firmware caps or per-direction-tunes that advance.

This is corroborated in-tree: the BlueGill overlay `Modules/Timing.asm` adds a
fix that reaches exactly this conclusion in its own comments
(`overlay:Modules/Timing.asm:535-547`):

> "The 6-step core is direction-SYMMETRIC (reverse = an A<->C phase relabel of
> forward), so the reverse comm-timing/slip inversion was NOT a
> fixed-sense-advance bug -- it was the demag-metric auto-advance
> (adjust_comm_timing) ratcheting into over-advance in whichever direction is
> marginal first (reverse, from the physical thruster-load asymmetry)."

The overlay's `apply_comm_timing_angle` (`overlay:Modules/Timing.asm:913-...`)
installs an **absolute** commutation advance (`(Comm_Timing_Angle_Adj-1) ×
1.875°`) and, when active, **overrides / severs the demag auto-advance** — but
scopes that override to **physical reverse only**, via the predicate
`C = Flag_Motor_Dir_Rev XOR Flag_Sine_Mode` (`overlay:Modules/Timing.asm:548-553`),
so physical forward keeps the proven stock preset+auto-advance path. This gives
reverse its own bench-tunable, non-ratcheting advance.

**Bottom line for BlueGill:** the reverse failure is *not* a phase-logic bug in
this firmware. It is a physical/load asymmetry (thruster loading forward vs
reverse) amplified by the symmetric-but-nonlinear demag auto-advance. The right
firmware lever is to bound/replace that feedback in the marginal direction —
which is precisely what the overlay's physical-reverse-scoped
`apply_comm_timing_angle` does. Secondary physical suspects to verify on the
bench (all outside this firmware's phase logic): comparator/virtual-neutral
offset, per-phase FET/gate-drive timing skew, and the handoff rotor position /
`Flag_Motor_Dir_Rev` state the sine→6-step crossover asserts at the instant it
enters `run1` (if the crossover enters at a rotor angle the fixed run-state edge
does not expect, the first few reverse commutations will mis-time and seed the
demag ratchet).

---

## 7. BLHeli_S vs Bluejay — differences relevant to the above

Bluejay is a BLHeli_S fork; the commutation *state machine* (the six steps, the
A↔C reverse relabel, the fixed per-state comparator phase and edge, the
advance-split timing model) is inherited essentially unchanged from BLHeli_S —
so the §6 symmetry argument applies to both. Bluejay's divergences that touch
this analysis:

- **PWM generation & frequency:** Bluejay replaces BLHeli_S's fixed hardware PWM
  with a configurable, higher-frequency (24/48/96 kHz) scheme and selectable
  8–11-bit resolution (`Enums.asm:41-50`). Higher/cleaner PWM changes the demag
  behavior quantitatively (less ripple in the floating-phase reading) but not
  the direction symmetry.
- **DShot / bidirectional DShot & telemetry:** Bluejay adds full DShot command
  decode (`DShot.asm`) and GCR telemetry (`Macros.asm:30-101`); the 48 MHz clock
  switch at motor start (`Bluejay.asm:771-790`) and the `IF MCU_TYPE == BB2/BB51`
  divide-by-2 of the raw Timer2 count (`Timing.asm:68-73`) are Bluejay
  additions. These affect timing *scaling*, applied equally to both directions.
- **Comparator handling:** Bluejay supports hardware-inverted comparator output
  on BB2/BB51 (`Layouts/Base.inc:119-128`) and the BB51 fixed-mux
  `Set_Comparator_Phase_*` (`:135-155`). This is a per-*part* difference, not a
  per-*direction* one.
- **Demag metric / auto-advance:** the `adjust_comm_timing` demag→timing
  feedback (§4.3) and the sliding `Demag_Detected_Metric` exist in BLHeli_S as
  well; Bluejay retunes the constants (130/160 thresholds, minimum 120) but the
  structure — and its direction-symmetric-but-nonlinear nature — is shared.
  This is the mechanism §6 identifies, so the reverse-slip failure mode would
  exist on stock BLHeli_S too under the same load asymmetry.

Nothing in the BLHeli_S→Bluejay delta introduces a fixed-sense (non-mirrored)
direction dependence in the 6-step BEMF path.

---

## Appendix — primary citations

- Six-step FET table & comm routines incl. `_rev`: `Modules/Commutation.asm:44-58, 62-185`
- Comparator mux (no polarity) & PWM phase macros: `Layouts/Base.inc:124-219`
- Run loop, fixed high/low per state, run6_bidir: `Bluejay.asm:828-1024`
- Comm-period measure / averaging / 15° base: `Timing.asm:53-252`
- Advance split, presets, startup capture-widening: `Timing.asm:325-472`
- `wait_for_comp_out_high/low`, comp scan, timeout: `Timing.asm:587-736`
- `setup_comm_wait`, `evaluate_comparator_integrity`, `wait_for_comm`, demag metric: `Timing.asm:745-866`
- **Demag→advance feedback `adjust_comm_timing`**: `Timing.asm:367-387`
- Direction decode: `DShot.asm:105-136`, `Isrs.asm:231-256`, `Bluejay.asm:792-798`
- BlueGill physical-reverse-scoped advance override: `overlay:Modules/Timing.asm:524-553, 913-...`
