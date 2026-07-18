# #3 — Open-loop sinusoidal drive for BlueGill (design)

Status: design + reference-model (2026-07-14), branch `mas/bluegill-sine`. **All bench-pending**
(needs the low-KV motor). This is the first increment of the #3/#4 track from
`drive-methods-and-position-control.md`. Goal: smooth **ultra-low-speed** rotation (below the
~185 RPM 6-step floor) and **low-speed position holding**, by driving 3-phase **sinusoidal** PWM
open-loop (V/f, no current sense, no encoder), handing off to Bluejay's BEMF 6-step above a
crossover RPM.

---
## S1 as-built — forced-commutation "stepper" mode (shipped on `mas/bluegill-sine`)

**The rest of this document (from "Why sine" down) is the ORIGINAL true-3-phase-sine LUT
design sketch.** S2 (below) is now **as-built**, but it is a *min-clamp two-phase* SVPWM, NOT the
continuous 3-phase / 6-FET scheme sketched at the bottom (that would need 3 complementary PWM
pairs = 6 driven pins, which the 3 PCA channels + shoot-through safety cannot provide). S1 remains
the simpler 6-step forced-commutation stepper. All three modes share the `Pgm_Sine_Mode` param
(0=stock, 1=S1, 2=S2) and the "smooth sub-185-RPM rotation + zero-speed hold" goal.

**What S1 actually does (see `src/Modules/SineMode.asm`):**
- Opt-in via `Pgm_Sine_Mode` (0x2E, default 0 = exact stock). When set, `motor_start` branches to
  `sine_run` right after the stock commutation init pair — no BEMF timing, no startup ramp.
- A **fixed-point angle accumulator** advances at a host-commanded rate and steps the existing
  `comm1_comm2..comm6_comm1` vectors in **forward order only** (delta-based; re-seeded by the stock
  init pair on entry and after any direction flip). This is forced commutation, NOT sinusoidal —
  one of 6 discrete vectors at a time → a 42-detent/rev synchronous stepper (12N14P, 7 pp).
- **V/f duty**: `Sine_Amp = hold_amp + (inc>>8)`, clamped to `min(amp_max, Pwm_Limit)` and applied
  by `sine_set_duty` (Power.asm), which mirrors `t1_int`'s deadtime-skew / Pwm_Braking clamp
  byte-for-byte. At zero commanded rate the rotor sits on one vector at `hold_amp` → **holding
  torque** (thrust 0 = hold, no floor).
- Four params at 0x2E–0x31: `Sine_Mode` / `Sine_Hold_Amp` (~3%) / `Sine_Amp_Max` (~8%) /
  `Sine_Ramp` (slew). Rev 224→225, count 31→35. Amplitudes are decode-clamped (hold ≤ 40, amp_max
  ≤ 60) so a stale/hostile EEPROM can never command full duty.

**Host throttle → speed contract (bidirectional 3D mode):**
- Control tick = `SINE_TICK_T2` = 4000 Timer2 ticks = **1.000 ms** (Timer2 = SYSCLK/12 = 4 MHz on
  the 48 MHz BB21 core), i.e. a 1 kHz stepper update.
- Per-tick accumulator increment `inc = Rcp << 3` (Rcp = 11-bit throttle magnitude 0..2047); a
  sector step occurs on each 16-bit accumulator overflow. Hence **eRPM ≈ Rcp × 1.2207**, mech RPM =
  eRPM / 7, and **full scale (Rcp=2047) ≈ 2499 eRPM ≈ 357 mech RPM**. Sign of the step rate comes
  from `Flag_Rcp_Dir_Rev`; a direction change **only happens through zero** (decelerate to a stop,
  `switch_power_off`, flip, re-seed the init pair). Host thrust s∈[0,1000] → DShot (1000+s) → Rcp,
  so thrust=1000 ≈ full scale; the host `posctl.py` servo uses `FULLSCALE_RPM ≈ 357` and
  `Kff ≈ 0.467`. Exact numbers are printed by `tools/sim/sine_drive_model.py` (stepper section) and
  MUST match the asm EQUs.

**Telemetry:** `sine_run` never updates `Comm_Period4x` (no BEMF period to measure), so DShot eRPM
telemetry is **stale** in sine mode. Intentional — seeding it would corrupt other run-mode code;
the host closes its loop on the AS5600 encoder (`enc`).

**FET-safety invariants** are the header block of `src/Modules/SineMode.asm` (6 invariants, each
traceable to code). **All S1 firmware is bench-pending** (cautious hold→slow→servo ramp, low amps
first, watch tele temp) — see `docs/low-speed-tuning.md`.

---
## S2 as-built — min-clamp two-phase micro-stepping (`Pgm_Sine_Mode==2`, `Flag_Sine_Micro`)

S2 keeps S1's entire control skeleton (1 kHz Timer2 tick, fixed-point angle accumulator, V/f
`Sine_Amp`, slew, reverse-through-zero, thermal governor) and replaces only the FET drive: instead
of stepping one of 6 discrete comm vectors, it synthesises a **true rotating sinusoidal vector** by
min-clamp (flat-bottom / DPWM) modulation of two phases at a time. Result: no 8.6 deg-e detent
staircase — the rotor holds and creeps at *any* angle (192 microsteps/erev).

**Modulation.** At each electrical angle the most-negative phase is **clamped** to the negative rail
(its Com/low-side FET fully on — exactly what a stock comm vector already does). The other two
phases carry sinusoidal high-side duty `L[idx]·Sine_Amp/256`, where `L` is a 49-byte cosine-arc LUT
(`Sine2_Arc_Lut`, `L[j]=round(255·sin(j·1.875°))`). The reconstructed line-to-line voltage is
sinusoidal to **0.18 % RMS** (see `tools/sim/sine_drive_model.py print_s2_dpwm_section`, which emits
the exact LUT bytes and self-checks duty∈[0,255], line-line RMS <2 %, sector continuity).

**FET topology / why it cannot shoot through.** The two modulated phases are driven by:
- the **pair** phase (lower-lettered ⇒ lower Port-1 pins): the stock POWER+DAMP complementary pair
  (PCA modules 0/1) with the existing DEADTIME skew — proven-safe (S1 Inv 3);
- the **second** phase (higher pins): the otherwise-unused free PCA **module 2 / CEX2**, HIGH-SIDE
  ONLY. Its Com/low-side FET is **latched off**, so that leg freewheels through the body diode and
  can never shoot through, for any m2 duty.

The clamp phase's leg is low-side-on / high-side-off. So in all three per-segment states no leg ever
has both FETs commanded on. `pca_int` stays `reti` (module 2 is written synchronously from the tick,
like `sine_set_duty`, not from an ISR).

**Crossbar role-fixity (verified against the SiLabs EFM8BB2 Reference Manual).** The plan's
load-bearing claim — that the crossbar deterministically assigns CEX2 to the second phase's Pwm pin
without a shoot-through path — was confirmed from the RM (not present in the repo/lake), NOT merely
inferred:
- **§11.3.3 (Priority Crossbar Decoder), Fig 11.4:** each enabled resource takes the *least-
  significant un-skipped* pin; `PnSKIP` pins are skipped; PCA priority is fixed **CEX0 > CEX1 >
  CEX2**.
- **§11.4.2 (XBR1):** `PCA0ME=3` routes CEX0,CEX1,CEX2 (S2 uses `XBR1=03h`; stock `02h`).
- **§16.4.7 / §16.3.8:** CEX2 has independent polarity (`PCA0POL.2`, left at the stock 0 = same as
  CEX0, so `PCA0POL` is never rewritten) and per-channel center-alignment (`PCA0CENT.2`, already set
  at init).

With Layout A pin order `Ap=0<Ac=1<Bp=2<Bc=3<Cp=4<Cc=5` and the rule **"pair = lower-lettered
modulated phase"**, CEX0/CEX1 always bind to the pair's Pwm/Com (the two lowest un-skipped pins) and
CEX2 to the second phase's Pwm — for all three clamp segments and both directions. Module roles are
therefore fixed; only three static `P1SKIP` masks are needed: **clamp A → 0E3h, clamp B → 0ECh,
clamp C → 0F8h**.

**Remux ordering (geometry correction to the plan).** Under *min-clamp* DPWM the clamped phase is the
most-negative one, whose 120° windows are centred on each phase's negative peak, so the clamp
boundaries fall at θ = 90/210/330° — the **centres of the even comm sectors (2/4/6), not at sector
boundaries**. S2 re-muxes (`sine2_apply_segment`) only at those 3 points/erev, where the outgoing
CEX2 duty is ~0 (glitch-free). A remux, under `clr IE_EA`, FIRST forces `P1SKIP=0FFh` + all FETs off
(a full de-energise ⇒ no shoot-through window), THEN latches the new clamp Com on and un-skips the
pair+second pins; duties are written before the remux so the re-routed modules drive correct reloads
immediately. `sine2_hw_exit` restores `XBR1=02h` / `PCA0CPM2=00h` on every exit.

*Bench watch-item (safe, not a bug):* at the sector-4 remux (g=112, clamp C→A, pair A→B)
the new pair-B duty is written to modules 0/1 while they are still routed to the A pins, so
the A leg briefly PWMs at the (high) B duty for ~1–2 µs before `sine2_apply_segment`
de-energises and re-routes. It is deadtime-protected (no shoot-through) but is a small torque
glitch — watch for a faint tick once per erev at that sector on the bench. If it is objectionable
it can be removed by floating the crossbar (`P1SKIP=0FFh`) before the duty writes on a remux
tick; left as-is here to avoid churning the reviewed core.

**Reverse** mirrors the electrical sequence (`g_eff = 192 − g`) — no second table; bench BOTH
directions (double the FET surface).

**Host contract.** S2 reuses the S1 params and the *identical* speed/`Sine_Amp` law, so
`posctl.py` is unchanged (`FULLSCALE_RPM ≈ 357`, `Kff ≈ 0.467`). The torque *shape* is smoother but
the peak per-phase duty at a given `Sine_Amp` differs from S1's square drive, so `sine_hold_amp` may
need a bench retune for mode 2. EEPROM layout revision is **unchanged (225)** — S2 adds no new
params, so esctool/firmware stay in lockstep. Profile: `host/profiles/posctl_930kv_sine2.yaml`
(timid: temp-prot on, hold 8 / amp_max 20). **All S2 firmware is bench-pending on live FETs** — this
is a novel topology; validate hold → slow creep (both dirs) → posctl accuracy with temp-prot on.

## S3 as-built — bidirectional sine ↔ BEMF-6-step crossover with hysteresis

S3 hands a **spinning** rotor between forced sine (S1/S2) and the stock closed-loop BEMF 6-step run
loop, both ways, gated by two new EEPROM params so modes 0/1/2 are byte-identical when it is off.

**Triggers (two thresholds, INVERSE units — this is the easy thing to get wrong).**
- **Up-handoff** (forced-sine → BEMF), param `Cross_Up` (0x32, `Pgm_Sine_Cross_Up`, in `Sine_Inc_H`
  units, ≈ **39.0625 eRPM/unit**, larger = faster). In `sine_tick`, once per 1 kHz tick, a debounce
  counter `Sine_Cross_Cnt` inc-saturates while `Sine_Inc_H ≥ Cross_Up` **and** the commanded
  direction matches the spinning direction (it resets on any direction-change-pending or
  below-threshold tick). The handoff fires **only at a sector-6→1 boundary** (phase-aligned), when
  `Cross_Up ≠ 0`, `Sine_Cross_Cnt ≥ SINE_CROSS_DEBOUNCE (16)`, and the measured **4-sector** window
  `Sine_Step_Ticks` is in **[`SINE_CROSS_TICKS_MIN=9`, `SINE_CROSS_TICKS_MAX=30`]**.
  `Sine_Step_Ticks` is a whole-tick counter reset on entering sector 3 and read on entering sector 1
  (= duration of sectors 3+4+5+6). The `≤30` bound is the **BEMF speed floor** (~40000/30 ≈ **1333
  eRPM**, just above the stock 6-step floor; also keeps the seed 16-bit); the `≥9` bound **caps the
  one-shot seed quantization** to ~1/9 ≈ **11 %** so a too-fast crossover config can never silently
  seed BEMF with a badly-wrong period — a shorter window *refuses* the handoff (rotor stays safely in
  forced sine), and the host `--sine-crossover-erpm` rejects such configs up front (band
  ~1333–4444 eRPM). A 4-sector window (vs 2) halves the quantization for the same eRPM.
- **Down-handoff** (BEMF → forced-sine), param `Cross_Dn` (0x33, in `Comm_Period4x_H` units, ≈
  **312500 eRPM/unit, INVERSE**, larger byte = slower). Checked in `run6_check_speed` (normal-run
  path only, never during startup/initial-run) **before** the stock `0F0h` min-speed exit: if
  `Flag_Sine_Mode` and `Cross_Dn ≠ 0` and `Comm_Period4x_H ≥ Cross_Dn`, set `Flag_Sine_Handoff` and
  `ljmp motor_start` (the proven stall-restart shape). `Cross_Dn` is clamped to `≤ 0xEF` at decode so
  it always fires before `0F0h`.

**The `Comm_Period4x` seed (safety-critical — a factor-2 error desyncs a live rotor).** The BEMF run
loop needs `Comm_Period4x` (its notion of rotor speed) seeded to the *actual* rotor speed at handoff.
`calc_next_comm_period` timestamps run in **halved-TMR2 ticks** on BB2 (it divides the raw 4 MHz
Timer2 by 2). One commutation = **one 60° electrical sector**, and `Comm_Period4x` (the "4x" period) =
**4 commutations = 4 sectors**, in halved-TMR2 counts. `Sine_Step_Ticks` spans **sectors 3→6 = 4
sectors** (reset entering sector 3, read entering sector 1), so it directly measures the 4-sector
`Comm_Period4x` span — no ÷2×2 sleight-of-hand. Each control tick is `SINE_TICK_T2 = 4000` raw Timer2
counts (`Bluejay.asm:201-202`) = **2000 halved-TMR2 counts**, so `Comm_Period4x = Sine_Step_Ticks ×
SINE_TICK_T2/2 = ticks_4sector × 2000` (one 8×16 multiply, `×0x07D0` via `mul #0D0h`/`mul #07h`). That
is exactly **80e6/eRPM** — the vendor eRPM↔period mapping — so the run loop resumes at the true rotor
speed. Cross-check @2000 eRPM: `Sine_Inc = 13107 → 5 ticks/sector → ticks_4sector = 20 → seed =
20 × 2000 = 40000 = 80e6/2000` ✓ (the `sine_drive_model.py` S3 self-check asserts 0.0 % error).

`Prev_Comm = (TMR2_now>>1) − Comm_Period4x/4` back-dates the previous commutation by exactly one
commutation, so the single priming `calc_next_comm_period` measures `this_period = seed/4` and its
average is invariant: `4T − 4T/8 + T/2 = 4T` (with `T = seed/4`), for all three speed branches
(div_16_4 / div_8_2 / div_8_2_slow — the last fires here since we set `Flag_Initial_Run_Phase`). The
seed's ±1-tick quantisation (±1 in a 20-tick window ≈ ±5 %, bounded to ≤~11 % at the fast end by the
`TICKS_MIN` gate) is a one-shot error the BEMF averaging then re-locks. All of this is proven with
self-checks in `sine_drive_model.py print_s3_crossover_section()` (seed vs 80e6/eRPM within **±1 tick
AND ≤12 % relative** for every handoff-able config — a config that would ship a coarser seed FAILS the
sim; averaging invariance; both threshold conversions mirrored from the host).

**Root cause — the direction convention is inverted (bench-proven).** BlueGill's forced-sine field
convention is the **opposite physical sense** to stock 6-step comm for the *same* `Flag_Motor_Dir_Rev`.
On the bench, a mode-0 (pure 6-step) `+cmd` drove the encoder **−943°**, while forced sine at `+cmd`
drives the encoder **+enc** (forward) — i.e. **sine-forward == 6step-reverse**. The up-handoff entered
the 6-step run loop carrying sine's `Flag_Motor_Dir_Rev`, so every `comm*_comm*` (which branches on that
flag) and every `run*` comparator wait ran the *reverse* polarity and drove the rotor **backward**,
reverse-locking a rotor that was spinning forward. This is the primary cause of the S3 crossover stall,
and reconciling the direction (the `cpl Flag_Motor_Dir_Rev` up-flip) is the load-bearing fix.

**Why the BEMF catch is ALSO retained (direction ⊕ sector).** Bench-testing the direction fix *alone*
(blind sector-1 seed) showed it rode the **up-handoff forward** (229 → 678 RPM ✓), but the high-speed
6-step then **destabilised and reversed on ramp-down** — the blind sector-1 energise gives only marginal
high-speed synchronisation (the rotor can be spinning several sectors past sector 1 at handoff). Now that
the *direction* is correct, the *entry sector* still matters: the BEMF *catch-a-spinning-rotor* machinery
(`sine_catch_detect`/`_read`/`_lookup` + `Sine_Catch_Table`) reads the coasting rotor's actual sector so
the run loop resumes at `run_s` with **both direction and sector aligned**. Direction fix + catch are
complementary: the flip makes torque forward, the catch makes the *entry point* correct. The catch costs
≈ **110 bytes** of app flash; to fit under the `0x19FD` ceiling the non-essential **startup beep melody**
(`play_beep_melody`) was removed (DShot telemetry replaces audio identification here) — see *Cost /
lockstep* below. `tools/sim/catch_truth_table.py` is retained (it re-derives `Sine_Catch_Table`).

**The governing invariant.** In a sine config (`Flag_Sine_Mode` set): the **6-step loop runs
`Flag_Motor_Dir_Rev == NOT Flag_Rcp_Dir_Rev`**, while **forced sine runs `Flag_Motor_Dir_Rev ==
Flag_Rcp_Dir_Rev`**. Both handoffs are just symmetric re-assertions of this invariant; mode 0/1/2 with
`Flag_Sine_Mode` clear reduce exactly to stock. The only writers of `Flag_Motor_Dir_Rev` are
`motor_start` (:913/:918), the `run6_bidir` post-brake assignment, `sine_tick` (:307, forced-field
stepping), and the two new handoff sites below.

**Up-handoff sequence (FET-safe, single-writer-preserving).** When the gate passes, `sine_cross_update`
only **arms `Flag_Cross_Up_Armed`** and `ret`s up the call chain; `sine_run_loop` (at its own baseline
stack depth) then `ljmp`s `sine_cross_up`. So there is **no manual `SP` surgery and no coupling to the
`sine_tick`/`sine_cross_update` call depth** — a future refactor of that chain cannot silently smash the
stack; `run1` runs at exactly the depth the stock startup path uses. `sine_cross_up` then: `clr IE_EA` →
`switch_power_off` → if `Flag_Sine_Micro` `sine2_hw_exit` (which re-enables IE — we re-`clr IE_EA`;
`sine_read_timer2` below likewise re-enables IE, both safe because `Flag_Sine_Run` stays set) → at
`sine_cross_up_hw_done` **`cpl Flag_Motor_Dir_Rev`** (the up-flip: it MUST precede the catch detect AND
the energise below, because `sine_catch_lookup` swaps A↔C under this flag and `sine_step_sector`'s `comm*`
branches on it — so the flag must already hold the 6-step-forward convention before we read/look-up/
energise the sector that continues the rotor's *actual* forward spin) → **`call sine_catch_detect`**
(reads the coasting rotor's BEMF sector; `jnz` → detected state `s` in `A`; on failure `A=0` →
`ljmp motor_start` flat-stack re-grab) → store the **predecessor** `Sine_Sector = s-1` (wrap 1→6) →
seed `Comm_Period4x`/`Prev_Comm` → pre-arm Timer3 (see below) → `setb Flag_Initial_Run_Phase` /
`mov Startup_Cnt,#1` / `clr Flag_Startup_Phase` / `Initial_Run_Rot_Cntd=#12` / `clr Flag_High_Rpm` →
`Pwm_Limit=Pwm_Limit_By_Rpm=Pwm_Limit_Beg` (conservative) → energise the **detected** state `s` from
all-off via a single `call sine_step_sector` (steps predecessor `s-1` → `s`, i.e. `comm(s-1)_comm(s)`, an
absolute overwrite — see *Energising from all-off* below) → **only now** `clr IE_EA` / `clr Flag_Sine_Run`
/ `setb IE_EA` (the single-writer gate opens under IE off, after the conservative limit, so `t1_int` can
never write a full-`Pwm_Limit` duty in the window) → one `calc_next_comm_period` → dispatch
`jmp @A+DPTR` through a 6-entry `ljmp run1..run6` table to `run_s`. With direction flipped AND the entry
sector detected, closed loop locks on the first zero-cross (seeded `Comm_Period4x` + `Startup_Cnt=1`) at
forward torque. Bench-proven forward ride (direction fix) **229 → 678 RPM** through the up-handoff; the
catch adds the correct entry sector to stabilise the high-speed 6-step on ramp-down. Initial-run for 12
rotations means the down-handoff cannot fire immediately after an up-handoff (anti-chatter on the BEMF
side).

**`run6_bidir` conjugation (the running-direction test in the 6-step loop).** Once in 6-step, the
bidirectional reversal-brake logic must respect the inverted convention or it would treat a normally
running sine-config motor as a commanded reversal and brake it every loop. The mismatch test and the
post-brake assignment are conjugated with the invariant: the **expected** running direction is
`(Flag_Rcp_Dir_Rev XOR Flag_Sine_Mode)` — `NOT Flag_Rcp_Dir_Rev` in a sine config, `== Flag_Rcp_Dir_Rev`
otherwise. It is built into `C` with `mov C,Flag_Rcp_Dir_Rev` / `jnb Flag_Sine_Mode,skip` / `cpl C` /
`skip:` (8051 `jb`/`jnb` never touch `C`, so it survives the interleaved bit tests), then the loop brakes
iff `Flag_Motor_Dir_Rev != C`, and the post-brake sets `Flag_Motor_Dir_Rev := C`. **With `Flag_Sine_Mode`
clear this reduces exactly to the stock `Flag_Rcp_Dir_Rev` comparison and assignment** — mode 0/1/2 are
behaviourally untouched (verified across all 8 combos of `(Flag_Sine_Mode, Flag_Rcp_Dir_Rev,
Flag_Motor_Dir_Rev)`: a steady sine-config run never spuriously reverses, and a genuine command reversal
still brakes to `Motor == NOT Rcp`).

**Down-handoff un-flip.** On the way back (BEMF → forced sine), `motor_start` re-derives
`Flag_Motor_Dir_Rev` from `Flag_Pgm_Dir_Rev`/`Flag_Rcp_Dir_Rev` (`Bluejay.asm:912-918`) — it knows
nothing of sine's running direction. Sine's convention is `Flag_Motor_Dir_Rev == Flag_Rcp_Dir_Rev`, so
`sine_run` re-asserts it on the handoff path only: right after `clr Flag_Sine_Handoff` (with `IE_EA`
already off since sine_run entry, so the write is atomic) `mov C,Flag_Rcp_Dir_Rev` / `mov
Flag_Motor_Dir_Rev,C`, before the seed-math. Without this the field re-entered *backward* and the bench
motor **decayed to zero RPM** at `cmd ≈ 600`; with the symmetric un-flip sine resumes forward. The
normal-start path (`sine_run_inc_zero`) gains no instructions — an ordinary sine start already runs the
`motor_start` derivation and needs no correction.

**Closed-loop from commutation #1 (`Startup_Cnt`).** The comparator-acceptance gate is `Startup_Cnt`,
not `Flag_Initial_Run_Phase`. `wait_for_comp_out_*` DISCARDS every good comparator reading while
`Startup_Cnt==0` (`Timing.asm:661-662` — "force a timeout for the first commutation"), so the run
loop can only advance on zero-cross **timeouts** = blind forced commutation, which desyncs and reverses
a spinning rotor. `Startup_Cnt` is incremented only under `Flag_Startup_Phase`, which the sine path
never runs (it `clr`s it), and `motor_start` zeroed it — so it stays `0` unless we set it. `sine_cross_up`
sets `Startup_Cnt=#1`, which makes the comparator ACCEPTED from the first zc scan = real closed-loop
BEMF from commutation #1. It is side-effect-free (only zero/nonzero is ever tested on the sine path; no
magnitude reader is reached). We keep `Flag_Initial_Run_Phase` set (it is the *tolerant tracked* window
— widened `Comm_Period4x/2` zc timeout, demag preclear, no run-mode-exit on a missed zc, slow
averaging, and the 12-erev `Pwm_Limit` lift — NOT a blind regime; clearing it would strand `Pwm_Limit`
at `Pwm_Limit_Beg` and bounce out of run mode on the first missed zc).

**Catch a spinning rotor (`sine_catch_detect`).** While the rotor **coasts** (all FETs off from
`switch_power_off`, so the open phases carry back-EMF), `sine_catch_detect` reads the sign of each phase
against the neutral node through the comparator — `Set_Comparator_Phase_X` selects the mux, `CMP_CN0` bit
`0x40` set = phase > neutral = BEMF **positive** (`COMPARATOR_INVERT=0`) — forming a 3-bit pattern
`bit2=A, bit1=B, bit0=C`. Over a full electrical revolution the three signs give **6 valid codes, one per
60° sector** (no 180° ambiguity). Reads run with `IE_EA` off and no FET energised. Two filters guard
acceptance: **(1) a FIXED initial demag settle** (`SINE_CATCH_DEMAG_SETTLE`, ~450 µs) before the first
read so both reads land past the body-diode flyback (a read inside the flyback returns a *stable, valid,
WRONG* stale-field sector the consecutive-equal check can't reject); **(2)** within that clean regime a
pattern is accepted only when **two consecutive reads agree** and the code is valid (≠ 000/111), bounded
by `SINE_CATCH_RETRIES` (~2 ms), earliest stable+valid pair wins. On exhaustion it returns 0 and the
handoff falls back to `motor_start` (flat stack; `Flag_Sine_Mode` still set → re-enters `sine_run` and
re-arms). The lookup maps each pattern to the run whose **next** zero-cross fires next in that sector,
giving forward torque (field-to-rotor angle stays 60–120° ⇒ ≥ 0.87 × peak torque):

| pattern ABC | idx | 60° sector | next zero-cross | enter |
|:-----------:|:---:|:----------:|:---------------:|:-----:|
| 110 | 6 | 0–60°   | B↓ | `run2` |
| 100 | 4 | 60–120° | C↑ | `run3` |
| 101 | 5 | 120–180°| A↓ | `run4` |
| 001 | 1 | 180–240°| B↑ | `run5` |
| 011 | 3 | 240–300°| C↓ | `run6` |
| 010 | 2 | 300–360°| A↑ | `run1` |
| 000 / 111 | 0 / 7 | — | balanced 3-phase sum ≈ 0 ⇒ **INVALID** | retry → fallback |

So `Sine_Catch_Table[idx0..7] = 0,5,1,6,3,4,2,0`, derived from the BEMF model `Ea=sin θ`,
`Eb=sin(θ−240°)`, `Ec=sin(θ−120°)` (the only convention whose zero-cross order matches the hardware
`A↑0 B↓60 C↑120 A↓180 B↑240 C↓300`), cross-checked against the stock float-phase and `run1..run6`
comparator-wait polarities, and re-derived independently by `tools/sim/catch_truth_table.py` (asserts
this exact sequence + the reverse rule). **Reverse** (`Flag_Motor_Dir_Rev`, which the up-flip has already
set to the 6-step-forward convention before detect): the `_rev` comm routines swap comparator phase A↔C
on runs 1/3/4/6 (2/5 keep B) — exactly a **bit A(2)↔C(0) swap of the read pattern followed by the SAME
table** (`sine_catch_lookup`). Catch parameters are HW-only and bench-tuned; they are held at their
proven values (retries/settle unchanged — the flash budget for the catch came from removing the startup
beep melody, not from reducing catch robustness).

**Energising the detected state from all-off.** `switch_power_off` leaves every FET/phase in a
fully-defined all-off state, and the stock comm macros are *absolute* register overwrites
(`Set_Pwm_Phase_X` rewrites all of `P1SKIP`, `Set_Comparator_Phase_X` rewrites `CMP_MX`, the
`*_Fet_On/Off` macros drive their own pins), so any single direction-aware `comm(k)_comm(k+1)` fully
establishes that state's conduction **and** the matching comparator phase for the current (now flipped)
direction — each branches on `Flag_Motor_Dir_Rev` internally, exactly as the stock `motor_start_bidir_done`.
That is why `sine_step_sector` can jump straight to the **detected** state `s` from all-off with no
pre-kick: seed the predecessor sector `s-1` (wrap 1→6) so its step calls `comm(s-1)_comm(s)`, which
overwrites the lot. No `comm5_comm6` pre-kick is needed (from all-off it would only flash a momentary
backward vector). `sine_run`'s own entry instead keeps the stock `comm5_comm6`+`comm6_comm1` pair because
it re-seeds sector 1 while the drive is still live.

**Timer3-arming hazard — BOTH the count AND the reload are stale (the first-power-cycle BEMF-lock bug).**
The single priming `calc_next_comm_period` falls through into `wait_advance_timing`, whose
`Wait_For_Timer3` blocks **only while `Flag_Timer3_Pending` is set** and consumes a Timer3 wrap. Sine
mode skips `initialize_timing`, so **both** the Timer3 *count* AND its *auto-reload* register `TMR3RL`
are stale at handoff, and **two** Timer3 wraps happen before the first ZC scan opens — each must be
bounded:

- **wrap#1** (the priming `wait_advance_timing`'s `Wait_For_Timer3`): `sine_cross_up` **pre-arms the
  Timer3 count from the live seed**, mirroring `setup_zc_scan_timeout`'s SFR order (`Timing.asm:561-573`):
  with `Temp4:Temp3 = Comm_Period4x/4` still in registers from the `Prev_Comm` back-date, one more `rrc`
  pair gives `Comm_Period4x/8`, exactly the 15°-el `TMR3` magnitude (15° el = `Comm_Period4x/16` in
  halved-`TMR2` counts; `Timer3` runs at 2× that rate via the BB2 ×2 in `calc_new_wait_times`, so the
  count is `2·Comm_Period4x/16 = Comm_Period4x/8`; the ÷4→15° and ×2→Timer3 factors cancel to a single
  right shift). `setb Flag_Timer3_Pending` makes the priming wait consume this bounded ~15°-el wrap.
- **wrap#2** (the hardware auto-reload `TMR3 := TMR3RL` that fires when wrap#1 completes):
  `setup_zc_scan_timeout_startup_done` writes only `TMR3L/H`, **never `TMR3RL`** (vendor
  `Timing.asm:560-574`). On the **first handoff per power cycle** `TMR3RL = 0x0000`, so wrap#2 = **65536
  counts ≈ 16 ms of *energised* clamp** on the caught commutation vector *before* the first ZC scan can
  open — that destroys the catch's rotor alignment and the run loop then free-runs in the measured
  **601 ↔ 6620 eRPM** instant-accept/timeout limit cycle. (`t3_int` reloads `TMR3RL := −6` on every
  serviced wrap, so from the 2nd wrap on it is benign — only this first wrap was exposed; and the
  bench restart re-inits each run, so it reproduced every run.) **Fix:** seed `TMR3RL := −16`
  (`mov TMR3RLL,#0F0h` / `mov TMR3RLH,#0FFh`, +6 bytes) right at the pre-arm, next to the count seed,
  bounding wrap#2 to a few µs. These are direct-to-SFR immediate loads — they clobber no
  `A/B/Temp*/PSW/Sine_Sector`, so the predecessor `s-1` in `Sine_Sector` survives to the energise.

**Fixed entry timeline (first handoff).** `cpl Flag_Motor_Dir_Rev` → catch detect → energise state `s`
→ gate release → `calc_next_comm_period` → **wrap#1** = ~15°-el pre-armed window (bounded) → HW reloads
`TMR3 := −16` → **wrap#2** falls through in ~µs (bounded) → `wait_before_zc_scan` → `setup_zc_scan_timeout`
arms `TMR3 = −(Comm_Period4x/2)` for the real ZC-timeout window → the first ZC scan opens ~15°-el after
entry, with `Startup_Cnt=1` accepting the comparator on that first scan. The wrap#2-vs-`setb
Flag_Timer3_Pending` race is benign in **both** orders (one consumer per wrap; an early wrap merely
clears the pending flag and the priming wait falls through harmlessly). We deliberately do **not** call
the stock final `initialize_timing` (it resets `Comm_Period4x` to `0x00F0` and would wipe the seed). If
the bench is still marginal, the documented fallback is to shrink the wrap#1 pre-arm magnitude (scan
opens nearer 0° el) and/or back-date `Prev_Comm` by `1.5·CP/4`.

**Down-handoff re-entry.** `Flag_Sine_Handoff` is `Flags3.6` — `Flags3` survives `motor_start`'s
`Flags0/1` wipe. `sine_run` consumes+clears it and pre-seeds `Sine_Inc` from `Sine_Inc_Seed =
2048000/Cross_Dn` (computed once at decode via the generalised `div_2048000` core; `2048000 = 312500 ×
65536/10000`) instead of `Sine_Inc=0`, so the field re-enters at ~rotor rate rather than re-
accelerating from stop. A decode-time guard disables the whole crossover (zeros both thresholds) if
`Cross_Up==0` or `Sine_Inc_Seed_H ≥ Cross_Up` — the latter is provably equivalent to `dn_eRPM ≥
up_eRPM`, i.e. no hysteresis window / instant up-retrigger = chatter.

**Chatter analysis.** Three independent margins prevent limit-cycling between the two regimes: (1) the
`Cross_Up`/`Cross_Dn` **speed** gap (host rejects `dn_eff ≥ up_eff`); (2) the `SINE_CROSS_DEBOUNCE=16`
(~16 ms ≈ 3 sectors @2000 eRPM) time gap on the up side; (3) the 12-rotation initial-run lock-out on
the BEMF side after an up-handoff. Under heavy load a slow limit cycle is still possible — tune
`N`/thresholds at the bench.

**Host contract — the discontinuity (RESOLVED in-firmware, 2026-07-17).** Forced sine is
**thrust = speed** (open-loop V/f, host closes position on the AS5600 `enc`); stock BEMF run is
**thrust = duty** (the DShot throttle maps to PWM duty, speed is load-dependent). At the crossover the
same throttle meant different things, so a manual ramp jumped from ~`Cross_Up` speed (~212 mech) to
the open-loop duty's natural speed (~6233 mech) in one step, and the mid-range was unreachable.
`cross_rescale_duty` (SineMode.asm) now fixes this **in the ESC**: while the crossover is engaged in
6-step it remaps the demanded duty to a **deterministic affine of the throttle** anchored so the
handoff throttle maps to a small `CROSS_DUTY_MIN` seed and full throttle to full duty. thrust->speed
is now continuous and monotonic across the seam (bench: 205->486 mech then a smooth ramp to full,
both directions, slip ~1.0 throughout; cmd 571 holds ~1175 mech where it used to jump to ~6570). It is
a pure function of throttle+config (no run-time capture) so the same command gives the same steady
speed under a given load. Load-INDEPENDENT speed (same thrust -> same speed regardless of load) would
still need a closed speed loop; the affine only removes the discontinuity, which was the actual
problem. Minor open item: reverse shows a small ~390 mech dead-band between its handoff and Rcp_cross
(reverse reaches `Cross_Up` at a slightly lower throttle than forward) — smooth, not a jump.

**Reverse lock — RESOLVED (it was a measurement artifact, 2026-07-17).** An earlier increment added a
*reverse-only* absolute commutation-timing override (`0x2B` scoped to physical reverse) to chase an
apparent "reverse won't lock / over-commutates 3–21×". That was **reverted** (commit follows): the
6-step BEMF loop locks in **both** directions with the SAME config (`comm_timing=MediumHigh`,
`demag=High`, `comm_timing_angle=0`) — bench-confirmed slip ≈ 1.0 forward and reverse, symmetric at
cmd ±571. The "reverse failure" never existed at the motor: it was **Nyquist aliasing** in the host
tooling (the rotor really spins ~6000–9000 mech in 6-step, far past the host's 50 Hz sampler; the
encoder folded — worse in reverse, which ran slightly faster), compounded by a **host telemetry
double-division** (the tele line is already mechanical RPM, and the host divided by pole pairs again,
so a true lock read as ~0.143 = 1/7). Both were fixed in `pico-esc-tool` (on-device de-aliased
`encv` + removing the double-division). Firmware needed **no** direction-specific timing. See
`docs/sensorless-6step-analysis.md` for the symmetry proof of the 6-step core.

**Cost / lockstep.** EEPROM layout revision **225 → 226**, `EEPROM_B2_PARAMETERS_COUNT 35 → 37`; the
two params append after `Pgm_Sine_Ramp` (RAM 0xAF/0xB0), `Stack` relocated to `ISEG AT 0B2h`,
`Sine_Inc_Seed_L/H` at `ISEG AT 0C2h` (Keil map verified: params at 0xAF/0xB0, `Stack` 0xB2–0xC1,
`Sine_Inc_Seed` 0xC2–0xC3, no overlap with `Temp_Storage@0xD0`). The rev/count bump ships atomically
with `esctool.py` (adds `sine_cross_up`/`sine_cross_dn` + the `--sine-crossover-erpm UP,DN` convenience)
and `esc_tool.cpp` `FIELDS[]`. **Flash is the binding constraint.** The app carries BOTH the direction
fix (≈ 14 B: one `cpl`, the down-handoff un-flip, and the `run6_bidir` `Flag_Sine_Mode` conjugation) AND
the re-added **catch-a-spinning-rotor** machinery (`sine_catch_detect`/`_read`/`_lookup` + `Sine_Catch_Table`
+ `jmp @A+DPTR` dispatch, ≈ 110 B); together they overran the `0x19FD` ceiling on the 48 kHz build by
**9 bytes**. To fit — and per the explicit priority *"free flash from beeps before cutting catch
robustness"* — the non-essential **startup beep melody** was removed: `play_beep_melody` is stubbed to a
bare `ret` in a new `src/Modules/Fx.asm` overlay and its single call in `Bluejay.asm` deleted (~39 B
freed). DShot bidirectional telemetry provides identification/health here, so the audio startup jingle is
redundant; **all functional beeps** (RC-detect, arm, beacon locate, signal-lost, motor-stalled,
bootloader) are UNCHANGED, and the catch's tuning constants (`SINE_CATCH_RETRIES` / `SINE_CATCH_SETTLE` /
`SINE_CATCH_DEMAG_SETTLE`) stay at their proven values. The later **TMR3RL seed fix** adds **+6 bytes**
(two direct-to-SFR immediate loads). Net: the app CSEG top (`?CO?BLUEJAY?20`) is
**`0x19D8` (24 kHz) / `0x19E5` (48 kHz)** — **37 / 24** bytes clear of the `0x19FD` ceiling. The Timer3
pre-arm still calls the vendor `setup_zc_scan_timeout_startup_done` (trim retained). **All S3 firmware is
bench-pending on live FETs** across two control regimes — validate manual up/down ramps, both directions.

> Build note: the Keil linker emits `WARNING L30: MEMORY SPACE OVERLAP` at `I:0x20–0x28`. That is
> **pre-existing** (the vendor's bit-addressable `DSEG AT 20h` region — `Bit_Access`/`Flags0-3` — which
> intentionally overlaps the direct-addressed data segment there). It is **NOT** caused by the S3
> `Stack`→`0xB2` relocation, whose ISEG lives at 0xAF–0xC3, nowhere near `0x20`.

---
## Why sine, and the hard boundary
6-step trapezoidal energizes 2 of 3 phases with a rotating block → strong torque ripple and
unreliable sync at very low RPM (weak BEMF). Sinusoidal drive rotates a smooth voltage vector, so
the rotor follows synchronously like a stepper — smooth at arbitrarily low speed and it can *hold*
a commanded angle (limited holding torque, set by applied voltage). Cost: open-loop sine has **no
feedback**, so under load/at speed it can lose sync and it wastes power (no BEMF-optimal timing).
Hence the **hybrid**: sine only below a crossover; Bluejay 6-step (unchanged) above.

## Hardware reality (from vendor/bluejay/src, verified)
- 8-bit 8051 @ ~24.5 MHz; PWM is timer/ISR-driven, resolution 8..11-bit (`Isrs.asm` PWM_BITS).
- Complementary/damped FETs with DEADTIME (our A_H_30 target = 30) — sine needs center-aligned
  complementary duty per phase; dead-time already handled by the existing PWM path.
- **No current sensing** → torque is set only by applied voltage amplitude and load angle; there is
  no current loop and no overcurrent trip beyond the coarse demag/temperature limiters. **Thermal
  headroom must be bounded in firmware** (cap sine amplitude; watch the temp ADC).
- The BEMF comparator (`Modules/Timing.asm`) still works — used to detect sync for the crossover.

## Architecture
1. **Enable + crossover** — new BlueGill EEPROM param `Pgm_Sine_Mode` (appended at 0x2E, off=0 =
   pure stock) and `Pgm_Sine_Crossover_Rpm` (0x2F). Off ⇒ exact Bluejay behaviour.
2. **Sine LUT** — a quarter-wave table in code (CSEG); full-wave reconstructed by symmetry to save
   flash. Reference model emits the exact bytes. Third-harmonic / SVPWM injection (+15% linear
   range) optional; start with pure sine for simplicity.
3. **Angle integrator** — electrical angle θ advances by Δθ = ω·Δt each PWM period; ω from the
   commanded speed. mech RPM = eRPM / pole_pairs; eRPM = 60·f_elec; our motor 7 pole-pairs.
4. **3-phase duty write** — duty_X = mid + amp·sin(θ + kX), kA/kB/kC = 0/120/240°, mid = 50% (the
   complementary zero). Reuses the phase-PWM FET macros (`Set_Pwm_Phase_*` / the ISR duty regs).
5. **V/f amplitude schedule** — amp rises with commanded speed from a small `boost` floor (holding
   voltage at 0 speed) — the classic V/Hz curve; clamp amp ≤ a thermal-safe max.
6. **Startup & handoff** — start in sine open-loop from standstill (no blind 6-step); ramp θ; once
   eRPM > crossover and BEMF zero-crossings are consistent, hand off to Bluejay's run loop
   (`Flag_Startup_Phase` clear, seed `Comm_Period4x` from the sine ω). Downward: below crossover −
   hysteresis, re-enter sine.
7. **Host interface** — in sine mode the DShot throttle maps to a **speed setpoint** (and, under
   #4B, the Pico's position loop sets it); a future signed/thrust mapping gives slow bidirectional
   crawl. Telemetry (eRPM/EDT) unchanged.

## Firmware cost / risks
- **Angle-update rate** bounds max electrical frequency: the 8051 must recompute 3 duties every PWM
  period at low speed (cheap) — fine, since sine is only used at low RPM. Budget verified in the
  reference model (cycles/update estimate).
- Replacing/duplicating the startup path is the riskiest change — keep 6-step startup available via
  the off default; sine is opt-in until bench-proven.
- No current sense → **amplitude clamp + temp watch are mandatory** before enabling on hardware.
- Largest firmware effort in the project → its own orchestrate loop once the low-KV motor is here.

## Encoder usage in #3 — DEBUG / VALIDATION ONLY (not in the loop)
The #3 sine drive is **open-loop feed-forward**: the ESC generates the commanded electrical
angle from the angle integrator with NO rotor feedback. The AS5600 encoder (on the Pico,
`enc` command in esc_tool) is used **only to debug and check** the sine drive — measure whether
the rotor actually follows the commanded angle, detect slip/step-out, and quantify low-speed
smoothness/cogging — it does **not** close the drive loop. Closed-loop use of the encoder is #4B
(a separate outer position loop on the Pico), below. Keep the two concerns distinct: #3 = smooth
open-loop rotation; encoder = an instrument to verify it.

## #4B linkage (encoder position control, Pico-side)
The Pico reads the encoder → position/velocity PID → speed/angle setpoint. Above crossover it
commands throttle (ESC 6-step, as today, no firmware change). Below crossover / for holding, it
commands sine mode. #4B host prototype (in pico-esc-tool) can be built + simulated now; it needs
sine mode (this doc) only for the true low-speed/hold regime.

## What's validatable NOW (no hardware) — see `tools/sim/sine_drive_model.py`
- Generate the exact sine LUT bytes for the asm (chosen resolution + PWM scale).
- Verify 3-phase balance (Σ of the three duties ≡ 3·mid, i.e. neutral is constant) across θ.
- Quantify LUT quantization error vs ideal sine at 8/9/10/11-bit PWM.
- Fixed-point angle-integrator model: Δθ per PWM period for a target mech RPM (7 pole-pairs),
  and the resulting f_elec range vs the crossover.
- Optional third-harmonic injection gain check (+15% linear range, still Σ-neutral).
