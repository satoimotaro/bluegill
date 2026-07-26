;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Bluejay digital ESC firmware for controlling brushless motors in multirotors
;
; Copyleft  2022-2023 Daniel Mosquera
; Copyright 2020-2022 Mathias Rasmussen
; Copyright 2011-2017 Steffen Skaug
;
; This file is part of Bluejay.
;
; Bluejay is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; Bluejay is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with Bluejay.  If not, see <http://www.gnu.org/licenses/>.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill S1 forced-commutation "stepper" mode
;
; An opt-in (Pgm_Sine_Mode != 0) drive that abandons BEMF sensing and instead
; steps the stock 6-step commutation vectors at a HOST-COMMANDED rate through a
; fixed-point angle accumulator, with a V/f duty schedule. This lets the rotor
; creep smoothly far below the ~185 RPM 6-step floor and, at zero commanded rate,
; sit on a commutation vector with a small DC hold current -> zero-speed holding
; torque (a 42-detent/rev synchronous stepper on a 12N14P). It is the highest
; FET-risk path in the firmware; the invariants below are load-bearing.
;
; ------------------------------------------------------------------------------
; FET-SAFETY INVARIANTS (each one is traceable to the code marked [Inv n])
; ------------------------------------------------------------------------------
;  1. Sine code NEVER pokes P1 / P1SKIP / PCA0CPM / XBR / the FET SFRs directly.
;     Phase/FET state changes go ONLY through the stock comm1_comm2..comm6_comm1
;     routines and switch_power_off; duty goes ONLY through sine_set_duty (which
;     uses the Set_Power/Set_Damp_Pwm_Reg macros). [Inv 1]
;  2. The comm routines are delta-based (they assume the prior sector). They are
;     called in FORWARD ORDER ONLY (1->2->3->4->5->6->1), from a state seeded by
;     the stock init pair (comm5_comm6 + comm6_comm1 => sector 1). After ANY
;     switch_power_off (only at a direction flip here) the init pair is re-run
;     before stepping resumes. [Inv 2]
;     EXCEPTION (up-handoff, sine_cross_up): after cpl Flag_Motor_Dir_Rev reconciles
;     the drive direction (sine-fwd == 6step-rev), the catch detects the coasting
;     rotor's state s, writes the PREDECESSOR sector s-1 into Sine_Sector, and a SINGLE
;     sine_step_sector call steps INTO s (comm(s-1)_comm(s)). From the fully-defined
;     ALL-OFF left by switch_power_off this is safe -- the comm macros are absolute
;     register overwrites (Set_Pwm_Phase_X rewrites all of P1SKIP, Set_Comparator_Phase_X
;     rewrites CMP_MX), so no prior LIVE sector is assumed; the synthesised predecessor
;     just selects which comm(k)_comm(k+1) energises. [Inv 2]
;     sine_run's own entry (:186-187) instead keeps the stock comm5_comm6+comm6_comm1
;     pair because it re-seeds sector 1 while still live. [Inv 2]
;  3. Duty is applied only via sine_set_duty, which mirrors t1_int's DEADTIME skew,
;     Pwm_Braking clamp and >=0 clamp byte-for-byte, so the damping (complementary)
;     reload can never cross the power reload. [Inv 3 -> Power.asm sine_set_duty]
;  4. Single duty writer: while Flag_Sine_Run is set, t1_int_set_pwm is gated so the
;     ISR does not touch the PCA reload registers; sine_set_duty writes them with
;     IE_EA off, so no torn 16-bit reload is ever observable. [Inv 4]
;  5. Amplitude is clamped to min(Amp_Max<=60, Pwm_Limit) every tick, so the
;     temperature governor (scheduler_run lowers Pwm_Limit) directly bounds it; if
;     Pwm_Limit ever drops below the hold amplitude the loop hard-exits. [Inv 5]
;  6. EVERY exit clears Flag_Sine_Run and lands in exit_run_mode (FETs off), and the
;     gate stays set until exit_run_mode clears it AFTER its own clr IE_EA, so the
;     single-writer gate never opens while interrupts are enabled. At entry the FETs
;     are de-energised (switch_power_off) and the hold duty is installed BEFORE sector
;     1 is re-energised, so sine mode's first sustained drive is hold_amp, not the
;     stale reload left by the stock init pair. What actually guarantees the ISR cannot
;     apply a high (throttle-based) duty in sine mode is the Flag_Sine_Run gate (Inv 4),
;     which is set before any re-energisation here — NOT interrupt ordering (the stock
;     init pair leaves IE_EA enabled when sine_run is entered). [Inv 6]
;
; Telemetry note: sine_run does not update Comm_Period4x (there is no BEMF period to
; measure), so DShot eRPM telemetry is STALE in sine mode. This is intentional and
; documented: seeding Comm_Period4x would corrupt other run-mode code that reads it.
; The host closes its position loop on the AS5600 encoder (`enc`), not on eRPM.
;
; ------------------------------------------------------------------------------
; S2 FET-SAFETY INVARIANTS (Pgm_Sine_Mode==2 / Flag_Sine_Micro; add to Inv 1-6)
; ------------------------------------------------------------------------------
;  1b. S2 is the ONE documented exception to Inv 1: it DOES poke P1SKIP / PCA0CPM2 /
;      XBR1 and the Com FET latches directly. It does so ONLY through the routines
;      below, ONLY under `clr IE_EA`, and it restores the exact stock XBR1/PCA0CPM2
;      on every exit (sine2_hw_exit at sine_run_exit). Modes 0/1 never reach any of
;      this (guarded by Flag_Sine_Micro), so their instruction path is unchanged.
;
;  7. TWO-PHASE min-clamp (flat-bottom / DPWM) SVPWM. At every instant the most-
;     negative phase is CLAMPED to the negative rail (its Com/low-side FET fully on,
;     exactly like a stock 6-step vector); the other two phases carry sinusoidal
;     high-side duty. The "pair" phase (lower-lettered => lower P1 pins) uses the
;     stock POWER+DAMP complementary pair (PCA modules 0/1, DEADTIME skew -> proven,
;     Inv 3). The "second" phase (higher pins) uses the free PCA module 2 (CEX2)
;     HIGH-SIDE ONLY, with its Com/low-side FET LATCHED OFF -> that leg freewheels
;     through the body diode and can NEVER shoot through, whatever the m2 duty.
;
;  8. Crossbar role-fixity (verified vs EFM8BB2 Reference Manual):
;       - RM 11.3.3: the priority decoder assigns each enabled resource to the
;         least-significant UN-skipped pin; PnSKIP pins are skipped. Fig 11.4 fixes
;         the PCA priority order CEX0 > CEX1 > CEX2.
;       - RM 11.4.2: XBR1 PCA0ME=3 routes CEX0,CEX1,CEX2 (we use 03h; stock 02h).
;       - RM 16.4.7 / 16.3.8: CEX2 has independent polarity (PCA0POL.2, left at the
;         stock 0 = same as CEX0) and center-align (PCA0CENT.2, already set at init).
;     With Layout A pin order Ap=0<Ac=1<Bp=2<Bc=3<Cp=4<Cc=5 and "pair = lower-lettered
;     modulated phase", CEX0/CEX1 always bind to the pair's Pwm/Com (the two lowest
;     un-skipped pins) and CEX2 to the second phase's Pwm, for ALL three segments and
;     both directions. So module roles are FIXED; PCA0POL is never rewritten. The
;     three P1SKIP masks are: clamp A->0E3h, clamp B->0ECh, clamp C->0F8h.
;
;  9. Remux ordering. The clamp phase changes only 3x/erev (at the centres of the
;     even comm sectors; sine2_calc_clamp), where the outgoing CEX2 duty is ~0. A
;     change calls sine2_apply_segment, which under `clr IE_EA` FIRST forces P1SKIP=
;     0FFh + all FETs off (a full de-energise, so there is no shoot-through window),
;     THEN latches the new clamp Com on and un-skips the pair+second pins. Duties are
;     written BEFORE the remux so the re-routed modules drive the correct reloads
;     immediately (no stale-reload gap). Transients are deadtime-protected on the
;     pair leg and low-side-off on the CEX2 leg -> shoot-through-free by construction.
;
; 10. Single-writer still holds: t1_int writes only modules 0/1 and is gated by
;     Flag_Sine_Run (Inv 4); module 2 is written ONLY by sine2_* here, under IE_EA
;     off. Amplitude/thermal governor (Inv 5) is unchanged (Sine_Amp scales the LUT).
;**** **** **** **** **** **** **** **** **** **** **** **** ****

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Entry: reached from motor_start_bidir_done on a COLD start, or DIRECTLY from
; motor_start_seam on a BEMF->sine down-handoff (Flag_Sine_Handoff set). On a cold start
; the stock init pair (comm5_comm6 + comm6_comm1) ran just before the branch, so
; interrupts are ENABLED and sector 1 is briefly energised at the stale PCA reload
; (bounded by startup_power_max, deadtime intact) — exactly as on the stock startup path;
; we take control, gate the ISR duty writer, de-energise, install the hold duty, and only
; THEN re-energise sector 1. On the down-handoff seam the FETs are ALREADY LIVE at the
; run1 topology and IE_EA is off (from the seam): Flag_Sine_Handoff makes us SKIP the
; switch_power_off below, keeping the rotor energised while we re-seed phase/speed and
; take over make-before-break. Never returns (exits via exit_run_mode).
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_run:
    clr  IE_EA                          ; the init pair left IE_EA enabled; take control
    setb Flag_Sine_Run                  ; [Inv 4] gate the ISR duty writer FIRST

    jb   Flag_Sine_Handoff, sine_run_energised   ; seam: skip the power-off, keep FETs live
    ; [Inv 6] stop driving the FETs at the stale reload before touching the duty regs.
    call switch_power_off
sine_run_energised:

    ; Initialise stepper state (Inc==0 => hold; direction = whatever motor_start selected).
    mov  Sine_Sector, #1
    mov  Sine_Frac_L, #0
    mov  Sine_Frac_H, #0
    mov  Sine_Rcp_L, #0
    mov  Sine_Rcp_H, #0
    ; BlueGill S3: a BEMF->sine down-handoff (Flag_Sine_Handoff, survived motor_start) seeds the
    ; rotor's TRUE electrical phase (Sine_Sector) AND its actual speed (Sine_Inc, from the live
    ; Comm_Period4x) so the field re-enters continuously instead of snapping to sector 1 at Inc=0.
    ; Normal starts (flag clear) begin at sector 1 / Inc=0.
    ; Also reset the S3 crossover counters and disarm any stale up-handoff for the fresh run.
    mov  Sine_Step_Ticks, #0
    mov  Sine_Cross_Cnt, #0
    clr  Flag_Cross_Up_Armed
    jnb  Flag_Sine_Handoff, sine_run_inc_zero
    ; NOTE: do NOT clr Flag_Sine_Handoff here -- the S1/S2 energise steps below still branch
    ; on it. It is consumed at the single point sine_run_enter_done (after the energise).
    ; [S3 direction fix] motor_start re-derived Flag_Motor_Dir_Rev from Flag_Pgm_Dir_Rev
    ; (Bluejay.asm:912-918), NOT from sine's running direction. Sine's field convention is
    ; Flag_Motor_Dir_Rev == Flag_Rcp_Dir_Rev, so re-assert it here (IE_EA is off since :134 =>
    ; atomic) so sine resumes FORWARD after the down-handoff (fixes the decay-to-zero stall).
    ; Only this handoff path re-asserts it; the normal-start sine_run_inc_zero path is unchanged.
    mov  C, Flag_Rcp_Dir_Rev
    mov  Flag_Motor_Dir_Rev, C
    ; [S3 down-seed / phase] Seed the TRUE electrical phase. The 6-step state at the down-handoff
    ; is deterministic (always program-state 1), which maps to sine sector SINE_DN_SEED_SECTOR for
    ; both directions (PLAN A.1). Seed the PREDECESSOR sector; the energise step below steps forward
    ; into SINE_DN_SEED_SECTOR. Sine_Frac stays 0 (rotor just commutated into the state = exact
    ; sector boundary, not an approximation).
    mov  Sine_Sector, #(SINE_DN_SEED_SECTOR - 1)
    ; [S3 down-seed / speed] Seed Sine_Inc DYNAMICALLY from the live slow-side period. Comm_Period4x
    ; survives motor_start (only Flags0/1, demag, PWM limits, clock are touched), so it still holds
    ; the debounced BEMF period; deriving the field rate from it matches the rotor's ACTUAL speed
    ; (the static threshold seed would run slightly fast). div_2048000: A=divisor, returns
    ; Temp3/Temp4, clobbers Temp2-8; nothing live crosses this call. Divisor is guaranteed nonzero.
    mov  A, Comm_Period4x_H
    call div_2048000
    mov  Sine_Inc_L, Temp3
    mov  Sine_Inc_H, Temp4
    sjmp sine_run_inc_done
sine_run_inc_zero:
    mov  Sine_Inc_L, #0
    mov  Sine_Inc_H, #0
sine_run_inc_done:

    ; [Inv 5/3] install the hold duty into the PCA reload BEFORE re-energising, so the
    ; first energisation below is at hold_amp, not the stale reload.
    ; [S3 down-seed ORDER] This MUST stay AFTER the Sine_Inc seed above: on a down-handoff the
    ; entry amplitude scales with the seeded handoff speed (V/f), so a refactor must NOT reorder it.
    call sine_update_amp                ; sets Sine_Amp; ignore thermal C at entry (Pwm_Limit full)

    ; [Inv 1b/7] Branch on the S2 micro-stepping flag. S2 does NOT use the stock comm
    ; init pair; it routes the free PCA module 2 (CEX2), then sine2_set_duty applies the
    ; sector-1 segment (P1SKIP + clamp Com) and both modulated duties — that is S2's
    ; first energisation, at hold amplitude. S1 keeps its exact existing sequence.
    jb   Flag_Sine_Micro, sine_run_enter_s2
    call sine_set_duty                  ; [Inv 3/4] amplitude-only; independent of Sine_Sector

    ; [Inv 2] first energisation for the current direction (delta-based macros need the
    ; init pair after switch_power_off). This is the first energisation, now at hold_amp.
    jnb  Flag_Sine_Handoff, sine_run_s1_stock
    ; [S3 down-seed] seeded Sine_Sector = SINE_DN_SEED_SECTOR-1; one forward step energises
    ; comm(SINE_DN_SEED_SECTOR-1)_comm(SINE_DN_SEED_SECTOR) -- an ABSOLUTE FET overwrite. On the
    ; down-handoff SEAM switch_power_off is SKIPPED, so this starts from the LIVE run1 topology
    ; (comm6_comm1), NOT all-off: a genuine MAKE-BEFORE-BREAK handover. Shoot-through safety rests on
    ; hardware deadtime + the disjoint-leg source swap (sink phase stays driven), not on a prior all-off
    ; (IE_EA off, Flag_Sine_Run set => t1_int cannot write a duty), leaving Sine_Sector = SINE_DN_SEED_SECTOR.
    call sine_step_sector
    sjmp sine_run_enter_done
sine_run_s1_stock:
    call comm5_comm6
    call comm6_comm1
    sjmp sine_run_enter_done
sine_run_enter_s2:
    call sine2_hw_enter                 ; XBR1=03h, enable module 2, Sine_Seg=0FFh (force remux)
    ; [S3 down-seed] on a down-handoff, step the seeded predecessor (SINE_DN_SEED_SECTOR-1) to
    ; SINE_DN_SEED_SECTOR so sine2_set_duty derives segment g=(Sine_Sector-1)*32+m at the true
    ; phase. Frac=0 => m=0. Normal starts keep Sine_Sector=1 (segment 0).
    jnb  Flag_Sine_Handoff, sine_run_enter_s2_duty
    inc  Sine_Sector
sine_run_enter_s2_duty:
    call sine2_set_duty                 ; apply the segment + write pair/second duties
sine_run_enter_done:
    clr  Flag_Sine_Handoff              ; single consume point (idempotent on normal starts)

    ; Snapshot the Timer2 pacing baseline, then make sure interrupts are enabled.
    call sine_read_timer2               ; Temp1:Temp2 = current Timer2 (16-bit)
    mov  Sine_T2_L, Temp1
    mov  Sine_T2_H, Temp2
    setb IE_EA

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Main loop: paced to SINE_TICK_T2 Timer2 ticks (~1 kHz); services telemetry and the
; scheduler (temperature governor) every iteration, exactly like the stock idle loops.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_run_loop:
    ; [Inv 6] Exit on signal loss: t2_int decrements Rcp_Timeout_Cntd; t1_int reloads
    ; it to 10 on every valid frame (that reload is AFTER the gated block, so it still
    ; runs in sine mode). Zero => no signal for ~ a few ms => stop.
    mov  A, Rcp_Timeout_Cntd
    jz   sine_run_exit

    ; NOTE: neutral STOP is handled in 6-step (normal_run_checks, magnitude-based) which fires before the
    ; down-handoff can enter sine, so a neutral never reaches this loop; adding a magnitude check HERE is
    ; both redundant and unsafe (Sine_Rcp is 0 at sine_run entry, so it would abort every startup ramp).

    ; --- pacing: has a full control tick elapsed on Timer2? ---
    call sine_read_timer2               ; Temp1:Temp2 = now
    clr  C
    mov  A, Temp1
    subb A, Sine_T2_L
    mov  Temp3, A                       ; delta lo (mod 65536; interval << Timer2 wrap)
    mov  A, Temp2
    subb A, Sine_T2_H
    mov  Temp4, A                       ; delta hi
    clr  C
    mov  A, Temp3
    subb A, #LOW(SINE_TICK_T2)
    mov  A, Temp4
    subb A, #HIGH(SINE_TICK_T2)
    jc   sine_run_service               ; not a tick yet -> just service background tasks

    ; advance the pacing baseline by exactly one tick (phase-accurate, low jitter)
    mov  A, Sine_T2_L
    add  A, #LOW(SINE_TICK_T2)
    mov  Sine_T2_L, A
    mov  A, Sine_T2_H
    addc A, #HIGH(SINE_TICK_T2)
    mov  Sine_T2_H, A

    call sine_tick                      ; one control tick; C set => thermal exit requested
    jc   sine_run_exit
    ; [S3] up-handoff: sine_cross_update arms Flag_Cross_Up_Armed and ret's up the chain, so we
    ; fire it HERE at the sine_run_loop baseline stack depth (no manual SP surgery -> refactor-safe).
    jnb  Flag_Cross_Up_Armed, sine_run_service
    ljmp sine_cross_up

sine_run_service:
    ; Telemetry + scheduler, mirroring wait_for_start (do not create a packet while one
    ; is pending; scheduler_run self-gates on Flag_16ms_Elapsed).
    jb   Flag_Telemetry_Pending, sine_run_loop
    ; BlueGill S3: VIRTUAL eRPM for sine. There is no BEMF period here, so seed Comm_Period4x from the
    ; FORCED field rate (Sine_Inc), making DShot eRPM telemetry CONTINUOUS 0->sine->6-step (the host
    ; loop is otherwise blind in sine). eRPM = Sine_Inc*10000/65536 and Comm_Period4x = 80e6/eRPM, so
    ; Comm_Period4x = 524288000/Sine_Inc = 2048000/(Sine_Inc/256) ~= 2048000/Sine_Inc_H -> reuse
    ; div_2048000 (the seed's 24/8 core; quotient clamps to 0xFFFF, so below ~174 mech eRPM floors).
    ; div clobbers Temp2-8 but sine_tick already reuses them each tick, and the governor only caps on a
    ; HIGH eRPM (small period) which sine never reaches -> safe to leave Comm_Period4x at the sine rate.
    mov  A, Sine_Inc_H
    jz   sine_tlm_stopped
    call div_2048000                    ; Temp3=lo, Temp4=hi = 2048000/Sine_Inc_H (clamped 0xFFFF)
    clr  IE_EA
    mov  Comm_Period4x_L, Temp3
    mov  Comm_Period4x_H, Temp4
    setb IE_EA
    sjmp sine_tlm_build
sine_tlm_stopped:
    clr  IE_EA
    mov  Comm_Period4x_L, #0FFh          ; ~stopped -> slowest reportable period
    mov  Comm_Period4x_H, #0FFh
    setb IE_EA
sine_tlm_build:
    call dshot_tlm_create_packet
    call scheduler_run
    sjmp sine_run_loop

sine_run_exit:
    ; [Inv 7] S2 restores the stock crossbar/PCA state (module 2 off, XBR1=02h) on EVERY
    ; exit — this is the single choke-point for all S2 exits (timeout, thermal, signal
    ; loss). exit_run_mode's switch_power_off then de-energises the FETs.
    jnb  Flag_Sine_Micro, sine_run_exit_common
    call sine2_hw_exit
sine_run_exit_common:
    ; [Inv 6] Do NOT clear Flag_Sine_Run here: that would open the single-writer gate
    ; while interrupts are still enabled (a t1_int in that window would write the PWM
    ; reload). exit_run_mode clears it AFTER its own clr IE_EA, so the gate only opens
    ; with interrupts already disabled. Hand off to the stock power-down path (FETs off).
    ljmp exit_run_mode

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_tick - one 1 kHz control step.
;   1. target step-rate = Sine_Rcp << 3 (magnitude); direction = Flag_Rcp_Dir_Rev
;   2. slew Sine_Inc toward the target; a direction change decelerates to ZERO first,
;      then flips (switch_power_off + re-seed the init pair) [Inv 2]
;   3. accumulate the angle; step ONE sector forward on 16-bit overflow [Inv 1/2]
;   4. V/f amplitude + duty [Inv 3/5]
; Out: C set => thermal governor pulled Pwm_Limit below the hold amplitude => exit.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_tick:
    ; --- 1. target step-rate magnitude = Sine_Rcp << SINE_RCP_SHIFT -> Temp1:Temp2 ---
    mov  Temp1, Sine_Rcp_L
    mov  Temp2, Sine_Rcp_H
    mov  B, #SINE_RCP_SHIFT
sine_tick_shl:
    clr  C
    mov  A, Temp1
    rlc  A
    mov  Temp1, A
    mov  A, Temp2
    rlc  A
    mov  Temp2, A
    djnz B, sine_tick_shl

    ; --- 2. direction handling: reverse only through zero ---
    jb   Flag_Rcp_Dir_Rev, sine_tick_want_rev
    ; want forward
    jb   Flag_Motor_Dir_Rev, sine_tick_decel   ; currently reverse -> decelerate to zero first
    sjmp sine_tick_slew
sine_tick_want_rev:
    jnb  Flag_Motor_Dir_Rev, sine_tick_decel   ; currently forward -> decelerate to zero first

sine_tick_slew:
    ; direction matches: slew Sine_Inc toward the target in Temp1:Temp2
    call sine_slew
    sjmp sine_tick_accumulate

sine_tick_decel:
    ; direction change pending: force target 0 and slew down; flip once stopped.
    mov  Temp1, #0
    mov  Temp2, #0
    call sine_slew
    ; still moving? (Sine_Inc != 0) -> wait for the next ticks to reach zero
    mov  A, Sine_Inc_L
    orl  A, Sine_Inc_H
    jnz  sine_tick_accumulate
    ; [Inv 2] stopped: power off, adopt the new direction, re-seed the init pair.
    clr  IE_EA
    call switch_power_off
    mov  C, Flag_Rcp_Dir_Rev
    mov  Flag_Motor_Dir_Rev, C
    jb   Flag_Sine_Micro, sine_tick_decel_s2
    call comm5_comm6                    ; re-seed sector 1 for the new direction
    call comm6_comm1                    ; (comm routines re-enable IE_EA internally)
    sjmp sine_tick_decel_done
sine_tick_decel_s2:
    ; [Inv 7] S2: switch_power_off already skipped all pins (P1SKIP=0FFh); force a remux
    ; on the next sine2_set_duty so sector 1 is re-established for the new direction.
    mov  Sine_Seg, #0FFh
    setb IE_EA                          ; keep IE parity with the comm-routine path
sine_tick_decel_done:
    mov  Sine_Sector, #1
    mov  Sine_Frac_L, #0                ; start the new direction from a clean fractional phase
    mov  Sine_Frac_H, #0

sine_tick_accumulate:
    ; --- 3. accumulate the angle; step one sector on 16-bit overflow ---
    mov  A, Sine_Sector
    mov  Temp7, A                       ; [S3] remember the sector to detect a step this tick
    mov  A, Sine_Frac_L
    add  A, Sine_Inc_L
    mov  Sine_Frac_L, A
    mov  A, Sine_Frac_H
    addc A, Sine_Inc_H
    mov  Sine_Frac_H, A
    jnc  sine_tick_cross               ; no overflow -> no step this tick (Temp7==Sine_Sector)
    jnb  Flag_Sine_Micro, sine_tick_step_s1
    ; [Inv 7] S2: advance the sector counter only (1..6 wrap); the FET topology for the
    ; new sector is (re)established by sine2_set_duty, not by the stock comm routines.
    mov  A, Sine_Sector
    inc  A
    cjne A, #7, sine_tick_step_store
    mov  A, #1
sine_tick_step_store:
    mov  Sine_Sector, A
    sjmp sine_tick_cross
sine_tick_step_s1:
    call sine_step_sector               ; overflow -> advance one sector (forward only)

sine_tick_cross:
    ; [S3] maintain the crossover counters (Temp7 = pre-step sector) and, at a sector-1
    ; boundary once debounced and fast enough, hand off to BEMF 6-step (never returns).
    call sine_cross_update

sine_tick_amp:
    ; --- 4. V/f amplitude + duty ---
    call sine_update_amp                ; sets Sine_Amp; C set => thermal exit
    jc   sine_tick_thermal_exit
    jb   Flag_Sine_Micro, sine_tick_amp_s2
    call sine_set_duty                  ; [Inv 3/4] S1 single duty writer
    sjmp sine_tick_amp_done
sine_tick_amp_s2:
    call sine2_set_duty                 ; [Inv 7] S2 two-phase (pair m0/m1 + second m2) writer
sine_tick_amp_done:
    clr  C                              ; no exit
    ret
sine_tick_thermal_exit:
    setb C                              ; propagate thermal exit request
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_slew - move the 16-bit Sine_Inc toward the target in Temp1:Temp2 by at most
; Pgm_Sine_Ramp per call, clamping so it never overshoots the target.
; In:  Temp1:Temp2 = target magnitude. Clobbers A, Temp1..Temp6.
; (Only Temp1/Temp2 = R0/R1 can address indirectly, so the target is moved to
;  Temp3:Temp4 first to free Temp1 for the @Pgm_Sine_Ramp read.)
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_slew:
    mov  A, Temp1
    mov  Temp3, A                       ; Temp3 = target lo
    mov  A, Temp2
    mov  Temp4, A                       ; Temp4 = target hi
    mov  Temp1, #Pgm_Sine_Ramp
    mov  A, @Temp1
    mov  Temp5, A                       ; Temp5 = ramp step
    ; compare current Sine_Inc against target: C set => Sine_Inc < target
    clr  C
    mov  A, Sine_Inc_L
    subb A, Temp3
    mov  A, Sine_Inc_H
    subb A, Temp4
    jc   sine_slew_up

    ; ramp DOWN toward target (candidate in Temp1:Temp2)
    clr  C
    mov  A, Sine_Inc_L
    subb A, Temp5
    mov  Temp1, A
    mov  A, Sine_Inc_H
    subb A, #0
    mov  Temp2, A
    jc   sine_slew_clamp                ; underflowed past 0 -> snap to target
    clr  C
    mov  A, Temp1                       ; (Inc - ramp) < target ?
    subb A, Temp3
    mov  A, Temp2
    subb A, Temp4
    jnc  sine_slew_store                ; >= target -> use candidate
    sjmp sine_slew_clamp

sine_slew_up:
    ; ramp UP toward target (candidate in Temp1:Temp2)
    mov  A, Sine_Inc_L
    add  A, Temp5
    mov  Temp1, A
    mov  A, Sine_Inc_H
    addc A, #0
    mov  Temp2, A
    clr  C
    mov  A, Temp3                       ; target < (Inc + ramp) ?
    subb A, Temp1
    mov  A, Temp4
    subb A, Temp2
    jnc  sine_slew_store                ; target >= Inc+ramp -> use candidate
sine_slew_clamp:
    mov  A, Temp3                       ; overshoot -> snap to target
    mov  Temp1, A
    mov  A, Temp4
    mov  Temp2, A
sine_slew_store:
    mov  Sine_Inc_L, Temp1
    mov  Sine_Inc_H, Temp2
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_step_sector - advance exactly one commutation sector in FORWARD ORDER ONLY.
; [Inv 1/2] the only FET-state change on the stepping path.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_step_sector:
    mov  A, Sine_Sector
    dec  A
    jz   sine_step_1
    dec  A
    jz   sine_step_2
    dec  A
    jz   sine_step_3
    dec  A
    jz   sine_step_4
    dec  A
    jz   sine_step_5
    call comm6_comm1                    ; sector 6 -> 1
    mov  Sine_Sector, #1
    ret
sine_step_1:
    call comm1_comm2
    mov  Sine_Sector, #2
    ret
sine_step_2:
    call comm2_comm3
    mov  Sine_Sector, #3
    ret
sine_step_3:
    call comm3_comm4
    mov  Sine_Sector, #4
    ret
sine_step_4:
    call comm4_comm5
    mov  Sine_Sector, #5
    ret
sine_step_5:
    call comm5_comm6
    mov  Sine_Sector, #6
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_cross_update - BlueGill S3: per-tick forced-sine -> BEMF up-handoff bookkeeping.
; Called once per control tick from sine_tick_cross. In: Temp7 = the sector BEFORE this
; tick's step (so Sine_Sector != Temp7 <=> a sector step happened this tick).
;
; Two counters (both gated on Cross_Up != 0, so modes 0/1/2 and crossover-off never touch
; any S3 state):
;  * Sine_Cross_Cnt - debounce. Inc-saturates each tick the field rate is at/above the up
;    threshold (Sine_Inc_H >= Cross_Up) AND the commanded direction matches the spinning
;    direction; resets on any direction change pending or below-threshold tick.
;  * Sine_Step_Ticks - control ticks in the current 4-sector window. Increments every tick,
;    resets to 0 on the tick that steps INTO sector 3. At the tick that steps INTO sector 1
;    it equals ticks_4sector (sectors 3+4+5+6) => the Comm_Period4x seed base. A 4-sector
;    window (vs a 2-sector one) halves the one-shot seed quantization for the same eRPM.
;
; When a step lands on sector 1 with Cnt >= SINE_CROSS_DEBOUNCE and the window is in
; [SINE_CROSS_TICKS_MIN, SINE_CROSS_TICKS_MAX] ticks (fast enough that the seed is <=~11%
; coarse and BEMF is reliable, slow enough that ticks*2000 stays 16-bit and is above the BEMF
; floor), ARM Flag_Cross_Up_Armed and ret; sine_run_loop fires sine_cross_up at its baseline
; stack depth. Clobbers A,C,Temp1,Temp6.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_cross_update:
    mov  Temp1, #Pgm_Sine_Cross_Up
    mov  A, @Temp1
    jnz  sine_cross_enabled
    ret                                 ; Cross_Up == 0 -> crossover off, no S3 state touched
sine_cross_enabled:
    mov  Temp6, A                       ; Temp6 = Cross_Up

    ; --- debounce counter ---
    mov  C, Flag_Rcp_Dir_Rev            ; commanded direction
    jb   Flag_Motor_Dir_Rev, sine_cru_cur_rev
    jc   sine_cru_cnt_reset             ; want reverse, spinning forward -> dir change pending
    sjmp sine_cru_speed
sine_cru_cur_rev:
    jnc  sine_cru_cnt_reset             ; want forward, spinning reverse -> dir change pending
sine_cru_speed:
    clr  C
    mov  A, Sine_Inc_H
    subb A, Temp6                       ; Sine_Inc_H - Cross_Up
    jc   sine_cru_cnt_reset             ; below the up threshold -> reset
    mov  A, Sine_Cross_Cnt              ; matched + fast enough -> inc-saturate
    inc  A
    jnz  sine_cru_cnt_store
    dec  A                              ; saturate at 0xFF
sine_cru_cnt_store:
    mov  Sine_Cross_Cnt, A
    sjmp sine_cru_ticks
sine_cru_cnt_reset:
    mov  Sine_Cross_Cnt, #0

sine_cru_ticks:
    ; --- 4-sector window tick counter (increment first, then handle sector entry) ---
    mov  A, Sine_Step_Ticks
    inc  A
    jnz  sine_cru_ticks_store
    dec  A                              ; saturate at 0xFF
sine_cru_ticks_store:
    mov  Sine_Step_Ticks, A
    ; a step this tick? (Sine_Sector != Temp7)
    clr  C
    mov  A, Sine_Sector
    subb A, Temp7
    jz   sine_cru_ret                   ; no step this tick
    mov  A, Sine_Sector
    cjne A, #3, sine_cru_not3
    mov  Sine_Step_Ticks, #0            ; entered sector 3 -> begin a fresh 4-sector window
    ret
sine_cru_not3:
    cjne A, #1, sine_cru_ret            ; only a sector-1 entry can trigger the handoff
    ; --- up-handoff gate: debounced AND window within [MIN,MAX] ticks ---
    clr  C
    mov  A, Sine_Cross_Cnt
    subb A, #SINE_CROSS_DEBOUNCE
    jc   sine_cru_ret                   ; Cnt < debounce -> keep waiting
    clr  C
    mov  A, Sine_Step_Ticks
    subb A, #SINE_CROSS_TICKS_MIN
    jc   sine_cru_ret                   ; window < MIN -> too fast (seed too coarse) -> refuse
    clr  C
    mov  A, Sine_Step_Ticks
    subb A, #(SINE_CROSS_TICKS_MAX + 1)
    jnc  sine_cru_ret                   ; window > MAX -> below BEMF floor -> keep waiting
    setb Flag_Cross_Up_Armed            ; ARM: sine_run_loop fires sine_cross_up at baseline depth
sine_cru_ret:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_cross_up - BlueGill S3: hand a spinning rotor from forced sine to the stock BEMF
; 6-step run loop. Entered by ljmp from sine_run_loop (NOT from deep in sine_tick): the
; up-handoff condition is detected in sine_cross_update, which only ARMS Flag_Cross_Up_Armed
; and ret's; sine_run_loop then fires this at its own baseline stack depth. So there is NO
; manual SP surgery here and NO coupling to the sine_tick/sine_cross_update call depth (a
; future refactor of that chain cannot silently corrupt the stack) -> run1 runs at exactly the
; depth the stock startup path uses.
;
; Comm_Period4x seed (safety-critical): calc_next_comm_period timestamps run in halved-TMR2
; ticks (BB2 divides by 2). One commutation = one 60deg electrical sector, and Comm_Period4x is
; the "4x" period = 4 commutations = 4 SECTORS, in halved-Timer2 counts. Sine_Step_Ticks is a
; whole-tick counter that spans sectors 3..6 (reset entering sector 3, read entering sector 1;
; ground truth :546-550) = a 4-SECTOR window, so it directly measures the 4-sector Comm_Period4x
; span. Each control tick is SINE_TICK_T2 = 4000 raw Timer2 counts (Bluejay.asm:201-202), which
; is 2000 halved-Timer2 counts, so seed = Sine_Step_Ticks * SINE_TICK_T2/2 = ticks * 2000 (the
; mul #0D0h then #07h below). That is exactly 80e6/eRPM nominal (the vendor eRPM<->period
; mapping), so the run loop resumes at the true rotor speed. Prev_Comm = (TMR2_now>>1) -
; Comm_Period4x/4 back-dates the previous commutation exactly one commutation (= one sector),
; so the single priming calc_next_comm_period's average (4T - 4T/8 + T/2 = 4T, with T =
; this_period = seed/4) leaves the seed unchanged and arms the Timer3 wait chain that run1 consumes.
;
; Timer3 hazard (CONTEXT #9): sine mode skips initialize_timing, so BOTH the Timer3 COUNT and the
; Timer3 auto-RELOAD (TMR3RL) are stale at handoff, and TWO Timer3 wraps happen before the first
; ZC scan opens -- each must be bounded:
;   wrap#1 (the priming wait_advance_timing's Wait_For_Timer3, blocks WHILE Flag_Timer3_Pending):
;     bounded by the PRE-ARM below -- TMR3 count = -(Comm_Period4x/8) => a ~15deg-el window.
;   wrap#2 (the HW auto-reload TMR3:=TMR3RL fires at wrap#1): on the FIRST handoff per power cycle
;     TMR3RL=0x0000 => 65536 counts = ~16 ms of energised clamp on the catch vector, which destroys
;     the caught alignment before the first ZC scan (measured 601<->6620 eRPM limit cycle). We seed
;     TMR3RL := -16 at the pre-arm (below, next to the count seed) to bound wrap#2 to a few us; from
;     the 2nd serviced wrap on, t3_int keeps reloading TMR3RL:=-6, so this only needs closing once.
; After the pre-arm wrap the stock chain (wait_before_zc_scan -> setup_zc_scan_timeout) re-arms
; TMR3 = -(Comm_Period4x/2) for the real ZC-timeout window. The wrap#2-vs-`setb Flag_Timer3_Pending`
; race is benign in BOTH orders: one consumer per wrap, and an early wrap merely clears the pending
; flag so the priming wait falls through harmlessly. We deliberately do NOT call the stock final
; initialize_timing (it would reset Comm_Period4x to 0x00F0 and wipe the seed).
;
; IE note: both sine2_hw_exit AND sine_read_timer2 (below) re-enable IE_EA internally. That is
; safe because Flag_Sine_Run (the real single-writer gate) stays SET until the explicit release
; near the end, so t1_int cannot write a throttle-based duty in any of those windows.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_cross_up:
    clr  IE_EA                          ; [Inv 4/6] hold the single-writer gate; take control
    call switch_power_off               ; de-energise before reconfiguring the drive regime
    ; [Inv 7] S2 restores the stock crossbar/PCA (module 2 off, XBR1=02h). It re-enables IE
    ; internally, so re-clear afterwards to keep the gate closed until the release below.
    jnb  Flag_Sine_Micro, sine_cross_up_hw_done
    call sine2_hw_exit
    clr  IE_EA
sine_cross_up_hw_done:
    ; [S3 direction fix] BlueGill's forced-sine field convention is the OPPOSITE physical sense
    ; to stock 6-step comm for the same Flag_Motor_Dir_Rev (bench: sine +cmd -> +enc, but
    ; 6-step +cmd -> -enc). Both the catch detect/lookup (sine_catch_lookup swaps A<->C under
    ; this flag) AND the energise (sine_step_sector's comm*) branch on Flag_Motor_Dir_Rev, so
    ; flip it FIRST -- before detect -- so the catch reads/looks-up/energises the sector that
    ; continues the rotor's ACTUAL forward spin. Bench: direction fix alone rode the up-handoff
    ; forward (229->678 RPM) but high-speed 6-step reversed on ramp-down (marginal blind-seed
    ; sync); the catch supplies the correct ENTRY SECTOR so direction + sector are both aligned.
    cpl  Flag_Motor_Dir_Rev

    ; [S3 catch] The rotor is COASTING here: switch_power_off (:601) left every FET off, so we can
    ; safely READ its position before touching the drive. sine_catch_detect reads the 3-phase BEMF
    ; sign pattern via the comparator and looks up the FORWARD run state to enter (using the now
    ; flipped Flag_Motor_Dir_Rev), so closed-loop resumes at the sector the rotor is actually in
    ; (forward torque, correct next zero-cross) instead of blindly energising sector 1. It returns
    ; A = state s (1..6), or A = 0 if no stable+valid pattern was seen within the bounded retry
    ; budget. IE_EA is OFF and NO FET is energised across the whole detect (deliberate: an ISR
    ; could re-point CMP_MX mid-read). We MUST detect here, BEFORE the Timer3 pre-arm below, so the
    ; pre-arm still immediately precedes calc_next_comm_period (a ~2 ms detect between them would
    ; let the bounded wrap go stale).
    call sine_catch_detect
    jnz  sine_cross_up_detected
    ljmp motor_start                    ; [S3 catch] no valid pattern -> flat-stack re-grab.
                                        ; Reached with the sine_catch_detect call already
                                        ; RETURNED (SP back at sine_cross_up's baseline), so
                                        ; this ljmp has the same stack depth as the down-
                                        ; handoff's. Flag_Sine_Mode is still set, so motor_start
                                        ; re-enters sine_run, which re-arms the crossover for
                                        ; another attempt. NOTE: motor_start re-derives
                                        ; Flag_Motor_Dir_Rev, and sine_run's down-un-flip only
                                        ; runs under Flag_Sine_Handoff (not set here), so the
                                        ; fresh forced-field start uses sine's own convention.
                                        ; We do NOT set Flag_Sine_Handoff or write Sine_Inc_Seed_*.
sine_cross_up_detected:
    ; A = detected state s (1..6). Store its PREDECESSOR sector s-1 (wrap 1->6) into Sine_Sector
    ; now, so the energise step below is a bare sine_step_sector call (it steps predecessor -> s).
    ; Sine_Sector is a dedicated byte the seed math + Timer3 pre-arm below do NOT touch (they use
    ; only A/B/Temp*/Comm_Period4x/Prev_Comm), so the predecessor survives to the energise step.
    dec  A                              ; s-1
    jnz  sine_cross_up_no_wrap
    mov  A, #6                          ; s==1 -> predecessor sector 6
sine_cross_up_no_wrap:
    mov  Sine_Sector, A                 ; predecessor sector (s-1, wrap)

    ; --- Comm_Period4x = Sine_Step_Ticks * 2000  (0x07D0; ticks <= 30 => <= 60000, 16-bit) ---
    mov  A, Sine_Step_Ticks
    mov  B, #0D0h
    mul  AB                             ; B:A = ticks * 0xD0
    mov  Comm_Period4x_L, A
    mov  Temp5, B                       ; high byte of ticks*0xD0
    mov  A, Sine_Step_Ticks
    mov  B, #07h
    mul  AB                             ; A = ticks * 0x07 (B = 0 since ticks*7 <= 210)
    add  A, Temp5
    mov  Comm_Period4x_H, A

    ; --- Prev_Comm = (TMR2_now >> 1) - Comm_Period4x/4  (halved-TMR2 units) ---
    call sine_read_timer2               ; Temp1 = TMR2L, Temp2 = TMR2H
    clr  C
    mov  A, Temp2
    rrc  A
    mov  Temp2, A
    mov  A, Temp1
    rrc  A
    mov  Temp1, A                       ; Temp2:Temp1 = TMR2_now >> 1
    mov  A, Comm_Period4x_H             ; Temp4:Temp3 = Comm_Period4x >> 2
    clr  C
    rrc  A
    mov  Temp4, A
    mov  A, Comm_Period4x_L
    rrc  A
    mov  Temp3, A
    clr  C
    mov  A, Temp4
    rrc  A
    mov  Temp4, A
    mov  A, Temp3
    rrc  A
    mov  Temp3, A                       ; Temp4:Temp3 = Comm_Period4x / 4
    clr  C
    mov  A, Temp1
    subb A, Temp3
    mov  Prev_Comm_L, A
    mov  A, Temp2
    subb A, Temp4
    mov  Prev_Comm_H, A

    ; --- Bounded, seed-derived Timer3 pre-arm (first advance wait ~15deg el) ------------
    ; The priming calc_next_comm_period below falls through to wait_advance_timing, whose first
    ; Wait_For_Timer3 consumes Timer3 wrap#1. Sine mode skips initialize_timing, so both the TMR3
    ; COUNT and its auto-RELOAD (TMR3RL) are stale -> two wraps must be bounded (see the header
    ; hazard note): wrap#1 by the COUNT pre-arm here; wrap#2 by the TMR3RL seed just below.
    ; wrap#1: pre-arm TMR3 straight from the live seed so it is a bounded ~15deg-el window:
    ; Temp4:Temp3 = Comm_Period4x/4 (from the Prev_Comm block above); one more rrc pair ->
    ; Comm_Period4x/8, which IS the 15deg-el TMR3 magnitude. (15deg el = Comm_Period4x/16 in
    ; halved-TMR2 counts; Timer3 counts at 2x that rate per calc_new_wait_times' BB2 x2, so the
    ; count = 2*Comm_Period4x/16 = Comm_Period4x/8 -- the /4->15deg and x2->Timer3 factors cancel
    ; to a single right shift of the in-register /4.) The negate-into-TMR3 SFR sequence + arm is
    ; the stock setup_zc_scan_timeout startup tail, so [flash trim a] we CALL it
    ; (setup_zc_scan_timeout_startup_done) rather than duplicate it, re-clearing IE_EA after (its
    ; tail does setb IE_EA). Net IE_EA stays effectively OFF through the warm-start writes below
    ; until the explicit gate release; setb Flag_Timer3_Pending (done inside that routine) makes
    ; the priming wait_advance_timing consume wrap#1 (one consumer per wrap; if it ever wraps
    ; early, t3_int just clears the pending flag and the priming wait falls through harmlessly,
    ; then wait_before_zc_scan re-arms the seed-derived Comm_Period4x/2 zc timeout via
    ; setup_zc_scan_timeout).
    clr  C
    mov  A, Temp4
    rrc  A
    mov  Temp2, A
    mov  A, Temp3
    rrc  A
    mov  Temp1, A                       ; Temp2:Temp1 = Comm_Period4x/8 = 15deg-el TMR3 magnitude
    ; [flash trim a] The negate-into-TMR3L/H + arm sequence is byte-identical to the vendor
    ; setup_zc_scan_timeout's startup tail, so call it instead of duplicating it (Temp1:Temp2
    ; is its magnitude input; it also does setb Flag_Timer3_Pending + orl EIE1,#80h, both of
    ; which we want). Its final `setb IE_EA` re-enables interrupts, so we re-clear IE_EA right
    ; after to keep the single-writer gate closed through the warm-start writes until the
    ; explicit release below (the momentary IE-on window inside the routine is harmless: FETs
    ; are all-off and Flag_Sine_Run is still set, so no ISR can drive a FET or write a duty).
    call setup_zc_scan_timeout_startup_done
    clr  IE_EA

    mov  TMR3RLL, #0F0h                  ; [S3 lock fix] seed Timer3 auto-reload := -16 (mirrors
    mov  TMR3RLH, #0FFh                  ; t3_int's #0FAh/#0FFh = -6). setup_zc_scan_timeout_startup_done
                                         ; writes only TMR3L/H; the HW auto-reloads TMR3:=TMR3RL at
                                         ; wrap#1, and on the FIRST handoff per power cycle TMR3RL=0
                                         ; => a ~16 ms energised clamp on the catch vector that breaks
                                         ; the first ZC lock. Bound wrap#2 to a few us. Direct-to-SFR
                                         ; immediate loads: clobber no A/B/Temp*/PSW/Sine_Sector, so
                                         ; the predecessor s-1 in Sine_Sector survives to the energise.

    ; --- run1 warm-start conditions (mirror motor_start_no_sine's run1 preconditions) ---
    setb Flag_Initial_Run_Phase
    mov  Startup_Cnt, #1                ; [root-cause fix] Startup_Cnt!=0 => wait_for_comp_out_*
                                        ; ACCEPTS the comparator from the first zc scan
                                        ; (Timing.asm:661-662 discards good reads while 0).
                                        ; Sine never runs Flag_Startup_Phase, so this is the
                                        ; only writer; it is side-effect-free (zero/nonzero
                                        ; only) => closed-loop BEMF from commutation #1.
    clr  Flag_Startup_Phase
    mov  Initial_Run_Rot_Cntd, #12
    clr  Flag_High_Rpm
    mov  Pwm_Limit, Pwm_Limit_Beg       ; conservative startup limit before releasing the gate
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_Beg

    ; [Inv 2 / S3 catch] Energise the DETECTED state s from the fully-defined ALL-OFF state left by
    ; switch_power_off (:601). Sine_Sector already holds the predecessor s-1 (set at detect success
    ; above), so sine_step_sector steps INTO s, calling comm(s-1)_comm(s) -- an ABSOLUTE register
    ; overwrite (Set_Pwm_Phase_X rewrites all of P1SKIP, Set_Comparator_Phase_X rewrites CMP_MX,
    ; *_Fet_On/Off drive their own pins) that fully establishes state s's conduction AND comparator
    ; phase for the current (flipped) direction (comm*_comm* branch on Flag_Motor_Dir_Rev).
    ; sine_step_sector's comm* re-enables IE_EA, but Flag_Sine_Run is still set so t1_int cannot
    ; write a duty here.
    call sine_step_sector               ; energise state s; leaves Sine_Sector = s

    ; [Inv 4/6] release the single-writer gate ONLY under IE off, after the conservative limit.
    clr  IE_EA
    clr  Flag_Sine_Run
    setb IE_EA

    call calc_next_comm_period          ; prime timing + arm Timer3 wait chain (see hazard note)

    ; [S3 catch] Enter the stock BEMF run loop at the DETECTED state's run_s. sine_step_sector
    ; left Sine_Sector = s and calc_next_comm_period does not touch it. [flash trim b] jmp
    ; @A+DPTR through a table of ljmps (compact + constant-time vs a cjne chain).
    mov  A, Sine_Sector                 ; s (1..6)
    dec  A
    mov  B, #3
    mul  AB                             ; (s-1) * 3  (each dispatch entry is one 3-byte ljmp)
    mov  DPTR, #sine_cross_up_dispatch
    jmp  @A+DPTR
sine_cross_up_dispatch:
    ljmp run1
    ljmp run2
    ljmp run3
    ljmp run4
    ljmp run5
    ljmp run6

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_catch_detect - BlueGill S3 "catch a spinning rotor": determine which 6-step run
; state a COASTING rotor is in, so the up-handoff resumes closed-loop with FORWARD torque
; at the correct next zero-cross instead of blindly energising sector 1.
;
; Method: with all FETs off, the open phases carry the rotor's back-EMF. Read each phase
; against the neutral node through the comparator (Set_Comparator_Phase_X selects the mux;
; Read_Comparator_Output bit 0x40 set = phase > neutral = BEMF POSITIVE, COMPARATOR_INVERT=0)
; to build a 3-bit pattern (bit2=A, bit1=B, bit0=C). Over a full electrical revolution the
; three signs give 6 valid codes, one per 60deg sector -> NO 180deg ambiguity. Look up the
; run whose next zero-cross fires next in that sector.
;
; ---- THE TRUTH TABLE (load-bearing; a wrong entry reverse-locks the rotor on live FETs) --
; Derived from the phase-BEMF model Ea=sin(t), Eb=sin(t-240), Ec=sin(t-120) (the ONLY
; convention whose zero-cross order matches the hardware: A^0 B_60 C^120 A_180 B^240 C_300),
; cross-checked against the stock float-phase (Commutation.asm) + run1..run6 comparator wait
; polarities (Bluejay.asm), and re-derived independently by tools/sim/catch_truth_table.py.
;   pattern ABC | idx | 60deg sector | next zero-cross      | enter
;      110       |  6  |   0- 60      | B falling  (run2)    | run2
;      100       |  4  |  60-120      | C rising   (run3)    | run3
;      101       |  5  | 120-180      | A falling  (run4)    | run4
;      001       |  1  | 180-240      | B rising   (run5)    | run5
;      011       |  3  | 240-300      | C falling  (run6)    | run6
;      010       |  2  | 300-360      | A rising   (run1)    | run1
;      000 / 111 | 0/7 | balanced 3-phase sum ~ 0 => INVALID -> retry, then fallback
; => Sine_Catch_Table[idx0..7] = 0,5,1,6,3,4,2,0.
; REVERSE (Flag_Motor_Dir_Rev): the _rev comm routines swap the comparator phase A<->C on
; runs 1/3/4/6 (runs 2/5 keep B) and the run EDGE polarity is NOT direction-swapped, which is
; exactly a bit A(2)<->C(0) swap of the READ pattern followed by the SAME forward table (see
; sine_catch_lookup). torque-angle: entering state s in its own 60deg window keeps the field-
; to-rotor angle in 60..120deg => >= 0.87 * peak torque, always forward; +-1-state boundary
; jitter is benign (self-correcting on the next zero-cross).
;
; Filtering, in two independent stages:
;  1. DEMAG rejection = a FIXED initial settle (below) that waits out the body-diode flyback so
;     BOTH reads land in the clean coasting regime. This is what stops a stale-FIELD-sector clamp
;     (stable + valid + WRONG) from being accepted -- the consecutive-equal check does NOT do this.
;  2. NOISE rejection = within the coasting regime, accept only when two CONSECUTIVE reads agree
;     AND the code is valid (not 000/111); a per-phase mux settle precedes each comparator sample.
; The retry loop accepts the EARLIEST stable+valid pair, so the demag settle (stage 1) MUST run
; first. NO FET is energised until a pattern is accepted; on exhaustion return 0 (caller falls back
; to motor_start). The coasting-BEMF sign validity is HW-only and bench-tuned (SINE_CATCH_*) after
; commit. Runs with IE_EA OFF throughout.
;
; Out: A = state s (1..6), or A = 0 on failure. Clobbers A, B, C, PSW_F0, Temp5, Temp6, Temp7.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
SINE_CATCH_RETRIES  EQU 64                      ; bounded retry budget (~2 ms of ~30 us reads)
SINE_CATCH_SETTLE   EQU (120 SHL IS_MCU_48MHZ)  ; comparator mux-settle djnz count (HW-tunable)
SINE_CATCH_DEMAG_SETTLE EQU 48                  ; demag pre-settle outer loops (~450 us; HW-tunable)

sine_catch_detect:
    ; --- FIXED demag pre-settle (FETs OFF, IE OFF): wait ~one body-diode flyback time BEFORE the
    ;     first read. Right after switch_power_off the two last-driven phases are clamped to the
    ;     rails by their freewheeling body diodes; a read taken inside that window returns the stale
    ;     FIELD sector -- which is STABLE and valid (non-0/7), so the consecutive-equal filter (which
    ;     only rejects NOISE, not a stable clamp) would ACCEPT it and reverse-lock. This settle puts
    ;     BOTH agreeing reads in the clean coasting regime. Nested djnz (not a Timer2 span) so IE
    ;     stays OFF throughout -- sine_read_timer2 would re-enable it -- and it is a few bytes rather
    ;     than the ~17 the Timer2-span costs (flash is tight; this avoids touching bench-validated S1).
    ;     Duration is a tunable HW EQU. ---
    mov  Temp6, #SINE_CATCH_DEMAG_SETTLE
sine_catch_demag_o:
    mov  Temp5, #(120 SHL IS_MCU_48MHZ)
sine_catch_demag_i:
    djnz Temp5, sine_catch_demag_i
    djnz Temp6, sine_catch_demag_o

    mov  Temp6, #SINE_CATCH_RETRIES     ; retry counter
    mov  Temp7, #0FFh                    ; seed "previous" with an impossible pattern (0..7 only),
                                         ; so the first loop iteration just latches the first real
                                         ; read as the new "previous" (saves a separate seed read)
sine_catch_loop:
    call sine_catch_read                ; A = B = current pattern (read leaves it in B too)
    xrl  A, Temp7                       ; 0 iff current == previous read
    mov  Temp7, B                       ; previous := current (for the next comparison)
    jnz  sine_catch_dec                 ; differ -> not stable yet, retry
    ; two consecutive reads agree. Validate + map (A<->C swap if reverse).
    mov  A, B                           ; restore the current pattern
    call sine_catch_lookup              ; in A = pattern; out A = state (0 if 000/111)
    jnz  sine_catch_done                ; valid state -> accept (A = s)
    ; stable but invalid (000/111): Temp7 == this pattern -> keep retrying
sine_catch_dec:
    djnz Temp6, sine_catch_loop
    clr  A                              ; retries exhausted -> signal failure (A = 0)
sine_catch_done:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_catch_read - sample the 3 phase-BEMF signs into one 3-bit pattern.
; Loops the comparator mux over phases A,B,C (mux hi-nibble steps by (B_Mux-A_Mux); the
; Set_Comparator_Phase_X macros expand to `mov CMP_MX,#((X_Mux SHL 4)+V_Mux)`, so this is
; the same writes, just table-driven to save flash). Read polarity: CMP_CN0 bit 0x40 set =
; phase > neutral (COMPARATOR_INVERT=0). Out: A = pattern (bit2=A,bit1=B,bit0=C; 1 = BEMF>0).
; Clobbers A, B (pattern accumulator), C, Temp3, Temp4, Temp5.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; [assemble-time guard] This loop builds CMP_MX as ((X_Mux SHL 4)+V_Mux) with the mux hi-nibble
; stepping by (B_Mux-A_Mux), so it needs: (1) the BB1/BB2 mux encoding (BB51 uses fixed #10h/
; #11h/#12h instead), (2) the phase muxes an arithmetic progression (A_Mux,B_Mux,C_Mux equally
; spaced), and (3) a non-inverted comparator (raw bit 0x40 = phase>0 -- the whole truth table
; depends on it). All hold on the BlueGill target (BB2, A_Mux=1,B_Mux=2,C_Mux=3, INVERT=0).
; Refuse to assemble otherwise (undefined symbol => AX51 #A45; the name is the diagnostic).
IF (MCU_TYPE == MCU_BB51) or ((B_Mux - A_Mux) != (C_Mux - B_Mux)) or (COMPARATOR_INVERT != 0)
    DB S3_CATCH_BUILD_ERROR__needs_BB1_BB2_progressive_mux_and_noninverted_comparator
ENDIF
sine_catch_read:
    clr  A
    mov  B, A                           ; pattern accumulator = 0
    mov  Temp3, #((A_Mux SHL 4) + V_Mux); CMP_MX for phase A (V_Mux = neutral on lo nibble)
    mov  Temp4, #3                       ; 3 phases: A, then B, then C (MSB-first)
sine_catch_read_lp:
    mov  CMP_MX, Temp3                   ; select this phase (= Set_Comparator_Phase_X)
    mov  Temp5, #SINE_CATCH_SETTLE       ; mux settle (comparator was just re-pointed)
sine_catch_read_settle:
    djnz Temp5, sine_catch_read_settle
    mov  A, CMP_CN0                      ; read comparator (no invert on this target)
    mov  C, ACC.6                        ; C = comparator output = phase > neutral
    mov  A, B
    rlc  A                               ; shift prior bits up, bring this sign into b0
    mov  B, A
    mov  A, Temp3
    add  A, #((B_Mux - A_Mux) SHL 4)     ; advance mux to the next phase
    mov  Temp3, A
    djnz Temp4, sine_catch_read_lp
    mov  A, B                            ; A = 00000 A B C
    ret

; sine_catch_lookup - map a 3-bit pattern to a run state via Sine_Catch_Table. On
; Flag_Motor_Dir_Rev, swap bits A(2)<->C(0) BEFORE the lookup (see the truth-table note) so
; reverse uses the SAME table. In: A = pattern (0..7). Out: A = state (1..6), 0 for 000/111.
sine_catch_lookup:
    jnb  Flag_Motor_Dir_Rev, sine_catch_lk_tbl
    mov  C, ACC.0                       ; save old C-bit
    mov  PSW_F0, C
    mov  C, ACC.2                       ; old A-bit
    mov  ACC.0, C                       ; bit0 := old A-bit
    mov  C, PSW_F0                      ; old C-bit
    mov  ACC.2, C                       ; bit2 := old C-bit  (A<->C swapped)
sine_catch_lk_tbl:
    mov  DPTR, #Sine_Catch_Table
    movc A, @A+DPTR
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Sine_Catch_Table - coasting-rotor BEMF sector pattern -> forward run state (see the
; derivation in sine_catch_detect's header). Indexed by pattern bit2=A/bit1=B/bit0=C;
; entries 0 (000) and 7 (111) are the invalid balanced codes. VERIFIED by
; tools/sim/catch_truth_table.py (asserts this exact byte sequence + the reverse swap rule).
; MUST stay byte-identical to that emitter.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
Sine_Catch_Table:
    DB 0, 5, 1, 6, 3, 4, 2, 0

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_update_amp - V/f amplitude: Sine_Amp = min(hold_amp + Inc_H, Amp_Max, Pwm_Limit).
; [Inv 5] the min() with Pwm_Limit is what makes the temperature governor bound the
; hold/drive current. Out: C set => Pwm_Limit < hold_amp (thermal) => caller must exit.
; Clobbers A, B, Temp1..Temp3.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_update_amp:
    mov  Temp1, #Pgm_Sine_Hold_Amp
    mov  A, @Temp1
    mov  Temp2, A                       ; Temp2 = hold_amp
    add  A, Sine_Inc_H                  ; V/f: raise volts with speed (Inc high byte)
    jnc  sine_update_amp_no_of
    mov  A, #0FFh                       ; 8-bit saturate
sine_update_amp_no_of:
    mov  Temp3, A                       ; Temp3 = candidate (hold + Inc_H)
    ; clamp to Amp_Max
    mov  Temp1, #Pgm_Sine_Amp_Max
    mov  A, @Temp1
    mov  B, A                           ; B = amp_max
    clr  C
    mov  A, Temp3
    subb A, B
    jc   sine_update_amp_lt_max         ; candidate < amp_max -> keep
    mov  Temp3, B                       ; else clamp to amp_max
sine_update_amp_lt_max:
    ; clamp to Pwm_Limit
    clr  C
    mov  A, Temp3
    subb A, Pwm_Limit
    jc   sine_update_amp_lt_lim         ; candidate < Pwm_Limit -> keep
    mov  Temp3, Pwm_Limit               ; else clamp to Pwm_Limit
sine_update_amp_lt_lim:
    mov  Sine_Amp, Temp3
    ; thermal test: C set if Pwm_Limit < hold_amp
    clr  C
    mov  A, Pwm_Limit
    subb A, Temp2
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; sine_read_timer2 - atomic 16-bit Timer2 read (mirrors calc_next_comm_period). The
; control interval (~4000 ticks) is far below a 16-bit Timer2 wrap, so the extended
; byte is not needed and modular subtraction of two reads is always the true delta.
; Out: Temp1 = TMR2L, Temp2 = TMR2H.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_read_timer2:
    clr  IE_EA
    clr  TMR2CN0_TR2                    ; Disable Timer2 while reading the pair
    mov  Temp1, TMR2L
    mov  Temp2, TMR2H
    setb TMR2CN0_TR2                    ; Re-enable Timer2
    setb IE_EA
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; ==============================================================================
; S2 ASSEMBLE-TIME HARDWARE GUARDS (PLAN §2 — fail loudly, never ship silently)
; ==============================================================================
; S2's three static P1SKIP masks (0E3h/0ECh/0F8h), the pair/second phase->pin
; mapping, the CEX2-low-side-off topology, and the DEADTIME complementary pair are
; ALL specific to Layout A pin order (Ap0<Ac1<Bp2<Bc3<Cp4<Cc5) AND a non-zero dead
; time. Building S2 for any other layout, or with DEADTIME==0, would silently apply
; these Layout-A masks to the WRONG physical FETs (esctool's layout check does NOT
; catch this). Refuse to assemble rather than emit a shoot-through-capable image.
; Mechanism: reference an undefined symbol (AX51 error #A45) so assembly aborts; the
; symbol name is the diagnostic. Guards are skipped (no error) on the valid target.
IF DEADTIME == 0
    DB S2_BUILD_ERROR__requires_DEADTIME_nonzero
ENDIF
IF (A_Pwm != 0) or (A_Com != 1) or (B_Pwm != 2) or (B_Com != 3) or (C_Pwm != 4) or (C_Com != 5)
    DB S2_BUILD_ERROR__requires_Layout_A_pin_order
ENDIF

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; ==============================================================================
; S2 min-clamp (flat-bottom / DPWM) TWO-PHASE micro-stepping (Pgm_Sine_Mode==2)
; ==============================================================================
; Everything below runs ONLY when Flag_Sine_Micro is set. See the "S2 FET-SAFETY
; INVARIANTS (Inv 7+)" block in the file header. Reference model + LUT cross-check:
; tools/sim/sine_drive_model.py (print_s2_dpwm_section); FET analysis:
; docs/sine-drive-design.md ("S2 as-built").
;**** **** **** **** **** **** **** **** **** **** **** **** ****

; ------------------------------------------------------------------------------
; sine2_hw_enter - route CEX2 to a pin and enable the free PCA module 2 so the
; SECOND modulated phase can be driven high-side-only. Writes a 0% module-2 reload
; FIRST (high-side off) so enabling/​routing never energises at a stale duty.
; ------------------------------------------------------------------------------
sine2_hw_enter:
    clr  A
    call sine2_write_second             ; module-2 auto-reload = 0% (high-side off)
    clr  IE_EA
    mov  PCA0CPM2, #42h                 ; enable comparator + PWM mode (mirrors POWER, DEADTIME!=0)
    mov  XBR1, #03h                     ; PCA0ME=3: route CEX0,CEX1,CEX2 (was 02h = CEX0/CEX1)
    setb IE_EA
    mov  Sine_Seg, #0FFh                ; no segment applied yet -> first duty write forces a remux
    ret

; ------------------------------------------------------------------------------
; sine2_hw_exit - restore the exact stock crossbar/PCA state (module 2 disabled,
; XBR1 back to CEX0/CEX1). Called on EVERY S2 exit (see sine_run_exit).
; ------------------------------------------------------------------------------
sine2_hw_exit:
    clr  IE_EA
    mov  XBR1, #02h                     ; restore stock routing
    mov  PCA0CPM2, #00h                 ; disable module 2
    setb IE_EA
    ret

; ------------------------------------------------------------------------------
; sine2_apply_segment - (re)establish the FET topology for a clamp phase.
; In: A = clamp phase (0=A, 1=B, 2=C).
; Fully de-energises first (P1SKIP=0FFh -> all pins GPIO, all FETs off: no
; shoot-through window), then latches the clamp phase's Com FET ON and un-skips
; the pair (POWER+DAMP, two lowest pins) + second (CEX2) Pwm pins. The crossbar
; then binds CEX0/CEX1 to the pair Pwm/Com and CEX2 to the second Pwm (RM 11.3.3
; priority decode). Called only when the clamp changes (3x/erev).
; ------------------------------------------------------------------------------
sine2_apply_segment:
    clr  IE_EA
    Set_All_Pwm_Phases_Off              ; P1SKIP=0FFh: all P1 pins become GPIO
    All_Pwm_Fets_Off                    ; every Pwm (high-side) FET off
    All_Com_Fets_Off                    ; every Com (low-side) FET off -> clean slate
    jnz  sine2_seg_not_a                ; A==0 -> clamp phase A
    A_Com_Fet_On                        ; clamp A to negative rail
    mov  P1SKIP, #0E3h                  ; un-skip B_Pwm/B_Com (pair) + C_Pwm (second)
    sjmp sine2_seg_done
sine2_seg_not_a:
    dec  A
    jnz  sine2_seg_c                    ; A==1 -> clamp phase B
    B_Com_Fet_On
    mov  P1SKIP, #0ECh                  ; un-skip A_Pwm/A_Com (pair) + C_Pwm (second)
    sjmp sine2_seg_done
sine2_seg_c:                            ; A==2 -> clamp phase C
    C_Com_Fet_On
    mov  P1SKIP, #0F8h                  ; un-skip A_Pwm/A_Com (pair) + B_Pwm (second)
sine2_seg_done:
    setb IE_EA
    ret

; ------------------------------------------------------------------------------
; sine2_calc_clamp - clamped (most-negative) phase from the electrical position.
; In: Sine_G = g_eff (0..191, 192 microsteps/erev). Out: A = clamp (0=A,1=B,2=C).
; Windows (min-clamp): g<48 -> B, 48<=g<112 -> C, 112<=g<176 -> A, g>=176 -> B.
; ------------------------------------------------------------------------------
sine2_calc_clamp:
    mov  A, Sine_G
    clr  C
    subb A, #48
    jc   sine2_clamp_is_b               ; g < 48
    mov  A, Sine_G
    clr  C
    subb A, #112
    jc   sine2_clamp_is_c               ; 48 <= g < 112
    mov  A, Sine_G
    clr  C
    subb A, #176
    jc   sine2_clamp_is_a               ; 112 <= g < 176
sine2_clamp_is_b:                       ; g >= 176 falls through here too
    mov  A, #1
    ret
sine2_clamp_is_c:
    mov  A, #2
    ret
sine2_clamp_is_a:
    clr  A
    ret

; ------------------------------------------------------------------------------
; sine2_circdist - circular distance (mod 192) between g_eff and a peak.
; In: Temp3 = peak (0..192), Sine_G = g_eff. Out: A = distance (0..96).
; Clobbers A, Temp4.
; ------------------------------------------------------------------------------
sine2_circdist:
    clr  C
    mov  A, Sine_G
    subb A, Temp3                       ; g - peak
    jnc  sine2_cd_abs
    cpl  A
    inc  A                              ; |g - peak| (two's-complement negate; |diff|<=192)
sine2_cd_abs:
    mov  Temp4, A                       ; |g - peak| (0..192)
    clr  C
    subb A, #97
    jc   sine2_cd_keep                  ; <= 96 -> already the short way
    mov  A, #192                        ; else wrap: 192 - |g-peak|
    clr  C
    subb A, Temp4
    ret
sine2_cd_keep:
    mov  A, Temp4
    ret

; ------------------------------------------------------------------------------
; sine2_idx - arc-LUT index for a modulated phase at the current position.
; In: A = phase (0/1/2), Sine_G = g_eff. Out: A = index (0..48) into Sine2_Arc_Lut.
; A phase peaks (duty=255) at the two sector boundaries flanking its saddle:
;   peak1 = phase*64 + 32,  peak2 = peak1 + 32.  index = 48 - min(dist to each).
; Clobbers A, B, Temp3..Temp5.
; ------------------------------------------------------------------------------
sine2_idx:
    swap A                              ; phase*16
    rl   A
    rl   A                              ; phase*64  (<=128)
    add  A, #32                         ; peak1
    mov  Temp3, A
    call sine2_circdist                 ; dist to peak1
    mov  Temp5, A
    mov  A, Temp3
    add  A, #32                         ; peak2 = peak1 + 32
    mov  Temp3, A
    call sine2_circdist                 ; dist to peak2 (in A)
    mov  Temp4, A
    clr  C
    subb A, Temp5                       ; dist2 - dist1
    jnc  sine2_idx_min                  ; dist2 >= dist1 -> min already in Temp5
    mov  A, Temp4                       ; else min = dist2
    mov  Temp5, A
sine2_idx_min:
    mov  A, #48
    clr  C
    subb A, Temp5                       ; index = 48 - min_dist
    ret

; ------------------------------------------------------------------------------
; sine2_lut_scale - duty for a modulated phase = (Sine2_Arc_Lut[idx] * Sine_Amp) >> 8.
; In: A = index (0..48). Out: A = 8-bit throttle-equivalent duty. Clobbers A, B.
; ------------------------------------------------------------------------------
sine2_lut_scale:
    mov  DPTR, #Sine2_Arc_Lut
    movc A, @A+DPTR                     ; L[idx] (0..255)
    mov  B, Sine_Amp
    mul  AB                             ; B:A = L * Sine_Amp
    mov  A, B                           ; >> 8 (high byte)
    ret

; ------------------------------------------------------------------------------
; sine2_set_duty - one S2 control step: compute the electrical position, the two
; modulated phase duties, write them (pair -> modules 0/1 via sine_set_duty,
; second -> module 2), and remux the FET topology if the clamp phase changed.
; Assumes Sine_Amp was set by sine_update_amp. Reverse: mirrors the sequence
; (g_eff = 192 - g), so the field rotates the other way (no extra tables).
; ------------------------------------------------------------------------------
sine2_set_duty:
    ; --- microstep m = Sine_Frac_H >> 3 (top 5 bits) ---
    mov  A, Sine_Frac_H
    clr  C
    rrc  A
    clr  C
    rrc  A
    clr  C
    rrc  A                              ; A = m (0..31)
    mov  Sine_D_Pair, A                 ; stash m
    ; --- g = (Sine_Sector-1)*32 + m ---
    mov  A, Sine_Sector
    dec  A
    swap A                              ; (sector-1)*16
    rl   A                              ; (sector-1)*32  (<=160)
    add  A, Sine_D_Pair                 ; g (0..191)
    ; --- reverse: g_eff = 192 - g (map 192 -> 0) ---
    jnb  Flag_Motor_Dir_Rev, sine2_g_store
    mov  Sine_D_Pair, A
    mov  A, #192
    clr  C
    subb A, Sine_D_Pair                 ; 192 - g  (1..192)
    cjne A, #192, sine2_g_store         ; 192 -> 0
    clr  A
sine2_g_store:
    mov  Sine_G, A

    ; --- clamp -> derive pair / second phases ---
    call sine2_calc_clamp               ; A = clamp phase
    jnz  sine2_sd_not_a
    mov  Sine_D_Pair, #1                ; clamp A: pair=B, second=C
    mov  Sine_D_Second, #2
    sjmp sine2_sd_ps
sine2_sd_not_a:
    dec  A
    jnz  sine2_sd_c
    mov  Sine_D_Pair, #0                ; clamp B: pair=A, second=C
    mov  Sine_D_Second, #2
    sjmp sine2_sd_ps
sine2_sd_c:
    mov  Sine_D_Pair, #0                ; clamp C: pair=A, second=B
    mov  Sine_D_Second, #1
sine2_sd_ps:
    ; --- duty8 for the pair phase ---
    mov  A, Sine_D_Pair
    call sine2_idx
    call sine2_lut_scale
    mov  Sine_D_Pair, A                 ; duty8_pair
    ; --- duty8 for the second phase ---
    mov  A, Sine_D_Second
    call sine2_idx
    call sine2_lut_scale
    mov  Sine_D_Second, A               ; duty8_second

    ; --- write duties: pair via modules 0/1, second via module 2 ---
    mov  Sine_Amp, Sine_D_Pair          ; hijack Sine_Amp for the pair (recomputed next tick)
    call sine_set_duty                  ; [Inv 3/4] deadtime-skewed complementary pair
    mov  A, Sine_D_Second
    call sine2_write_second             ; CEX2 high-side (its Com FET is latched off)

    ; --- remux only if the clamp phase changed (3x/erev, second duty ~0 at swaps) ---
    call sine2_calc_clamp
    mov  Sine_D_Pair, A                 ; stash new clamp
    cjne A, Sine_Seg, sine2_sd_remux
    ret
sine2_sd_remux:
    mov  A, Sine_D_Pair
    call sine2_apply_segment
    mov  Sine_Seg, Sine_D_Pair
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; cross_rescale_duty - BlueGill S3 smooth-handoff duty rescale.
;
; While the crossover is engaged in 6-step, the raw open-loop throttle would drive a HIGH duty
; right at the handoff, so the rotor jumps from ~Cross_Up speed to that duty's natural (much
; higher) speed. Rescale the 11-bit demand (Temp4=lo, Temp5=hi) to a DETERMINISTIC affine of the
; throttle so thrust->speed is continuous and monotonic across the seam:
;   Rcp_cross = Cross_Up << 5                       ; the throttle at which sine reaches Cross_Up
;   throttle <  Rcp_cross -> duty = CROSS_DUTY_MIN   ; low floor across the hysteresis band
;   throttle >= Rcp_cross -> duty = CROSS_DUTY_MIN + (throttle-Rcp_cross)*2.5, clamp 0x7FF
; A PURE function of throttle+config (NO run-time capture), so the same command always maps to the
; same duty => same steady speed under a given load. Calibration-free (CROSS_DUTY_MIN is a small
; fixed seed; slope 2.5 reaches full scale for Cross_Up >= ~32). Called from t1_int ONLY while in
; crossover 6-step. Runs in ISR bank 1; clobbers A, B, C, Temp1, Temp2, Temp3 (all recomputed by
; the caller right after). In/out: Temp4/Temp5.
CROSS_DUTY_MIN EQU 96                                ; ~4.7% duty handoff floor (~500 mech at this Kv/V)
;**** **** **** **** **** **** **** **** **** **** **** **** ****
cross_rescale_duty:
    mov  Temp1, #Pgm_Sine_Cross_Up
    mov  A, @Temp1
    jz   crd_ret                                    ; Cross_Up == 0 -> crossover off (safety)
    ; Rcp_cross = Cross_Up << 5  ->  Temp2 (lo) : Temp3 (hi)
    mov  B, A
    anl  A, #07h
    swap A
    add  A, ACC
    mov  Temp2, A                                   ; Rcp_cross_L = (Cross_Up & 7) << 5
    mov  A, B
    rr   A
    rr   A
    rr   A
    anl  A, #1Fh
    mov  Temp3, A                                   ; Rcp_cross_H = Cross_Up >> 3
    ; Delta = throttle(Temp4/Temp5) - Rcp_cross(Temp2/Temp3)
    clr  C
    mov  A, Temp4
    subb A, Temp2
    mov  Temp2, A                                   ; Delta_L
    mov  A, Temp5
    subb A, Temp3
    jc   crd_below                                  ; throttle < Rcp_cross -> floor to CROSS_DUTY_MIN
    mov  Temp3, A                                   ; Delta_H
    ; (Delta * 2) -> Temp4 (lo) : Temp5 (hi)
    mov  A, Temp2
    clr  C
    rlc  A
    mov  Temp4, A
    mov  A, Temp3
    rlc  A
    mov  Temp5, A
    ; (Delta >> 1) -> B (hi) : A (lo)
    clr  C
    mov  A, Temp3
    rrc  A
    mov  B, A
    mov  A, Temp2
    rrc  A
    ; sum = Delta*2 + Delta>>1  (== Delta * 2.5)
    add  A, Temp4
    mov  Temp4, A
    mov  A, B
    addc A, Temp5
    mov  Temp5, A
    ; + CROSS_DUTY_MIN
    mov  A, Temp4
    add  A, #CROSS_DUTY_MIN
    mov  Temp4, A
    mov  A, Temp5
    addc A, #0
    mov  Temp5, A
    ; clamp to 0x07FF (11-bit)
    anl  A, #0F8h
    jz   crd_ret
    mov  Temp4, #0FFh
    mov  Temp5, #07h
crd_ret:
    ret
crd_below:
    mov  Temp4, #CROSS_DUTY_MIN
    mov  Temp5, #00h
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Sine2_Arc_Lut - min-clamp cosine-arc LUT, L[j] = round(255*sin(j*1.875 deg)),
; j = 0..48 (49 bytes, peak 255). A modulated phase's high-side duty (0..255) is
; L[48 - min_circular_dist_to_peak]. Generated + cross-checked (line-to-line RMS
; 0.18%, duty in [0,255]) by tools/sim/sine_drive_model.py print_s2_dpwm_section.
; MUST stay byte-identical to that emitter.
;**** **** **** **** **** **** **** **** **** **** **** **** ****
Sine2_Arc_Lut:
    DB 0,8,17,25,33,42,50,58,66,74,82,90,98,105,113,120,127,135,142,149
    DB 155,162,168,174,180,186,192,197,202,207,212,217,221,225,229,232,236,239,241,244
    DB 246,248,250,252,253,254,254,255,255
