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
; Power control
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Switch power off
;
; Switches all FETs off
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
switch_power_off:
    All_Pwm_Fets_Off                    ; Turn off all pwm FETs
    All_Com_Fets_Off                    ; Turn off all commutation FETs
    Set_All_Pwm_Phases_Off
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set PWM limit low RPM
;
; Sets power limit for low RPM
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit:
    jnb  Flag_High_Rpm, set_pwm_limit_low_rpm ; If high rpm,limit pwm by rpm instead
    ljmp set_pwm_limit_high_rpm         ; (long jump: BlueGill code widened this span)

set_pwm_limit_low_rpm:
    ; Set pwm limit for startup phase to avoid burning the esc/motor during startup
    ; (Startup can happen after a desync caused by a crash, if that is the case it
    ; will be better to avoid burning esc/motor)
    mov  Temp1, Pwm_Limit_Beg

    ; Exit if startup phase is set
    jb   Flag_Startup_Phase, set_pwm_limit_low_rpm_exit

    ; Set default pwm limit for other phases
    mov  Temp1, #0FFh                   ; Default full power

    mov  A, Low_Rpm_Pwr_Slope           ; Check if low RPM power protection is enabled
    jz   set_pwm_limit_low_rpm_exit     ; Exit if disabled (zero)

    mov  A, Comm_Period4x_H
    jz   set_pwm_limit_low_rpm_exit     ; Avoid divide by zero

    mov  A, #255                        ; Divide 255 by Comm_Period4x_H
    jnb  Flag_Initial_Run_Phase, set_pwm_limit_calculate ; More protection for initial run phase
    mov  A, #127

set_pwm_limit_calculate:
    mov  B, Comm_Period4x_H
    div  AB
    mov  B, Low_Rpm_Pwr_Slope           ; Multiply by slope
    mul  AB
    mov  Temp1, A                       ; Set new limit
    xch  A, B

    ; If RPM_PWM_LIMIT < 255 goto set_pwm_limit_check_limit_to_min
    jz   set_pwm_limit_check_limit_to_min ; Limit to max

    ; Limit is bigger than 0xFF -> set max pwm and exit
    mov  Pwm_Limit_By_Rpm, #0FFh
    sjmp apply_erpm_cap                 ; BlueGill: fold in eRPM cap governor

set_pwm_limit_check_limit_to_min:
    clr  C
    mov  A, Temp1                       ; Limit to min
    subb A, Pwm_Limit_Beg
    jnc  set_pwm_limit_low_rpm_exit

    mov  Temp1, Pwm_Limit_Beg

set_pwm_limit_low_rpm_exit:
    mov  Pwm_Limit_By_Rpm, Temp1
    ; fall through to apply_erpm_cap (BlueGill)

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill: eRPM cap governor
;
; Optional hard eRPM ceiling. When enabled, ramps Pwm_Limit_By_Erpm down while
; Comm_Period4x is below the cap (rpm above target) and back up above the release
; threshold, then folds it into Pwm_Limit_By_Rpm via min(). Disabled (cap == 0)
; leaves Pwm_Limit_By_Rpm untouched => byte-identical stock behavior. Low-rpm path
; only (the high-rpm governor is left untouched).
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
apply_erpm_cap:
    mov  A, Max_Erpm_Cap_L
    orl  A, Max_Erpm_Cap_H
    jz   apply_erpm_cap_ret             ; Cap == 0 -> disabled
    ; Compare Comm_Period4x against the cap threshold
    clr  C
    mov  A, Comm_Period4x_L
    subb A, Max_Erpm_Cap_L
    mov  A, Comm_Period4x_H
    subb A, Max_Erpm_Cap_H
    jnc  apply_erpm_cap_check_rel       ; Comm_Period4x >= Cap -> not over target
    ; Over target rpm -> decrement governor (floor 10)
    mov  A, Pwm_Limit_By_Erpm
    clr  C
    subb A, #11
    jc   apply_erpm_cap_min             ; already <= 10 -> hold
    dec  Pwm_Limit_By_Erpm
    sjmp apply_erpm_cap_min

apply_erpm_cap_check_rel:
    clr  C
    mov  A, Comm_Period4x_L
    subb A, Max_Erpm_Rel_L
    mov  A, Comm_Period4x_H
    subb A, Max_Erpm_Rel_H
    jc   apply_erpm_cap_min             ; between Cap and Rel -> hold
    ; Below release rpm -> increment governor (ceil 255)
    mov  A, Pwm_Limit_By_Erpm
    cjne A, #0FFh, apply_erpm_cap_inc
    sjmp apply_erpm_cap_min
apply_erpm_cap_inc:
    inc  Pwm_Limit_By_Erpm

apply_erpm_cap_min:
    ; Pwm_Limit_By_Rpm = min(Pwm_Limit_By_Rpm, Pwm_Limit_By_Erpm)
    clr  C
    mov  A, Pwm_Limit_By_Rpm
    subb A, Pwm_Limit_By_Erpm
    jc   apply_erpm_cap_ret             ; Pwm_Limit_By_Rpm < Erpm -> keep
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_By_Erpm
apply_erpm_cap_ret:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill: low-speed damping switch
;
; When enabled, overrides the braking (complementary-pwm) strength with a
; separate value below a low-speed threshold, reverting above it (with
; hysteresis). Disabled (Pgm_LowSpeed_Damping == 0) is a no-op. On DEADTIME == 0
; targets Pwm_Braking is unused so this has no audible/electrical effect.
;
; NOTE: Pwm_Braking_L/H is shared with Brake_On_Stop. If damping is engaged when
; the motor stops, brake-on-stop momentarily uses the damping strength rather than
; Pgm_Braking_Strength. This is not hazardous (identical PWM-limit envelope) and is
; left as-is for a thin diff; restoring the programmed strength on the stop path is
; an optional future refinement.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
update_lowspeed_damping:
    mov  Temp1, #Pgm_LowSpeed_Damping
    mov  A, @Temp1
    jz   update_lowspeed_damping_off    ; feature disabled
    ; Enabled: decide engage/release from Comm_Period4x_H (larger = slower)
    mov  A, LowSpeed_Damping_State
    jnz  update_lowspeed_damping_active
    ; Currently normal braking: engage if slow enough
    clr  C
    mov  A, Comm_Period4x_H
    subb A, #LOWSPEED_DAMPING_THR
    jc   update_lowspeed_damping_ret    ; above speed threshold -> stay normal
    mov  Temp1, #Pgm_LowSpeed_Damping
    mov  A, @Temp1
    call scale_braking_strength
    clr  IE_EA                          ; Atomic 2-byte Pwm_Braking store vs ISR
    mov  Pwm_Braking_L, Temp5
    mov  Pwm_Braking_H, Temp6
    setb IE_EA
    mov  LowSpeed_Damping_State, #1
    ret

update_lowspeed_damping_active:
    ; Currently damping: revert when fast enough (hysteresis)
    clr  C
    mov  A, Comm_Period4x_H
    subb A, #LOWSPEED_DAMPING_REL
    jnc  update_lowspeed_damping_ret    ; still slow -> stay in damping
    sjmp update_lowspeed_damping_restore

update_lowspeed_damping_off:
    mov  A, LowSpeed_Damping_State      ; if we had engaged, restore once
    jz   update_lowspeed_damping_ret

update_lowspeed_damping_restore:
    mov  Temp1, #Pgm_Braking_Strength
    mov  A, @Temp1
    call scale_braking_strength
    clr  IE_EA
    mov  Pwm_Braking_L, Temp5
    mov  Pwm_Braking_H, Temp6
    setb IE_EA
    mov  LowSpeed_Damping_State, #0
update_lowspeed_damping_ret:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set PWM limit high RPM
;
; Sets power limit for high RPM
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit_high_rpm:
    clr  C
    mov  A, Comm_Period4x_L
IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    subb A, #0A0h                       ; Limit Comm_Period4x to 160,which is ~510k erpm
ELSE
    subb A, #0E4h                       ; Limit Comm_Period4x to 228,which is ~358k erpm
ENDIF
    mov  A, Comm_Period4x_H
    subb A, #00h

    mov  A, Pwm_Limit_By_Rpm
    jnc  set_pwm_limit_high_rpm_inc_limit

    dec  A
    sjmp set_pwm_limit_high_rpm_store

set_pwm_limit_high_rpm_inc_limit:
    inc  A

set_pwm_limit_high_rpm_store:
    jz   set_pwm_limit_high_rpm_end
    mov  Pwm_Limit_By_Rpm, A

set_pwm_limit_high_rpm_end:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill S1: set PWM duty for forced-commutation stepper mode
;
; In:  Sine_Amp = V/f amplitude, 8-bit throttle-equivalent (already clamped to
;      min(Amp_Max, Pwm_Limit) by the caller, so it can never exceed the governed limit).
;
; This is the ONLY duty writer while Flag_Sine_Run is set (t1_int_set_pwm is gated off
; in that state), so there is a single writer of the PCA power/damp auto-reload registers.
; It mirrors t1_int's tail byte-for-byte: scale the amplitude to the PWM resolution and
; invert it, subtract DEADTIME to form the damping (complementary) reload, clamp that to
; Pwm_Braking (never crossing the power reload) and to >= 0, then write the auto-reload
; registers. It branches on PWM_BITS_H exactly like t1_int (do NOT assume a width) and
; writes the register pair with IE_EA off so no ISR can observe a torn 16-bit reload.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_set_duty:
    mov  A, Sine_Amp
    call sine_pwr_from_amp              ; Temp2:Temp3 = inverted power auto-reload for Sine_Amp

; Set PWM registers -- mirrors t1_int_set_pwm
IF DEADTIME != 0
    ; Subtract dead time from normal pwm and store as damping PWM
    ; Damping PWM duty cycle will be higher because numbers are inverted
    clr  C
    mov  A, Temp2                       ; Skew damping FET timing
IF MCU_TYPE == MCU_BB1
    subb A, #((DEADTIME + 1) SHR 1)
ELSE
    subb A, #(DEADTIME)
ENDIF
    mov  Temp4, A
    mov  A, Temp3
    subb A, #0
    mov  Temp5, A
    jnc  sine_set_duty_max_braking_set

    clr  A                              ; Set to minimum value
    mov  Temp4, A
    mov  Temp5, A
    sjmp sine_set_duty_pwm_braking_set  ; Max braking is already zero - branch

sine_set_duty_max_braking_set:
    clr  C
    mov  A, Temp4
    subb A, Pwm_Braking_L
    mov  A, Temp5
    subb A, Pwm_Braking_H               ; Is braking pwm more than maximum allowed braking?
    jc   sine_set_duty_pwm_braking_set  ; Yes - branch
    mov  Temp4, Pwm_Braking_L           ; No - set desired braking instead
    mov  Temp5, Pwm_Braking_H

sine_set_duty_pwm_braking_set:
ENDIF

    ; Write the auto-reload registers with interrupts disabled (single, atomic writer)
    clr  IE_EA
IF PWM_BITS_H != PWM_8_BIT
    ; Set power pwm auto-reload registers
    Set_Power_Pwm_Reg_L Temp2
    Set_Power_Pwm_Reg_H Temp3
ELSE
    Set_Power_Pwm_Reg_H Temp2
ENDIF

IF DEADTIME != 0
    ; Set damp pwm auto-reload registers
IF PWM_BITS_H != PWM_8_BIT
    Set_Damp_Pwm_Reg_L Temp4
    Set_Damp_Pwm_Reg_H Temp5
ELSE
    Set_Damp_Pwm_Reg_H Temp4
ENDIF
ENDIF
    setb IE_EA
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill S2: factored amplitude -> inverted power auto-reload (Temp2:Temp3)
;
; In:  A = 8-bit throttle-equivalent amplitude.
; Out: Temp2 (lo), Temp3 (hi) = inverted power PWM auto-reload (before dead-time skew).
; Clobbers A, B, Temp2..Temp5. Mirrors t1_int/S1 scaling byte-for-byte; shared by the
; S1/S2 "pair" writer (sine_set_duty above) and the S2 module-2 writer below, so there is
; exactly ONE copy of the width-dependent scale+invert math (flash budget).
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine_pwr_from_amp:
IF PWM_BITS_H == PWM_8_BIT              ; 8-bit pwm
    mov  Temp2, A
ELSE
    mov  B, #8                          ; Multiply amplitude by 8 for 11-bit pwm
    mul  AB
    mov  Temp4, A
    mov  Temp5, B
ENDIF

; Scale pwm resolution and invert (duty cycle is defined inversely) -- mirrors t1_int
IF PWM_BITS_H == PWM_11_BIT
    mov  A, Temp5
    cpl  A
    anl  A, #7
    mov  Temp3, A
    mov  A, Temp4
    cpl  A
    mov  Temp2, A
ELSEIF PWM_BITS_H == PWM_10_BIT
    clr  C
    mov  A, Temp5
    rrc  A
    cpl  A
    anl  A, #3
    mov  Temp3, A
    mov  A, Temp4
    rrc  A
    cpl  A
    mov  Temp2, A
ELSEIF PWM_BITS_H == PWM_9_BIT
    mov  B, Temp5
    mov  A, Temp4
    mov  C, B.0
    rrc  A
    mov  C, B.1
    rrc  A
    cpl  A
    mov  Temp2, A
    mov  A, Temp5
    rr   A
    rr   A
    cpl  A
    anl  A, #1
    mov  Temp3, A
ELSEIF PWM_BITS_H == PWM_8_BIT
    mov  A, Temp2                       ; Temp2 already 8-bit
    cpl  A
    mov  Temp2, A
    mov  Temp3, #0
ENDIF
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; BlueGill S2: write the free PCA module 2 (CEX2) auto-reload for the "second"
; modulated phase (high-side only; its Com/low-side FET is latched OFF, so the leg
; freewheels through the body diode and can never shoot through).
;
; In:  A = 8-bit throttle-equivalent amplitude for the second phase.
; Uses the shared sine_pwr_from_amp scaler (same width/invert as the pair), then
; writes PCA0CPL2/PCA0CPH2 with IE_EA off (no torn 16-bit reload). No damp register.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
sine2_write_second:
    call sine_pwr_from_amp              ; Temp2:Temp3 = inverted power reload for A
    clr  IE_EA
IF PWM_BITS_H != PWM_8_BIT
    Set_Power2_Pwm_Reg_L Temp2
    Set_Power2_Pwm_Reg_H Temp3
ELSE
    Set_Power2_Pwm_Reg_H Temp2
ENDIF
    setb IE_EA
    ret
