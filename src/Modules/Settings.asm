;**** **** **** **** ****
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
; Settings
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

; Sets default programming parameters
set_default_parameters:
    mov  Temp1, #_Pgm_Gov_P_Gain
    mov  @Temp1, #0FFh                  ; _Pgm_Gov_P_Gain
    imov Temp1, #DEFAULT_PGM_STARTUP_POWER_MIN ; Pgm_Startup_Power_Min
    imov Temp1, #DEFAULT_PGM_STARTUP_BEEP ; Pgm_Startup_Beep
    imov Temp1, #0FFh                   ; _Pgm_Dithering
    imov Temp1, #DEFAULT_PGM_STARTUP_POWER_MAX ; Pgm_Startup_Power_Max
    imov Temp1, #0FFh                   ; _Pgm_Rampup_Slope
    imov Temp1, #DEFAULT_PGM_RPM_POWER_SLOPE ; Pgm_Rpm_Power_Slope
    imov Temp1, #(24 SHL PWM_FREQ)      ; Pgm_Pwm_Freq
    imov Temp1, #DEFAULT_PGM_DIRECTION  ; Pgm_Direction
    imov Temp1, #0FFh                   ; _Pgm_Input_Pol

    inc  Temp1                          ; Skip Initialized_L_Dummy
    inc  Temp1                          ; Skip Initialized_H_Dummy

    imov Temp1, #0FFh                   ; _Pgm_Enable_TX_Program
    imov Temp1, #DEFAULT_PGM_BRAKING_STRENGTH ; Pgm_Braking_Strength
    imov Temp1, #0FFh                   ; _Pgm_Gov_Setup_Target
    imov Temp1, #0FFh                   ; _Pgm_Startup_Rpm
    imov Temp1, #0FFh                   ; _Pgm_Startup_Accel
    imov Temp1, #0FFh                   ; _Pgm_Volt_Comp
    imov Temp1, #DEFAULT_PGM_COMM_TIMING ; Pgm_Comm_Timing
    imov Temp1, #0FFh                   ; _Pgm_Damping_Force
    imov Temp1, #0FFh                   ; _Pgm_Gov_Range
    imov Temp1, #0FFh                   ; _Pgm_Startup_Method
    imov Temp1, #0FFh                   ; _Pgm_Min_Throttle
    imov Temp1, #0FFh                   ; _Pgm_Max_Throttle
    imov Temp1, #DEFAULT_PGM_BEEP_STRENGTH ; Pgm_Beep_Strength
    imov Temp1, #DEFAULT_PGM_BEACON_STRENGTH ; Pgm_Beacon_Strength
    imov Temp1, #DEFAULT_PGM_BEACON_DELAY ; Pgm_Beacon_Delay
    imov Temp1, #0FFh                   ; _Pgm_Throttle_Rate
    imov Temp1, #DEFAULT_PGM_DEMAG_COMP ; Pgm_Demag_Comp
    imov Temp1, #0FFh                   ; _Pgm_BEC_Voltage_High
    imov Temp1, #0FFh                   ; _Pgm_Center_Throttle
    imov Temp1, #0FFh                   ; _Pgm_Main_Spoolup_Time
    imov Temp1, #DEFAULT_PGM_ENABLE_TEMP_PROT ; Pgm_Enable_Temp_Prot
    imov Temp1, #0FFh                   ; _Pgm_Enable_Power_Prot
    imov Temp1, #0FFh                   ; _Pgm_Enable_Pwm_Input
    imov Temp1, #0FFh                   ; _Pgm_Pwm_Dither
    imov Temp1, #DEFAULT_PGM_BRAKE_ON_STOP ; Pgm_Brake_On_Stop
    imov Temp1, #DEFAULT_PGM_LED_CONTROL ; Pgm_LED_Control
    imov Temp1, #DEFAULT_PGM_POWER_RATING ; Pgm_Power_Rating
    imov Temp1, #DEFAULT_PGM_SAFETY_ARM ; Pgm_Safety_Arm
    imov Temp1, #DEFAULT_PGM_COMM_TIMING_ANGLE ; Pgm_Comm_Timing_Angle (BlueGill)
    imov Temp1, #DEFAULT_PGM_MAX_ERPM   ; Pgm_Max_Erpm (BlueGill)
    imov Temp1, #DEFAULT_PGM_LOWSPEED_DAMPING ; Pgm_LowSpeed_Damping (BlueGill)
    imov Temp1, #DEFAULT_PGM_SINE_MODE  ; Pgm_Sine_Mode (BlueGill S1)
    imov Temp1, #DEFAULT_PGM_SINE_HOLD_AMP ; Pgm_Sine_Hold_Amp (BlueGill S1)
    imov Temp1, #DEFAULT_PGM_SINE_AMP_MAX ; Pgm_Sine_Amp_Max (BlueGill S1)
    imov Temp1, #DEFAULT_PGM_SINE_RAMP  ; Pgm_Sine_Ramp (BlueGill S1)
    imov Temp1, #DEFAULT_PGM_SINE_CROSS_UP ; Pgm_Sine_Cross_Up (BlueGill S3)
    imov Temp1, #DEFAULT_PGM_SINE_CROSS_DN ; Pgm_Sine_Cross_Dn (BlueGill S3)

    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Decode settings
;
; Decodes programmed settings and set RAM variables accordingly
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
decode_settings:
    mov  Temp1, #Pgm_Direction          ; Load programmed direction
    mov  A, @Temp1
    dec  A
    mov  C, ACC.1                       ; Set bidirectional mode
    mov  Flag_Pgm_Bidir, C
    mov  C, ACC.0                       ; Set direction (Normal / Reversed)
    mov  Flag_Pgm_Dir_Rev, C

    ; Check startup power
    mov  Temp1, #Pgm_Startup_Power_Max
    mov  A, #80                         ; Limit to at most 80
    subb A, @Temp1
    jnc  decode_settings_check_low_rpm
    mov  @Temp1, #80

decode_settings_check_low_rpm:
    ; Check low rpm power slope
    mov  Temp1, #Pgm_Rpm_Power_Slope
    mov  A, #13                         ; Limit to at most 13
    subb A, @Temp1
    jnc  decode_settings_set_low_rpm
    mov  @Temp1, #13

decode_settings_set_low_rpm:
    mov  Low_Rpm_Pwr_Slope, @Temp1

    ; Decode demag compensation
    mov  Temp1, #Pgm_Demag_Comp
    mov  A, @Temp1
    mov  Demag_Pwr_Off_Thresh, #255     ; Set default

    cjne A, #2, decode_demag_high

    mov  Demag_Pwr_Off_Thresh, #160     ; Settings for demag comp low

decode_demag_high:
    cjne A, #3, decode_demag_done

    mov  Demag_Pwr_Off_Thresh, #130     ; Settings for demag comp high

decode_demag_done:
    ; Decode temperature protection limit
    mov  Temp_Prot_Limit, #0
    mov  Temp1, #Pgm_Enable_Temp_Prot
    mov  A, @Temp1
    mov  Temp2, A                       ; Temp2 = *Pgm_Enable_Temp_Prot;
    jz   decode_temp_done

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Power rating only applies to BB21 because voltage references behave diferently
; depending on if an external voltage regulator is used or not.
;
; NOTE: For BB51, the 1s power rating code path is mandatory
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
IF MCU_TYPE == MCU_BB1 or MCU_TYPE == MCU_BB2
    ; Read power rating and decode temperature limit
    mov  Temp1, #Pgm_Power_Rating
    cjne @Temp1, #01h, decode_temp_use_adc_use_vdd_3V3_vref
ENDIF

; Set A to temperature limit depending on power rating
decode_temp_use_adc_use_internal_1V65_vref:
    mov  A, #(TEMP_LIMIT_1S - TEMP_LIMIT_STEP)
    sjmp decode_temp_step
decode_temp_use_adc_use_vdd_3V3_vref:
    mov  A, #(TEMP_LIMIT_2S - TEMP_LIMIT_STEP)

; Increase A while Temp2-- != 0;
decode_temp_step:
    add  A, #TEMP_LIMIT_STEP
    djnz Temp2, decode_temp_step

; Set Temp_Prot_Limit to the temperature limit calculated in A
decode_temp_done:
    mov  Temp_Prot_Limit, A

    mov  Temp1, #Pgm_Beep_Strength      ; Read programmed beep strength setting
    mov  Beep_Strength, @Temp1          ; Set beep strength

    mov  Temp1, #Pgm_Braking_Strength   ; Read programmed braking strength setting
    mov  A, @Temp1
    call scale_braking_strength         ; Temp5/Temp6 = scaled Pwm_Braking_L/H
    mov  Pwm_Braking_L, Temp5
    mov  Pwm_Braking_H, Temp6

    ; BlueGill: decode appended parameters (all default off => stock behavior)
    call decode_bluegill_parameters

decode_end:
    ; Return
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Scale braking strength to pwm resolution
;
; In:  A = braking strength (0..255)
; Out: Temp5 = Pwm_Braking_L, Temp6 = Pwm_Braking_H
; Clobbers: A, B, Temp3
;
; Reusable by both decode_settings and the low-speed damping override.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
scale_braking_strength:
    mov  Temp3, A                       ; Preserve original strength
IF PWM_BITS_H == PWM_11_BIT             ; Scale braking strength to pwm resolution
    ; Note: Added for completeness
    ; Currently 11-bit pwm is only used on targets with built-in dead time insertion
    rl   A
    rl   A
    rl   A
    mov  B, A
    anl  A, #07h
    mov  Temp6, A
    mov  A, B
    anl  A, #0F8h
    mov  Temp5, A
ELSEIF PWM_BITS_H == PWM_10_BIT
    rl   A
    rl   A
    mov  B, A
    anl  A, #03h
    mov  Temp6, A
    mov  A, B
    anl  A, #0FCh
    mov  Temp5, A
ELSEIF PWM_BITS_H == PWM_9_BIT
    rl   A
    mov  B, A
    anl  A, #01h
    mov  Temp6, A
    mov  A, B
    anl  A, #0FEh
    mov  Temp5, A
ELSEIF PWM_BITS_H == PWM_8_BIT
    mov  Temp6, #0
    mov  Temp5, A
ENDIF
    mov  A, Temp3
    cjne A, #0FFh, scale_braking_strength_done
    mov  Temp5, #0FFh                   ; Apply full braking if setting is max
scale_braking_strength_done:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Decode BlueGill appended parameters
;
; Direct commutation timing angle (0x2B), eRPM cap (0x2C) and low-speed
; damping (0x2D). A byte value of 0xFF is stale stock-Bluejay EEPROM data
; (BlueGill flashed over a stock config) and is treated as OFF.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
decode_bluegill_parameters:
    ; --- Direct commutation timing angle -> Comm_Timing_Angle_Adj ---
    mov  Temp1, #Pgm_Comm_Timing_Angle
    mov  A, @Temp1
    cjne A, #0FFh, decode_cta_have
    clr  A                              ; 0xFF stale -> off
decode_cta_have:
    mov  Temp2, A                       ; Sanitized value
    jz   decode_cta_store               ; 0 = off (use preset)
    clr  C
    mov  A, Temp2
    subb A, #18                         ; Valid range is 1..17
    jc   decode_cta_store               ; < 18 -> keep value in Temp2
    mov  Temp2, #0                      ; >= 18 invalid -> off
decode_cta_store:
    mov  Comm_Timing_Angle_Adj, Temp2

    ; --- eRPM cap -> Max_Erpm_Cap / Max_Erpm_Rel (as Comm_Period4x thresholds) ---
    mov  Max_Erpm_Cap_L, #0
    mov  Max_Erpm_Cap_H, #0
    mov  Max_Erpm_Rel_L, #0
    mov  Max_Erpm_Rel_H, #0
    mov  Temp1, #Pgm_Max_Erpm
    mov  A, @Temp1
    cjne A, #0FFh, decode_erpm_have
    clr  A                              ; 0xFF stale -> off
decode_erpm_have:
    jz   decode_damping                 ; 0 = disabled
    ; Clamp to the max exactly-representable cap: 80000/N is exact only for N <= 136
    ; (see div_80000). Above ~156k eRPM the high-rpm path bypasses this governor, so
    ; the effective ceiling is <= 136k eRPM. Larger N is clamped down to 136 here.
    clr  C
    subb A, #137                        ; A - 137
    jc   decode_erpm_in_range           ; N < 137 -> keep
    mov  A, #136                        ; N >= 137 -> clamp to 136
    sjmp decode_erpm_div
decode_erpm_in_range:
    add  A, #137                        ; restore N (undo the compare subtract)
decode_erpm_div:
    call div_80000                      ; In A=N (1000 eRPM units); Out Temp3/Temp4 = 80000/N (Comm_Period4x)
    mov  Max_Erpm_Cap_L, Temp3
    mov  Max_Erpm_Cap_H, Temp4
    ; Rel = Cap + (Cap >> 4) (~6% hysteresis: larger period releases at lower rpm)
    mov  A, Temp3
    mov  Temp5, A
    mov  A, Temp4
    mov  Temp6, A
    mov  B, #4
decode_erpm_shift:
    clr  C
    mov  A, Temp6
    rrc  A
    mov  Temp6, A
    mov  A, Temp5
    rrc  A
    mov  Temp5, A
    djnz B, decode_erpm_shift
    mov  A, Temp3
    add  A, Temp5
    mov  Max_Erpm_Rel_L, A
    mov  A, Temp4
    addc A, Temp6
    mov  Max_Erpm_Rel_H, A
    jnc  decode_damping
    mov  Max_Erpm_Rel_L, #0FFh          ; Clamp on 16-bit overflow
    mov  Max_Erpm_Rel_H, #0FFh

decode_damping:
    ; --- Low-speed damping: sanitize stale 0xFF -> off; reset state ---
    mov  Temp1, #Pgm_LowSpeed_Damping
    mov  A, @Temp1
    cjne A, #0FFh, decode_damping_done
    mov  @Temp1, #0                     ; 0xFF stale -> off
decode_damping_done:
    mov  LowSpeed_Damping_State, #0

    ; --- BlueGill S1 forced-commutation stepper params (0x2E..0x31) ---
    ; A stale 0xFF (BlueGill flashed over a stock/older config) reads as OFF/default,
    ; and the amplitude params are clamped so a bad EEPROM can never drive full duty.
    clr  Flag_Sine_Mode                 ; default off; also clear the run gate for safety
    clr  Flag_Sine_Run
    clr  Flag_Sine_Micro                ; BlueGill S2: default off (only Pgm_Sine_Mode==2 sets it)

    ; Sine mode enable (0x2E): 0xFF stale -> off; 1 = S1 stepper; 2 = S2 micro-stepping
    mov  Temp1, #Pgm_Sine_Mode
    mov  A, @Temp1
    cjne A, #0FFh, decode_sine_mode_have
    clr  A                              ; 0xFF stale -> off
    mov  @Temp1, A
decode_sine_mode_have:
    jz   decode_sine_hold               ; 0 = off
    setb Flag_Sine_Mode
    cjne A, #2, decode_sine_hold        ; 1 (or other) = S1; exactly 2 = S2 micro-stepping
    setb Flag_Sine_Micro

decode_sine_hold:
    ; Hold amplitude (0x2F): 0xFF stale -> default; clamp to SINE_HOLD_AMP_CLAMP
    mov  Temp1, #Pgm_Sine_Hold_Amp
    mov  A, @Temp1
    cjne A, #0FFh, decode_sine_hold_have
    mov  A, #DEFAULT_PGM_SINE_HOLD_AMP  ; 0xFF stale -> default
decode_sine_hold_have:
    clr  C
    mov  Temp2, A
    subb A, #(SINE_HOLD_AMP_CLAMP + 1)  ; value - (clamp+1)
    jc   decode_sine_hold_store         ; value <= clamp -> keep
    mov  Temp2, #SINE_HOLD_AMP_CLAMP    ; over clamp -> clamp
decode_sine_hold_store:
    mov  Temp1, #Pgm_Sine_Hold_Amp
    mov  A, Temp2
    mov  @Temp1, A                      ; write back sanitized/clamped value

    ; Amplitude ceiling (0x30): 0xFF stale -> default; clamp to SINE_AMP_MAX_CLAMP
    mov  Temp1, #Pgm_Sine_Amp_Max
    mov  A, @Temp1
    cjne A, #0FFh, decode_sine_amp_have
    mov  A, #DEFAULT_PGM_SINE_AMP_MAX   ; 0xFF stale -> default
decode_sine_amp_have:
    clr  C
    mov  Temp2, A
    subb A, #(SINE_AMP_MAX_CLAMP + 1)
    jc   decode_sine_amp_store
    mov  Temp2, #SINE_AMP_MAX_CLAMP
decode_sine_amp_store:
    mov  Temp1, #Pgm_Sine_Amp_Max
    mov  A, Temp2
    mov  @Temp1, A

    ; Slew rate (0x31): 0xFF stale -> default (no clamp; 1..255 all valid)
    mov  Temp1, #Pgm_Sine_Ramp
    mov  A, @Temp1
    cjne A, #0FFh, decode_sine_ramp_done
    mov  @Temp1, #DEFAULT_PGM_SINE_RAMP ; 0xFF stale -> default
decode_sine_ramp_done:

    ; --- BlueGill S3 sine<->BEMF crossover thresholds (0x32/0x33) + down-handoff seed ---
    ; Both default OFF (0) => no crossover, modes 0/1/2 unchanged. A stale 0xFF reads as
    ; OFF. Cross_Dn is clamped < 0xF0 so a down-handoff always fires before the stock
    ; min-speed exit. Sine_Inc_Seed = 2048000/Cross_Dn pre-seeds the field rate on re-entry.
    clr  Flag_Sine_Handoff              ; no pending handoff at init (set only by run6_check_speed)

    ; Cross_Up: 0xFF stale -> 0
    mov  Temp1, #Pgm_Sine_Cross_Up
    mov  A, @Temp1
    cjne A, #0FFh, decode_cross_up_have
    clr  A
    mov  @Temp1, A
decode_cross_up_have:

    ; Cross_Dn: 0xFF stale -> 0; clamp >= 0xF0 down to 0xEF (must precede the stock 0xF0 exit)
    mov  Temp1, #Pgm_Sine_Cross_Dn
    mov  A, @Temp1
    cjne A, #0FFh, decode_cross_dn_have
    clr  A
decode_cross_dn_have:
    mov  Temp2, A                       ; Temp2 = Cross_Dn candidate
    clr  C
    subb A, #0F0h
    jc   decode_cross_dn_store          ; Cross_Dn < 0xF0 -> keep
    mov  Temp2, #0EFh                   ; >= 0xF0 -> clamp
decode_cross_dn_store:
    mov  Temp1, #Pgm_Sine_Cross_Dn
    mov  A, Temp2
    mov  @Temp1, A                      ; write back sanitized Cross_Dn

    ; Sine_Inc_Seed = 2048000 / Cross_Dn  (0 when Cross_Dn == 0)
    mov  Temp1, #Sine_Inc_Seed_L
    clr  A
    mov  @Temp1, A
    inc  Temp1
    mov  @Temp1, A                      ; default seed = 0
    mov  A, Temp2                       ; Cross_Dn
    jz   decode_cross_guard             ; Cross_Dn == 0 -> leave seed 0
    call div_2048000                    ; In A=Cross_Dn; Out Temp3=lo, Temp4=hi
    mov  Temp1, #Sine_Inc_Seed_L
    mov  A, Temp3
    mov  @Temp1, A
    inc  Temp1
    mov  A, Temp4
    mov  @Temp1, A

decode_cross_guard:
    ; Disable the crossover (zero both thresholds) when Cross_Up == 0, or when the down
    ; seed's high byte would already be at/above Cross_Up -> re-entering sine would instantly
    ; re-trigger the up-handoff (chatter). The host validates this too; belt and suspenders.
    mov  Temp1, #Pgm_Sine_Cross_Up
    mov  A, @Temp1
    jz   decode_cross_disable           ; Cross_Up == 0 -> disable
    mov  Temp1, #Sine_Inc_Seed_H
    mov  A, @Temp1
    clr  C
    mov  Temp1, #Pgm_Sine_Cross_Up
    subb A, @Temp1                      ; Sine_Inc_Seed_H - Cross_Up
    jc   decode_cross_done              ; seed_H < Cross_Up -> OK, keep both
decode_cross_disable:
    clr  A
    mov  Temp1, #Pgm_Sine_Cross_Up
    mov  @Temp1, A
    mov  Temp1, #Pgm_Sine_Cross_Dn
    mov  @Temp1, A                      ; both 0 -> crossover fully off
decode_cross_done:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Divide 80000 by N
;
; In:  A = divisor N (nonzero)
; Out: Temp3 = quotient lo, Temp4 = quotient hi (clamped to 0xFFFF on overflow)
; Uses: A, B, Temp2..Temp6
;
; 24-bit / 8-bit restoring division of the constant 80000 (0x013880). Used to
; convert the eRPM cap (1000 eRPM units) into a Comm_Period4x threshold, since
; eRPM ~= 80e6 / Comm_Period4x, so Comm_Period4x = 80000 / (cap in 1000 eRPM).
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
div_80000:
    mov  Temp3, #080h                   ; Dividend lo  (0x013880)
    mov  Temp4, #038h                   ; Dividend mid
    mov  Temp5, #001h                   ; Dividend hi
    sjmp div_generic

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; BlueGill S3: Divide 2,048,000 (0x1F4000) by N -> the same 24/8 restoring core.
; In:  A = divisor N (nonzero). Out: Temp3 = quotient lo, Temp4 = quotient hi
; (clamped to 0xFFFF on overflow). Used to seed Sine_Inc = 2048000/Cross_Dn so the
; down-handoff re-enters forced sine at the rotor's current field rate.
; (2048000 = Cross_Dn eRPM step 312500 * 65536/10000 = the Sine_Inc-per-eRPM factor.)
;**** **** **** **** **** **** **** **** **** **** **** **** ****
div_2048000:
    mov  Temp3, #000h                   ; Dividend lo  (0x1F4000)
    mov  Temp4, #040h                   ; Dividend mid
    mov  Temp5, #01Fh                   ; Dividend hi

div_generic:                            ; A = divisor; Temp3/4/5 = 24-bit dividend (preloaded)
    mov  Temp2, A                       ; Divisor (A preserved across the immediate loads above)
    mov  Temp6, #0                      ; Remainder
    mov  B, #24
div_80000_loop:
    clr  C                              ; Shift dividend left (MSB -> carry)
    mov  A, Temp3
    rlc  A
    mov  Temp3, A
    mov  A, Temp4
    rlc  A
    mov  Temp4, A
    mov  A, Temp5
    rlc  A
    mov  Temp5, A
    mov  A, Temp6                       ; Shift MSB into remainder (low 8 bits)
    rlc  A
    mov  Temp6, A
    mov  A, #0                          ; Capture the 9th remainder bit (carry-out)
    rlc  A                              ; A = bit8 (leaves carry clear)
    mov  Temp7, A                       ; Temp7 = remainder bit 8 (0 or 1)
    ; remainder' = 256*Temp7 + Temp6 ; subtract divisor if remainder' >= divisor.
    ; The subb below can only see the low 8 bits, so bit8 (remainder' >= 256 > any
    ; 8-bit divisor) forces the subtract regardless of the borrow result.
    clr  C
    mov  A, Temp6
    subb A, Temp2                       ; A = (Temp6 - divisor) mod 256 = correct new remainder
    mov  Temp8, A                       ; Stash candidate remainder''
    jnc  div_80000_take                 ; Temp6 >= divisor -> subtract
    mov  A, Temp7
    jz   div_80000_next                 ; bit8 == 0 and borrow -> remainder' < divisor
div_80000_take:
    mov  A, Temp8                       ; Commit remainder -= divisor
    mov  Temp6, A
    inc  Temp3                          ; set quotient LSB (was shifted in as 0)
div_80000_next:
    djnz B, div_80000_loop
    mov  A, Temp5                       ; Quotient hi byte non-zero => > 16-bit
    jz   div_80000_ret
    mov  Temp3, #0FFh                   ; Clamp to 0xFFFF
    mov  Temp4, #0FFh
div_80000_ret:
    ret
