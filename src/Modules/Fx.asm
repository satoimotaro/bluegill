
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
; Misc utility functions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait a number of milliseconds (Multiple entry points)
;
; Requirements:
; - System clock should be set to 24MHz
; - Interrupts should be disabled for precision
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait1ms:
    mov  Temp3, #0                      ; Milliseconds (hi byte)
    mov  Temp2, #1                      ; Milliseconds (lo byte)
    sjmp wait_ms

wait5ms:
    mov  Temp3, #0
    mov  Temp2, #5
    sjmp wait_ms

wait10ms:
    mov  Temp3, #0
    mov  Temp2, #10
    sjmp wait_ms

wait100ms:
    mov  Temp3, #0
    mov  Temp2, #100
    sjmp wait_ms

wait200ms:
    mov  Temp3, #0
    mov  Temp2, #200
    sjmp wait_ms

wait250ms:
    mov  Temp3, #0
    mov  Temp2, #250
    sjmp wait_ms

wait_ms:
    inc  Temp2                          ; Increment for use with djnz
    inc  Temp3
    sjmp wait_ms_start

wait_ms_o:                              ; Outer loop
    mov  Temp1, #24

wait_ms_m:                              ; Middle loop
    mov  A, #255
    djnz ACC, $                         ; Inner loop (41.6us - 1020 cycles)
    djnz Temp1, wait_ms_m

wait_ms_start:
    djnz Temp2, wait_ms_o
    djnz Temp3, wait_ms_o
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Beeper routines (Multiple entry points)
;
; Requirements:
; - Interrupts must be disabled and FETs turned off
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
beep_f1:
    mov  Temp3, #66                     ; Off wait loop length (Tone)
    mov  Temp4, #(3500 / 66)            ; Number of beep pulses (Duration)
    sjmp beep

beep_f2:
    mov  Temp3, #45
    mov  Temp4, #(3500 / 45)
    sjmp beep

beep_f3:
    mov  Temp3, #38
    mov  Temp4, #(3500 / 38)
    sjmp beep

beep_f4:
    mov  Temp3, #25
    mov  Temp4, #(3500 / 25)
    sjmp beep

beep_f5:
    mov  Temp3, #20
    mov  Temp4, #(3500 / 20)
    sjmp beep

beep_f1_short:
    mov  Temp3, #66
    mov  Temp4, #(2000 / 66)
    sjmp beep

beep_f2_short:
    mov  Temp3, #45
    mov  Temp4, #(2000 / 45)
    sjmp beep

beep:
    mov  A, Beep_Strength
    jnz  beep_start                     ; Start if beep strength is not 0
    ret

beep_start:
    mov  Temp2, #2

beep_on_off:
    clr  A
    B_Com_Fet_Off                       ; B com FET off
    djnz ACC, $                         ; Allow some time after com FET is turned off
    B_Pwm_Fet_On                        ; B pwm FET on (in order to charge the driver of the B com FET)
    djnz ACC, $                         ; Let the pwm FET be turned on a while
    B_Pwm_Fet_Off                       ; B pwm FET off again
    djnz ACC, $                         ; Allow some time after pwm FET is turned off
    B_Com_Fet_On                        ; B com FET on
    djnz ACC, $                         ; Allow some time after com FET is turned on

    mov  A, Temp2                       ; Turn on pwm FET
    jb   ACC.0, beep_a_pwm_on
    A_Pwm_Fet_On
beep_a_pwm_on:
    jnb  ACC.0, beep_c_pwm_on
    C_Pwm_Fet_On
beep_c_pwm_on:

    mov  A, Beep_Strength               ; On time according to beep strength
    djnz ACC, $

    mov  A, Temp2                       ; Turn off pwm FET
    jb   ACC.0, beep_a_pwm_off
    A_Pwm_Fet_Off
beep_a_pwm_off:
    jnb  ACC.0, beep_c_pwm_off
    C_Pwm_Fet_Off
beep_c_pwm_off:

    mov  A, #150                        ; Off for 25 us
    djnz ACC, $

    djnz Temp2, beep_on_off             ; Toggle next pwm FET

    mov  A, Temp3
beep_off:                               ; Fets off loop
    mov  Temp1, #200
    djnz Temp1, $
    djnz ACC, beep_off                  ; Off time according to beep frequency

    djnz Temp4, beep_start              ; Number of beep pulses (duration)

    B_Com_Fet_Off
    ret

; Beep sequences
beep_signal_lost:
    call beep_f1
    call beep_f2
    call beep_f3
    ret

beep_enter_bootloader:
    call beep_f2_short
    call beep_f1
    ret

beep_motor_stalled:
    call beep_f3
    call beep_f2
    call beep_f1
    ret

beep_safety_no_arm:
    call    beep_f2_short
    call    beep_f1_short
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Beacon beep
;
; Beep with beacon strength.
;
; Requirements:
; - Interrupts must be disabled
; - FETs must be turned off
; - Beep tone 1-5 in Temp1
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
beacon_beep:
    mov  Temp2, #Pgm_Beacon_Strength    ; Set beacon beep strength
    mov  Beep_Strength, @Temp2

    cjne Temp1, #CMD_BEEP_1, beacon_beep2
    call beep_f1
    sjmp beacon_beep_exit

beacon_beep2:
    cjne Temp1, #CMD_BEEP_2, beacon_beep3
    call beep_f2
    sjmp beacon_beep_exit

beacon_beep3:
    cjne Temp1, #CMD_BEEP_3, beacon_beep4
    call beep_f3
    sjmp beacon_beep_exit

beacon_beep4:
    cjne Temp1, #CMD_BEEP_4, beacon_beep5
    call beep_f4
    sjmp beacon_beep_exit

beacon_beep5:
    call beep_f5

beacon_beep_exit:
    mov  Temp2, #Pgm_Beep_Strength      ; Set normal beep strength
    mov  Beep_Strength, @Temp2
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Play beep melody
;
; [BlueGill S3 flash trim] The startup beep MELODY player is removed here to free app
; CODE-segment space for the S3 sine<->6-step crossover "catch a spinning rotor" machinery
; (the 48 kHz build otherwise overflows the ?CO?BLUEJAY?20 0x19FD ceiling by 9 bytes). The
; startup jingle is non-essential on this board: BlueGill uses reliable DShot bidirectional
; telemetry (not audio) for identification/health, and all FUNCTIONAL beeps below
; (RC-detect, arm, beacon locate, signal-lost, motor-stalled, bootloader) are UNCHANGED.
; The stub keeps the symbol so the single caller (Bluejay.asm) links regardless; the
; Eep_Pgm_Beep_Melody table stays at CSEG_MELODY (fixed segment, no app-CSEG cost).
;
; Requirements: interrupts disabled, FETs off (unchanged; the stub is trivially safe).
;**** **** **** **** **** **** **** **** **** **** **** **** ****
play_beep_melody:
    ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; LED control
;
; Controls LEDs
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
led_control:
    mov  Temp1, #Pgm_LED_Control
    mov  A, @Temp1
    mov  Temp2, A
    anl  A, #03h
    Set_LED_0
    jnz  led_0_done
    Clear_LED_0

led_0_done:
    mov  A, Temp2
    anl  A, #0Ch
    Set_LED_1
    jnz  led_1_done
    Clear_LED_1

led_1_done:
    mov  A, Temp2
    anl  A, #030h
    Set_LED_2
    jnz  led_2_done
    Clear_LED_2

led_2_done:
    mov  A, Temp2
    anl  A, #0C0h
    Set_LED_3
    jnz  led_3_done
    Clear_LED_3

led_3_done:
    ret
