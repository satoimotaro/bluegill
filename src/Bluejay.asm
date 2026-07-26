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
; Bluejay is a fork of BLHeli_S <https://github.com/bitdump/BLHeli> by Steffen Skaug.
;
; The input signal can be DShot with rates: DShot150, DShot300 and DShot600.
;
; This file is best viewed with tab width set to 5.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Master clock is internal 24MHz oscillator (or 48MHz, for which the times below are halved)
; Although 24/48 are used in the code, the exact clock frequencies are 24.5MHz or 49.0 MHz
; Timer0 (41.67ns counts) always counts up and is used for
; - RC pulse measurement
; - DShot telemetry pulse timing
; Timer1 (41.67ns counts) always counts up and is used for
; - DShot frame sync detection
; Timer2 (500ns counts) always counts up and is used for
; - RC pulse timeout counts and commutation times
; Timer3 (500ns counts) always counts up and is used for
; - Commutation timeouts
; PCA0 (41.67ns counts) always counts up and is used for
; - Hardware PWM generation
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Motor control:
; - Brushless motor control with 6 states for each electrical 360 degrees
; - An advance timing of 0deg has zero cross 30deg after one commutation and 30deg before the next
; - Timing advance in this implementation is set to 15deg nominally
; - Motor pwm is always damped light (aka complementary pwm, regenerative braking)
; Motor sequence starting from zero crossing:
; - Timer wait: Wt_Comm            15deg    ; Time to wait from zero cross to actual commutation
; - Timer wait: Wt_Advance         15deg    ; Time to wait for timing advance. Nominal commutation point is after this
; - Timer wait: Wt_Zc_Scan         7.5deg   ; Time to wait before looking for zero cross
; - Scan for zero cross            22.5deg  ; Nominal, with some motor variations
;
; Motor startup:
; There is a startup phase and an initial run phase, before normal bemf commutation run begins.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Legend:
; RX            Receive/transmit pin
; Am, Bm, Cm    Comparator inputs for BEMF
; Vn            Common Comparator input
; Ap, Bp, Cp    PWM pins
; Ac, Bc, Cc    Complementary PWM pins
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

$include (Modules\Enums.asm)

; List of enumerated supported ESCs
;                                         PORT 0                   |  PORT 1                   |  PWM    COM    PWM    LED
;                                         P0 P1 P2 P3 P4 P5 P6 P7  |  P0 P1 P2 P3 P4 P5 P6 P7  |  inv    inv    side    n
;                                         -----------------------  |  -----------------------  |  -------------------------
IF MCU_TYPE == MCU_BB1 or MCU_TYPE == MCU_BB2
    A_ EQU 1                            ; Vn Am Bm Cm __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     no     high   _
    B_ EQU 2                            ; Vn Am Bm Cm __ RX __ __  |  Cc Cp Bc Bp Ac Ap __ __  |  no     no     high   _
    C_ EQU 3                            ; RX __ Vn Am Bm Cm Ap Ac  |  Bp Bc Cp Cc __ __ __ __  |  no     no     high   _
    D_ EQU 4                            ; Bm Cm Am Vn __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     yes    high   _
    E_ EQU 5                            ; Vn Am Bm Cm __ RX L0 L1  |  Ap Ac Bp Bc Cp Cc L2 __  |  no     no     high   3 Pinout like A, with LEDs
    F_ EQU 6                            ; Vn Cm Bm Am __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     no     high   _
    G_ EQU 7                            ; Bm Cm Am Vn __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     no     high   _ Pinout like D, but non-inverted com FETs
    H_ EQU 8                            ; Cm Vn Bm Am __ __ __ RX  |  Cc Bc Ac __ Cp Bp Ap __  |  no     no     high   _
    I_ EQU 9                            ; Vn Am Bm Cm __ RX __ __  |  Cp Bp Ap Cc Bc Ac __ __  |  no     no     high   _
    J_ EQU 10                           ; Am Cm Bm Vn RX L0 L1 L2  |  Ap Bp Cp Ac Bc Cc __ __  |  no     no     high   3
    K_ EQU 11                           ; RX Am Vn Bm __ Cm __ __  |  Ac Bc Cc Cp Bp Ap __ __  |  no     yes    high   _
    L_ EQU 12                           ; Cm Bm Am Vn __ RX __ __  |  Cp Bp Ap Cc Bc Ac __ __  |  no     no     high   _
    M_ EQU 13                           ; __ __ L0 RX Bm Vn Cm Am  |  __ Ap Bp Cp Ac Bc Cc __  |  no     no     high   1
    N_ EQU 14                           ; Vn Am Bm Cm __ RX __ __  |  Ac Ap Bc Bp Cc Cp __ __  |  no     no     high   _
    O_ EQU 15                           ; Bm Cm Am Vn __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     yes    low    _ Pinout Like D, but low side pwm
    P_ EQU 16                           ; __ Cm Bm Vn Am RX __ __  |  __ Ap Bp Cp Ac Bc Cc __  |  no     no     high   _
    Q_ EQU 17                           ; __ RX __ L0 L1 Ap Bp Cp  |  Ac Bc Cc Vn Cm Bm Am __  |  no     no     high   2
    R_ EQU 18                           ; Vn Am Bm Cm __ RX __ __  |  Cp Bp Ap Cc Bc Ac __ __  |  no     no     high   _ Same as I
    S_ EQU 19                           ; Bm Cm Am Vn __ RX __ __  |  Ac Ap Bc Bp Cc Cp __ __  |  no     no     high   _
    T_ EQU 20                           ; __ Cm Vn Bm __ Am __ RX  |  Cc Bc Ac Ap Bp Cp __ __  |  no     no     high   _
    U_ EQU 21                           ; L2 L1 L0 RX Bm Vn Cm Am  |  __ Ap Bp Cp Ac Bc Cc __  |  no     no     high   3 Pinout like M, with 3 LEDs
    V_ EQU 22                           ; Am Bm Vn Cm __ RX __ Cc  |  Cp Bc __ __ Bp Ac Ap __  |  no     no     high   _
    W_ EQU 23                           ; __ __ Am Vn __ Bm Cm RX  |  __ __ __ __ Cp Bp Ap __  |  n/a    n/a    high   _ Tristate gate driver
    X_ EQU 24
    Y_ EQU 25
    Z_ EQU 26                           ; Bm Cm Am Vn __ RX __ __  |  Ac Ap Bc Bp Cc Cp __ __  |  yes    no     high   _ Pinout like S, but inverted pwm FETs

    ; Two letter layouts start here. Preferably the first letter is the base
    ; layout and the second letter is the variation in alphabetical order.
    OA_ EQU 27                          ; Bm Cm Am Vn __ RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     yes    low    _ Pinout Like O, but open drain instead of push-pull COM FETs
ENDIF

; BB51 - Required
IF MCU_TYPE == MCU_BB51
    A_ EQU 1                            ; __ Bm Cm Am Vn RX __ __  |  Ap Ac Bp Bc Cp Cc __ __  |  no     no     low    _
    B_ EQU 2                            ; __ Bm Cm Am Vn RX __ __  |  Ac Ap Bc Bp Cc Cp __ __  |  no     yes    high   _
    C_ EQU 3                            ; __ Bm Cm Am Vn RX __ __  |  Ac Ap Bc Bp Cc Cp __ __  |  yes    yes    high   _
ENDIF

; Select the port mapping to use (or unselect all for use with external batch compile file)
;ESCNO            EQU    A_

; Select the MCU type (or unselect for use with external batch compile file)
;MCU_TYPE        EQU    0    ; BB1
;MCU_TYPE        EQU    1    ; BB2
;MCU_TYPE        EQU    2    ; BB51

; Select the FET dead time (or unselect for use with external batch compile file)
;DEADTIME            EQU    15    ; 20.4ns per step

; Select the pwm frequency (or unselect for use with external batch compile file)
;PWM_FREQ            EQU    0    ; 0=24, 1=48, 2=96 kHz

PWM_CENTERED EQU DEADTIME > 0           ; Use center aligned pwm on ESCs with dead time

IF MCU_TYPE == MCU_BB1
    IS_MCU_48MHZ EQU 0
ELSE
    IS_MCU_48MHZ EQU 1
ENDIF

IF PWM_FREQ == PWM_24 or PWM_FREQ == PWM_48 or PWM_FREQ == PWM_96
    ; Number of bits in pwm high byte
    PWM_BITS_H EQU (2 + IS_MCU_48MHZ - PWM_CENTERED - PWM_FREQ)
ENDIF

$include (Modules\McuOffsets.asm)
$include (Modules\Codespace.asm)
$include (Modules\Common.asm)
$include (Modules\Macros.asm)

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Programming defaults
;**** **** **** **** **** **** **** **** **** **** **** **** ****
DEFAULT_PGM_RPM_POWER_SLOPE EQU 9       ; 0=Off,1..13 (Power limit factor in relation to rpm)
DEFAULT_PGM_COMM_TIMING EQU 4           ; 1=Low 2=MediumLow 3=Medium 4=MediumHigh 5=High
DEFAULT_PGM_DEMAG_COMP EQU 2            ; 1=Disabled 2=Low 3=High
DEFAULT_PGM_DIRECTION EQU 1             ; 1=Normal 2=Reversed 3=Bidir 4=Bidir rev
DEFAULT_PGM_BEEP_STRENGTH EQU 40        ; 0..255 (BLHeli_S is 1..255)
DEFAULT_PGM_BEACON_STRENGTH EQU 80      ; 0..255
DEFAULT_PGM_BEACON_DELAY EQU 4          ; 1=1m 2=2m 3=5m 4=10m 5=Infinite
DEFAULT_PGM_ENABLE_TEMP_PROT EQU 0      ; 0=Disabled 1=80C 2=90C 3=100C 4=110C 5=120C 6=130C 7=140C

DEFAULT_PGM_POWER_RATING EQU 2          ; 1=1S,2=2S+

DEFAULT_PGM_BRAKE_ON_STOP EQU 0         ; 1=Enabled 0=Disabled
DEFAULT_PGM_LED_CONTROL EQU 0           ; Byte for LED control. 2 bits per LED,0=Off,1=On

DEFAULT_PGM_STARTUP_POWER_MIN EQU 21    ; 0..255 => (1000..1125 Throttle): value * (1000 / 2047) + 1000
DEFAULT_PGM_STARTUP_BEEP EQU 1          ; 0=Short beep,1=Melody

DEFAULT_PGM_STARTUP_POWER_MAX EQU 5     ; 0..255 => (1000..2000 Throttle): Maximum startup power
DEFAULT_PGM_BRAKING_STRENGTH EQU 255    ; 0..255 => 0..100 % Braking

DEFAULT_PGM_SAFETY_ARM EQU 0            ; EDT safety arm is disabled by default

; BlueGill added parameters (all default OFF => byte-identical stock behavior)
DEFAULT_PGM_COMM_TIMING_ANGLE EQU 0     ; 0=Off(use 1..5 preset),1..17 = 0..30 deg advance (1.875 deg/step)
DEFAULT_PGM_MAX_ERPM EQU 0              ; 0=Off, else eRPM cap in units of 1000 eRPM
DEFAULT_PGM_LOWSPEED_DAMPING EQU 0      ; 0=Off, else braking strength used below low-speed threshold

; BlueGill low-speed damping engage/release thresholds (Comm_Period4x_H, larger = slower)
LOWSPEED_DAMPING_THR EQU 020h           ; Engage damping when Comm_Period4x_H >= this (~1430 mech rpm @7pp)
LOWSPEED_DAMPING_REL EQU 01Ch           ; Release when Comm_Period4x_H < this (hysteresis)

; BlueGill S1 forced-commutation stepper defaults (all timid => cautious bench ramp)
DEFAULT_PGM_SINE_MODE EQU 0            ; 0=Off (stock 6-step/BEMF), 1=forced-commutation stepper
DEFAULT_PGM_SINE_HOLD_AMP EQU 8       ; ~3% duty zero-speed holding amplitude (8-bit throttle-equiv)
DEFAULT_PGM_SINE_AMP_MAX EQU 20      ; ~8% duty V/f amplitude ceiling
DEFAULT_PGM_SINE_RAMP EQU 16          ; speed slew rate (inc-LSB/tick; ~1 s 0->full-scale)

; BlueGill S3 sine<->BEMF-6-step crossover thresholds (both default OFF => no crossover;
; modes 0/1/2 byte-identical). Cross_Up in Sine_Inc_H units (~39.06 eRPM/unit, >= = faster);
; Cross_Dn in Comm_Period4x_H units (~312500 eRPM/unit INVERSE, >= = slower).
DEFAULT_PGM_SINE_CROSS_UP EQU 0        ; 0=Off; forced-sine -> BEMF up-handoff threshold (0x32)
DEFAULT_PGM_SINE_CROSS_DN EQU 0        ; 0=Off; BEMF -> forced-sine down-handoff threshold (0x33)

; BlueGill S1 stepper fixed-point constants. MUST MATCH tools/sim/sine_drive_model.py.
; Timer2 runs at SYSCLK/12 = 4 MHz during run (48 MHz core on BB21), so a tick of
; SINE_TICK_T2 = 4000 Timer2 ticks = 1.000 ms (1 kHz control rate).
SINE_TICK_T2 EQU 4000                  ; Timer2 ticks per control tick (1.000 ms @ 4 MHz)
SINE_RCP_SHIFT EQU 3                    ; per-tick step-rate target = Sine_Rcp << 3 (x8)
SINE_HOLD_AMP_CLAMP EQU 40             ; decode clamp: hold amplitude ceiling
SINE_AMP_MAX_CLAMP EQU 60              ; decode clamp: V/f amplitude ceiling

; BlueGill S3: up-handoff debounce. Sine_Cross_Cnt must reach this (in matched-direction,
; at/above-threshold control ticks) before a forced-sine -> BEMF handoff is allowed. ~16 ms.
SINE_CROSS_DEBOUNCE EQU 16

; BlueGill S3: the up-handoff seeds Comm_Period4x = Sine_Step_Ticks * 2000 measured over a
; 4-SECTOR window (reset entering sector 3, read entering sector 1). The window must fall in
; [MIN,MAX] whole 1 kHz ticks or the handoff is refused (rotor stays safely in forced sine):
;  * TICKS_MIN caps the ONE-SHOT seed quantization: 1/9 ~= 11% worst-case relative error, so a
;    too-fast crossover config can never silently seed BEMF with a badly-wrong period.
;  * TICKS_MAX is the BEMF speed floor (~20000/30 ~= 667... i.e. 40000/30 ~= 1333 eRPM, just
;    above the stock 6-step floor) and also keeps ticks*2000 within 16 bits.
SINE_CROSS_TICKS_MIN EQU 9
SINE_CROSS_TICKS_MAX EQU 30

; BlueGill S3: down-handoff phase seed. The 6-step electrical state at a BEMF->sine
; down-handoff is deterministic (always program-state 1: run6_check_speed fires right
; after comm6_comm1), which maps to this sine sector for BOTH commanded directions (the
; comm* routines' Flag_Motor_Dir_Rev branch absorbs the physical mirror -- see PLAN A.1).
; TUNABLE: the bench sector scan confirms the constant (scan order 6, 2, 5, 3, 4).
SINE_DN_SEED_SECTOR EQU 6

; BlueGill S2: free PCA module 2 (CEX2) auto-reload write macros. The vendor Base.inc
; only defines module-0 (POWER) and module-1 (DAMP) macros; S2 adds a THIRD modulated
; phase on the otherwise-unused module 2. Kept here (NOT in vendor/) per overlay
; discipline. PCA0CPL2/PCA0CPH2 are the module-2 auto-reload low/high registers.
Set_Power2_Pwm_Reg_L MACRO value
    mov  PCA0CPL2, value
ENDM
Set_Power2_Pwm_Reg_H MACRO value
    mov  PCA0CPH2, value
ENDM

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Temporary register definitions
;**** **** **** **** **** **** **** **** **** **** **** **** ****
Temp1 EQU R0
Temp2 EQU R1
Temp3 EQU R2
Temp4 EQU R3
Temp5 EQU R4
Temp6 EQU R5
Temp7 EQU R6
Temp8 EQU R7

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; RAM definitions
; Bit-addressable data segment
;**** **** **** **** **** **** **** **** **** **** **** **** ****
DSEG AT 20h
Bit_Access: DS 1                        ; MUST BE AT THIS ADDRESS. Variable at bit accessible address (for non interrupt routines)
Bit_Access_Int: DS 1                    ; Variable at bit accessible address (for interrupts)

Flags0: DS 1                            ; State flags. Reset upon motor_start
    Flag_Startup_Phase BIT Flags0.0     ; Set when in startup phase
    Flag_Initial_Run_Phase BIT Flags0.1 ; Set when in initial run phase (or startup phase),before synchronized run is achieved.
    Flag_Motor_Dir_Rev BIT Flags0.2     ; Set if the current spinning direction is reversed
    Flag_Demag_Notify BIT Flags0.3      ; Set when motor demag has been detected but still not notified
    Flag_Desync_Notify BIT Flags0.4     ; Set when motor desync has been detected but still not notified
    Flag_Stall_Notify BIT Flags0.5      ; Set when motor stall detected but still not notified

Flags1: DS 1                            ; State flags. Reset upon motor_start
    Flag_Timer3_Pending BIT Flags1.0    ; Timer3 pending flag
    Flag_Demag_Detected BIT Flags1.1    ; Set when excessive demag time is detected
    Flag_Comp_Timed_Out BIT Flags1.2    ; Set when comparator reading timed out
    Flag_Motor_Running BIT Flags1.3
    Flag_Motor_Started BIT Flags1.4     ; Set when motor is started
    Flag_Dir_Change_Brake BIT Flags1.5  ; Set when braking before direction change in case of bidirectional operation
    Flag_High_Rpm BIT Flags1.6          ; Set when motor rpm is high (Comm_Period4x_H less than 2)

Flags2: DS 1                            ; State flags. NOT reset upon motor_start
    ; BIT    Flags2.0
    Flag_Pgm_Dir_Rev BIT Flags2.1       ; Set if the programmed direction is reversed
    Flag_Pgm_Bidir BIT Flags2.2         ; Set if the programmed control mode is bidirectional operation
    Flag_16ms_Elapsed BIT Flags2.3      ; Set when timer2 interrupt is triggered
    Flag_Ext_Tele BIT Flags2.4          ; Set if Extended DHOT telemetry is enabled
    Flag_Rcp_Stop BIT Flags2.5          ; Set if the RC pulse value is zero or if timeout occurs
    Flag_Rcp_Dir_Rev BIT Flags2.6       ; RC pulse direction in bidirectional mode
    Flag_Rcp_DShot_Inverted BIT Flags2.7 ; DShot RC pulse input is inverted (and supports telemetry)

Flags3: DS 1                            ; State flags. NOT reset upon motor_start
    Flag_Telemetry_Pending BIT Flags3.0 ; DShot telemetry data packet is ready to be sent
    Flag_Had_Signal BIT Flags3.1        ; Used to detect reset after having had a valid signal
    Flag_User_Reverse_Requested BIT Flags3.2 ; It is set when user request to reverse motors in turtle mode
    Flag_Sine_Mode BIT Flags3.3         ; BlueGill S1: forced-commutation stepper mode enabled (from param, NOT reset on motor_start)
    Flag_Sine_Run BIT Flags3.4          ; BlueGill S1: sine_run loop is active (single-duty-writer gate for t1_int; cleared in exit_run_mode)
    Flag_Sine_Micro BIT Flags3.5        ; BlueGill S2: min-clamp two-phase micro-stepping (Pgm_Sine_Mode==2; from param, NOT reset on motor_start)
    Flag_Sine_Handoff BIT Flags3.6      ; BlueGill S3: BEMF->sine down-handoff pending. Set in run6_check_speed, consumed+cleared at sine_run entry; MUST be in Flags3 (not cleared by motor_start's Flags0/1 wipe)
    Flag_Cross_Up_Armed BIT Flags3.7    ; BlueGill S3: forced-sine->BEMF up-handoff armed this tick. Set in sine_cross_update, ret's up the call chain, consumed by sine_run_loop -> ljmp sine_cross_up at baseline (no manual SP surgery). Cleared at sine_run entry.


Tlm_Data_L: DS 1                        ; DShot telemetry data (lo byte)
Tlm_Data_H: DS 1                        ; DShot telemetry data (hi byte)
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Direct addressing data segment
;**** **** **** **** **** **** **** **** **** **** **** **** ****
DSEG AT 30h
Rcp_Outside_Range_Cnt: DS 1             ; RC pulse outside range counter (incrementing)
Rcp_Timeout_Cntd: DS 1                  ; RC pulse timeout counter (decrementing)
Rcp_Stop_Cnt: DS 1                      ; Counter for RC pulses below stop value

Beacon_Delay_Cnt: DS 1                  ; Counter to trigger beacon during wait for start
Startup_Cnt: DS 1                       ; Startup phase commutations counter (incrementing)
Startup_Zc_Timeout_Cntd: DS 1           ; Startup zero cross timeout counter (decrementing)
Initial_Run_Rot_Cntd: DS 1              ; Initial run rotations counter (decrementing)
Startup_Stall_Cnt: DS 1                 ; Counts start/run attempts that resulted in stall. Reset upon a proper stop
Demag_Detected_Metric: DS 1             ; Metric used to gauge demag event frequency
Demag_Detected_Metric_Max: DS 1         ; Metric used to gauge demag event frequency
Demag_Pwr_Off_Thresh: DS 1              ; Metric threshold above which power is cut
Low_Rpm_Pwr_Slope: DS 1                 ; Sets the slope of power increase for low rpm
Timer2_X: DS 1                          ; Timer2 extended byte
Prev_Comm_L: DS 1                       ; Previous commutation Timer2 timestamp (lo byte)
Prev_Comm_H: DS 1                       ; Previous commutation Timer2 timestamp (hi byte)
Prev_Comm_X: DS 1                       ; Previous commutation Timer2 timestamp (ext byte)
Prev_Prev_Comm_L: DS 1                  ; Pre-previous commutation Timer2 timestamp (lo byte)
Prev_Prev_Comm_H: DS 1                  ; Pre-previous commutation Timer2 timestamp (hi byte)
Comm_Period4x_L: DS 1                   ; Timer2 ticks between the last 4 commutations (lo byte)
Comm_Period4x_H: DS 1                   ; Timer2 ticks between the last 4 commutations (hi byte)
Comparator_Read_Cnt: DS 1               ; Number of comparator reads done
Wt_Adv_Start_L: DS 1                    ; Timer3 start point for commutation advance timing (lo byte)
Wt_Adv_Start_H: DS 1                    ; Timer3 start point for commutation advance timing (hi byte)
Wt_Zc_Scan_Start_L: DS 1                ; Timer3 start point from commutation to zero cross scan (lo byte)
Wt_Zc_Scan_Start_H: DS 1                ; Timer3 start point from commutation to zero cross scan (hi byte)
Wt_Zc_Tout_Start_L: DS 1                ; Timer3 start point for zero cross scan timeout (lo byte)
Wt_Zc_Tout_Start_H: DS 1                ; Timer3 start point for zero cross scan timeout (hi byte)
Wt_Comm_Start_L: DS 1                   ; Timer3 start point from zero cross to commutation (lo byte)
Wt_Comm_Start_H: DS 1                   ; Timer3 start point from zero cross to commutation (hi byte)
Pwm_Limit: DS 1                         ; Maximum allowed pwm (8-bit)
Pwm_Limit_By_Rpm: DS 1                  ; Maximum allowed pwm for low or high rpm (8-bit)
Pwm_Limit_Beg: DS 1                     ; Initial pwm limit (8-bit)
Pwm_Braking_L: DS 1                     ; Max Braking pwm (lo byte)
Pwm_Braking_H: DS 1                     ; Max Braking pwm (hi byte)
Temp_Prot_Limit: DS 1                   ; Temperature protection limit
Temp_Pwm_Level_Setpoint: DS 1           ; PWM level setpoint
Beep_Strength: DS 1                     ; Strength of beeps
Flash_Key_1: DS 1                       ; Flash key one
Flash_Key_2: DS 1                       ; Flash key two
DShot_Pwm_Thr: DS 1                     ; DShot pulse width threshold value (Timer0 ticks)
DShot_Timer_Preset: DS 1                ; DShot timer preset for frame sync detection (Timer1 lo byte)
DShot_Frame_Start_L: DS 1               ; DShot frame start timestamp (Timer2 lo byte)
DShot_Frame_Start_H: DS 1               ; DShot frame start timestamp (Timer2 hi byte)
DShot_Frame_Length_Thr: DS 1            ; DShot frame length criteria (Timer2 ticks)
DShot_Cmd: DS 1                         ; DShot command
DShot_Cmd_Cnt: DS 1                     ; DShot command count
; Pulse durations for GCR encoding DShot telemetry data
DShot_GCR_Pulse_Time_1: DS 1            ; Encodes binary: 1
DShot_GCR_Pulse_Time_2: DS 1            ; Encodes binary: 01
DShot_GCR_Pulse_Time_3: DS 1            ; Encodes binary: 001

DShot_GCR_Pulse_Time_1_Tmp: DS 1
DShot_GCR_Pulse_Time_2_Tmp: DS 1
DShot_GCR_Pulse_Time_3_Tmp: DS 1
DShot_GCR_Start_Delay: DS 1
Ext_Telemetry_L: DS 1                   ; Extended telemetry data to be sent
Ext_Telemetry_H: DS 1
Scheduler_Counter: DS 1                 ; Scheduler Heartbeat

; BlueGill decoded run-time variables (free space 0x68..0x7F)
Pwm_Limit_By_Erpm: DS 1                 ; eRPM cap governor limit (8-bit), 255 = no cap
Comm_Timing_Angle_Adj: DS 1             ; Decoded direct timing angle: 0=off, else 1..17
Max_Erpm_Cap_L: DS 1                    ; eRPM cap as Comm_Period4x threshold (lo)
Max_Erpm_Cap_H: DS 1                    ; eRPM cap as Comm_Period4x threshold (hi)
Max_Erpm_Rel_L: DS 1                    ; eRPM cap release threshold w/ hysteresis (lo)
Max_Erpm_Rel_H: DS 1                    ; eRPM cap release threshold w/ hysteresis (hi)
LowSpeed_Damping_State: DS 1            ; 0 = programmed braking, 1 = damping override active

; BlueGill S1 forced-commutation stepper state (direct-addressed, free RAM 0x6F..0x78)
Sine_Rcp_L: DS 1                        ; 11-bit throttle magnitude snapshot from t1_int (lo)
Sine_Rcp_H: DS 1                        ; 11-bit throttle magnitude snapshot from t1_int (hi, 3-bit)
Sine_Sector: DS 1                       ; Current commutation sector 1..6 (forward order only)
Sine_Frac_L: DS 1                       ; Angle accumulator (lo)
Sine_Frac_H: DS 1                       ; Angle accumulator (hi) - advance one sector on 16-bit overflow
Sine_Inc_L: DS 1                        ; Slewed step-rate magnitude added per control tick (lo)
Sine_Inc_H: DS 1                        ; Slewed step-rate magnitude added per control tick (hi)
Sine_Amp: DS 1                          ; Current V/f amplitude (8-bit throttle-equivalent, <= Pwm_Limit)
Sine_T2_L: DS 1                         ; Timer2 pacing baseline snapshot (lo)
Sine_T2_H: DS 1                         ; Timer2 pacing baseline snapshot (hi)
; BlueGill S2 min-clamp two-phase micro-stepping state (only used when Flag_Sine_Micro)
Sine_Seg: DS 1                          ; Applied clamp phase 0=A/1=B/2=C (0FFh = none yet -> force remux)
Sine_G: DS 1                            ; This-tick electrical position g_eff (0..191, 192 microsteps/erev)
Sine_D_Pair: DS 1                       ; This-tick duty (8-bit) for the pair phase (PCA modules 0/1)
Sine_D_Second: DS 1                     ; This-tick duty (8-bit) for the second phase (PCA module 2 / CEX2)
; BlueGill S3 crossover counters (hot, direct-addressed; tail of the 0x30 DSEG, 0x7D..0x7E)
Sine_Step_Ticks: DS 1                   ; Control ticks in the current 4-sector window (reset entering sector 3, spans sectors 3..6); read at sector-1 entry -> up-handoff Comm_Period4x seed (0x7D)
Sine_Cross_Cnt: DS 1                     ; Up-handoff debounce: inc-sat while at/above Cross_Up in matched direction, reset otherwise (0x7E)
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Indirect addressing data segments
;**** **** **** **** **** **** **** **** **** **** **** **** ****
ISEG AT 080h                            ; The variables below must be in this sequence
_Pgm_Gov_P_Gain: DS 1                   ;
Pgm_Startup_Power_Min: DS 1             ; Minimum power during startup phase
Pgm_Startup_Beep: DS 1                  ; Startup beep melody on/off
_Pgm_Dithering: DS 1                    ; Enable PWM dithering
Pgm_Startup_Power_Max: DS 1             ; Maximum power (limit) during startup (and starting initial run phase)
_Pgm_Rampup_Slope: DS 1                 ;
Pgm_Rpm_Power_Slope: DS 1               ; Low RPM power protection slope (factor)
Pgm_Pwm_Freq: DS 1                      ; PWM frequency (temporary method for display)
Pgm_Direction: DS 1                     ; Rotation direction
_Pgm_Input_Pol: DS 1                    ; Input PWM polarity
Initialized_L_Dummy: DS 1               ; Place holder
Initialized_H_Dummy: DS 1               ; Place holder
_Pgm_Enable_TX_Program: DS 1            ; Enable/disable value for TX programming
Pgm_Braking_Strength: DS 1              ; Set maximum braking strength (complementary pwm)
_Pgm_Gov_Setup_Target: DS 1             ; Main governor setup target
_Pgm_Startup_Rpm: DS 1                  ; Startup RPM
_Pgm_Startup_Accel: DS 1                ; Startup acceleration
_Pgm_Volt_Comp: DS 1                    ; Voltage comp
Pgm_Comm_Timing: DS 1                   ; Commutation timing
_Pgm_Damping_Force: DS 1                ; Damping force
_Pgm_Gov_Range: DS 1                    ; Governor range
_Pgm_Startup_Method: DS 1               ; Startup method
_Pgm_Min_Throttle: DS 1                 ; Minimum throttle
_Pgm_Max_Throttle: DS 1                 ; Maximum throttle
Pgm_Beep_Strength: DS 1                 ; Beep strength
Pgm_Beacon_Strength: DS 1               ; Beacon strength
Pgm_Beacon_Delay: DS 1                  ; Beacon delay
_Pgm_Throttle_Rate: DS 1                ; Throttle rate
Pgm_Demag_Comp: DS 1                    ; Demag compensation
_Pgm_BEC_Voltage_High: DS 1             ; BEC voltage
_Pgm_Center_Throttle: DS 1              ; Center throttle (in bidirectional mode)
_Pgm_Main_Spoolup_Time: DS 1            ; Main spoolup time
Pgm_Enable_Temp_Prot: DS 1              ; Temperature protection enable
_Pgm_Enable_Power_Prot: DS 1            ; Low RPM power protection enable
_Pgm_Enable_Pwm_Input: DS 1             ; Enable PWM input signal
_Pgm_Pwm_Dither: DS 1                   ; Output PWM dither
Pgm_Brake_On_Stop: DS 1                 ; Braking when throttle is zero
Pgm_LED_Control: DS 1                   ; LED control
Pgm_Power_Rating: DS 1                  ; Power rating
Pgm_Safety_Arm: DS  1                   ; Various flag settings: bit 0 is require edt enable to arm
Pgm_Comm_Timing_Angle: DS 1             ; BlueGill: direct commutation timing angle (0=off, 1..17)
Pgm_Max_Erpm: DS 1                      ; BlueGill: eRPM cap (0=off, units of 1000 eRPM)
Pgm_LowSpeed_Damping: DS 1              ; BlueGill: low-speed damping braking strength (0=off)
Pgm_Sine_Mode: DS 1                     ; BlueGill S1: forced-commutation stepper mode (0=off) (0x2E)
Pgm_Sine_Hold_Amp: DS 1                 ; BlueGill S1: zero-speed holding amplitude, 8-bit throttle-equiv (0x2F)
Pgm_Sine_Amp_Max: DS 1                  ; BlueGill S1: V/f amplitude ceiling, 8-bit throttle-equiv (0x30)
Pgm_Sine_Ramp: DS 1                     ; BlueGill S1: speed slew rate, inc-LSB per control tick (0x31)
; BlueGill S3 crossover params. MUST stay contiguous (0xAF, 0xB0) right after Pgm_Sine_Ramp
; and in the SAME order as the Eep_ list below: vendor Eeprom.asm walks _Pgm_Enable_TX_Program
; (0x8C) for EEPROM_B2_PARAMETERS_COUNT bytes in lockstep (inc DPTR/inc Temp1), so RAM addr and
; count must agree. 0x8C + 37 - 1 = 0xB0 = Pgm_Sine_Cross_Dn.
Pgm_Sine_Cross_Up: DS 1                 ; BlueGill S3: forced-sine -> BEMF up-handoff threshold (0x32, 0xAF)
Pgm_Sine_Cross_Dn: DS 1                 ; BlueGill S3: BEMF -> forced-sine down-handoff threshold (0x33, 0xB0)

ISEG AT 0B2h                            ; Moved from 0B0h: 0xAF/0xB0 now hold the two S3 params (0xB1 spare)
Stack: DS 16                            ; Reserved stack space (0xB2-0xC1; mov SP,#Stack is symbolic)

ISEG AT 0C2h                            ; Cold, indirect-only: down-handoff field-rate seed (0xC2-0xC3)
Sine_Inc_Seed_L: DS 1                   ; BlueGill S3: Sine_Inc seed = 2048000/Cross_Dn (down-handoff), lo
Sine_Inc_Seed_H: DS 1                   ; BlueGill S3: down-handoff Sine_Inc seed, hi

ISEG AT 0D0h
Temp_Storage: DS 48                     ; Temporary storage (internal memory)

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; EEPROM code segments
; A segment of the flash is used as "EEPROM", which is not available in SiLabs MCUs
;**** **** **** **** **** **** **** **** **** **** **** **** ****
CSEG AT CSEG_EEPROM
EEPROM_FW_MAIN_REVISION EQU 0           ; Main revision of the firmware
EEPROM_FW_SUB_REVISION EQU 21           ; Sub revision of the firmware
EEPROM_LAYOUT_REVISION EQU 226          ; Revision of the EEPROM layout (BlueGill fork marker; S1 sine + S3 crossover params)
EEPROM_B2_PARAMETERS_COUNT EQU 37       ; Number of parameters (28 stock + 3 BlueGill + 4 S1 sine + 2 S3 crossover)

Eep_FW_Main_Revision: DB EEPROM_FW_MAIN_REVISION ; EEPROM firmware main revision number
Eep_FW_Sub_Revision: DB EEPROM_FW_SUB_REVISION ; EEPROM firmware sub revision number
Eep_Layout_Revision: DB EEPROM_LAYOUT_REVISION ; EEPROM layout revision number
_Eep_Pgm_Gov_P_Gain: DB 0FFh
Eep_Pgm_Startup_Power_Min: DB DEFAULT_PGM_STARTUP_POWER_MIN
Eep_Pgm_Startup_Beep: DB DEFAULT_PGM_STARTUP_BEEP
_Eep_Pgm_Dithering: DB 0FFh
Eep_Pgm_Startup_Power_Max: DB DEFAULT_PGM_STARTUP_POWER_MAX
_Eep_Pgm_Rampup_Slope: DB 0FFh
Eep_Pgm_Rpm_Power_Slope: DB DEFAULT_PGM_RPM_POWER_SLOPE ; EEPROM copy of programmed rpm power slope (formerly startup power)
Eep_Pgm_Pwm_Freq: DB (24 SHL PWM_FREQ)  ; Temporary method for display
Eep_Pgm_Direction: DB DEFAULT_PGM_DIRECTION ; EEPROM copy of programmed rotation direction
_Eep__Pgm_Input_Pol: DB 0FFh
Eep_Initialized_L: DB 055h              ; EEPROM initialized signature (lo byte)
Eep_Initialized_H: DB 0AAh              ; EEPROM initialized signature (hi byte)
; EEPROM parameters block 2 (B2)
_Eep_Enable_TX_Program: DB 0FFh         ; EEPROM TX programming enable
Eep_Pgm_Braking_Strength: DB DEFAULT_PGM_BRAKING_STRENGTH
_Eep_Pgm_Gov_Setup_Target: DB 0FFh
_Eep_Pgm_Startup_Rpm: DB 0FFh
_Eep_Pgm_Startup_Accel: DB 0FFh
_Eep_Pgm_Volt_Comp: DB 0FFh
Eep_Pgm_Comm_Timing: DB DEFAULT_PGM_COMM_TIMING ; EEPROM copy of programmed commutation timing
_Eep_Pgm_Damping_Force: DB 0FFh
_Eep_Pgm_Gov_Range: DB 0FFh
_Eep_Pgm_Startup_Method: DB 0FFh
_Eep_Pgm_Min_Throttle: DB 0FFh          ; EEPROM copy of programmed minimum throttle
_Eep_Pgm_Max_Throttle: DB 0FFh          ; EEPROM copy of programmed minimum throttle
Eep_Pgm_Beep_Strength: DB DEFAULT_PGM_BEEP_STRENGTH ; EEPROM copy of programmed beep strength
Eep_Pgm_Beacon_Strength: DB DEFAULT_PGM_BEACON_STRENGTH ; EEPROM copy of programmed beacon strength
Eep_Pgm_Beacon_Delay: DB DEFAULT_PGM_BEACON_DELAY ; EEPROM copy of programmed beacon delay
_Eep_Pgm_Throttle_Rate: DB 0FFh
Eep_Pgm_Demag_Comp: DB DEFAULT_PGM_DEMAG_COMP ; EEPROM copy of programmed demag compensation
_Eep_Pgm_BEC_Voltage_High: DB 0FFh
_Eep_Pgm_Center_Throttle: DB 0FFh       ; EEPROM copy of programmed center throttle
_Eep_Pgm_Main_Spoolup_Time: DB 0FFh
Eep_Pgm_Temp_Prot_Enable: DB DEFAULT_PGM_ENABLE_TEMP_PROT ; EEPROM copy of programmed temperature protection enable
_Eep_Pgm_Enable_Power_Prot: DB 0FFh     ; EEPROM copy of programmed low rpm power protection enable
_Eep_Pgm_Enable_Pwm_Input: DB 0FFh
_Eep_Pgm_Pwm_Dither: DB 0FFh
Eep_Pgm_Brake_On_Stop: DB DEFAULT_PGM_BRAKE_ON_STOP ; EEPROM copy of programmed braking when throttle is zero
Eep_Pgm_LED_Control: DB DEFAULT_PGM_LED_CONTROL ; EEPROM copy of programmed LED control
Eep_Pgm_Power_Rating: DB DEFAULT_PGM_POWER_RATING ; EEPROM copy of programmed power rating
Eep_Pgm_Safety_Arm: DB DEFAULT_PGM_SAFETY_ARM ; Various flag settings: bit 0 is require edt enable to arm
Eep_Pgm_Comm_Timing_Angle: DB DEFAULT_PGM_COMM_TIMING_ANGLE ; BlueGill: direct commutation timing angle (0x2B)
Eep_Pgm_Max_Erpm: DB DEFAULT_PGM_MAX_ERPM ; BlueGill: eRPM cap (0x2C)
Eep_Pgm_LowSpeed_Damping: DB DEFAULT_PGM_LOWSPEED_DAMPING ; BlueGill: low-speed damping (0x2D)
Eep_Pgm_Sine_Mode: DB DEFAULT_PGM_SINE_MODE ; BlueGill S1: forced-commutation stepper mode (0x2E)
Eep_Pgm_Sine_Hold_Amp: DB DEFAULT_PGM_SINE_HOLD_AMP ; BlueGill S1: zero-speed holding amplitude (0x2F)
Eep_Pgm_Sine_Amp_Max: DB DEFAULT_PGM_SINE_AMP_MAX ; BlueGill S1: V/f amplitude ceiling (0x30)
Eep_Pgm_Sine_Ramp: DB DEFAULT_PGM_SINE_RAMP ; BlueGill S1: speed slew rate (0x31)
Eep_Pgm_Sine_Cross_Up: DB DEFAULT_PGM_SINE_CROSS_UP ; BlueGill S3: up-handoff threshold (0x32)
Eep_Pgm_Sine_Cross_Dn: DB DEFAULT_PGM_SINE_CROSS_DN ; BlueGill S3: down-handoff threshold (0x33)

Eep_Dummy: DB 0FFh                      ; EEPROM address for safety reason
CSEG AT CSEG_NAME
Eep_Name: DB "Bluejay (.0)    "         ; Name tag (16 Bytes)

CSEG AT CSEG_MELODY
; BlueGill startup jingle — "Under the Sea" hook (G G G E, then G G G C^) so a fresh
; BlueGill flash is identifiable by ear. Format: (pulses,pitch) note pairs after a 4-byte
; header; pitch is the pulse half-period (LOWER value = HIGHER note); (ms,0) = silent gap;
; trailing 0 = end. Pitches: G5=51, E5=61 (lower), C6=38 (higher). ~1.3 s total.
Eep_Pgm_Beep_Melody: DB 2,58,4,32,255,16,111,20,99,24,120,0,166,31,120,0,254,16,111,20,99,24,120,0,254,16,0

    Interrupt_Table_Definition          ; SiLabs interrupts
CSEG AT CSEG_APP                        ; Code segment after interrupt vectors

; Submodule includes
$include (Modules\Isrs.asm)
$include (Modules\Fx.asm)
$include (Modules\Power.asm)
$include (Modules\Scheduler.asm)
$include (Modules\Timing.asm)
$include (Modules\Commutation.asm)
$include (Modules\DShot.asm)
$include (Modules\Eeprom.asm)
$include (Modules\Settings.asm)
$include (Modules\SineMode.asm)         ; BlueGill S1 forced-commutation stepper mode

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Main program
;
; Main program entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
pgm_start:
    Lock_Flash
    mov  WDTCN, #0DEh                   ; Disable watchdog (WDT)
    mov  WDTCN, #0ADh
    mov  SP, #Stack                     ; Initialize stack (16 bytes of indirect RAM)
IF MCU_TYPE == MCU_BB1 or MCU_TYPE == MCU_BB2
    orl  VDM0CN, #080h                  ; Enable the VDD monitor
ENDIF
    mov  RSTSRC, #06h                   ; Set missing clock and VDD monitor as a reset source if not 1S capable
    mov  CLKSEL, #00h                   ; Set clock divider to 1 (Oscillator 0 at 24MHz)
    call switch_power_off
    ; Ports initialization
    mov  P0, #P0_INIT
    mov  P0MDIN, #P0_DIGITAL
    mov  P0MDOUT, #P0_PUSHPULL
    mov  P0, #P0_INIT
    mov  P0SKIP, #P0_SKIP
    mov  P1, #P1_INIT
    mov  P1MDIN, #P1_DIGITAL
    mov  P1MDOUT, #P1_PUSHPULL
    mov  P1, #P1_INIT
    mov  P1SKIP, #P1_SKIP
    mov  P2MDOUT, #P2_PUSHPULL
IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    ; Not available on BB1
    mov  SFRPAGE, #20h
    mov  P2MDIN, #P2_DIGITAL
IF MCU_TYPE == MCU_BB2
    ; Not available on BB51
    mov  P2SKIP, #P2_SKIP
ENDIF
    mov  SFRPAGE, #00h
ENDIF
    Initialize_Crossbar                 ; Initialize the crossbar and related functionality
    call switch_power_off               ; Switch power off again,after initializing ports

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Internal RAM
;
; EFM8 consists of 256 bytes of internal RAM of which the lower 128 bytes can be
; directly adressed and the upper portion (starting at 0x80) can only be
; indirectly accessed.
;
; NOTE: Upper portion of RAM and SFR use the same address space. RAM is accessed
;       indirectly. If you are directly accessing the upper space, you are - in
;       fact - addressing the SFR.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Clear internal RAM
;
; First the accumlator is cleared, then address is overflowed to 255 and content
; of addresses 255 - 0 is set to 0.
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
    clr  A                              ; Clear accumulator
    mov  Temp1, A                       ; Clear Temp1
clear_ram:
    mov  @Temp1, A                      ; Clear RAM address
    djnz Temp1, clear_ram               ; Decrement address and repeat

    call set_default_parameters         ; Set default programmed parameters
    call read_all_eeprom_parameters     ; Read all programmed parameters
    call decode_settings                ; Decode programmed settings

    ; Initializing beeps
    clr  IE_EA                          ; Disable interrupts explicitly
    call wait100ms                      ; Wait a bit to avoid audible resets if not properly powered
    ; [BlueGill S3 flash trim] startup beep MELODY removed (freed app-CSEG flash for the S3
    ; catch machinery; DShot telemetry replaces audio ID -- see Fx.asm play_beep_melody stub).
    call led_control                    ; Set LEDs to programmed values

    call wait100ms                      ; Wait for flight controller to get ready

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; No signal entry point
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_no_signal:
    clr  IE_EA                          ; Disable interrupts explicitly
    Lock_Flash
    call switch_power_off

IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    ; While not armed, all MCUs run at 24MHz clock frequency. After arming those
    ; MCUs that support it (BB2 & BB51) are switched to 48MHz clock frequency.
    Set_MCU_Clk_24MHz
ENDIF

    ; If input signal is high for about ~150ms, enter bootloader mode
    mov  Temp1, #9
    mov  Temp2, #0
    mov  Temp3, #0
input_high_check:
    jnb  RTX_BIT, bootloader_done       ; If low is detected, skip bootloader check
    djnz Temp3, input_high_check
    djnz Temp2, input_high_check
    djnz Temp1, input_high_check

    call beep_enter_bootloader

    ljmp CSEG_BOOT_START                ; Jump to bootloader

bootloader_done:
    ; If we had a signal before, reset the flag, beep, wait a bit and contiune
    ; with DSHOT setup. If we did not have a signal yet, continue with DSHOT
    ; setup straight away.
    jnb  Flag_Had_Signal, setup_dshot
    call beep_signal_lost

    ; Wait for flight controller to get ready
    call wait250ms
    call wait250ms
    call wait250ms
    clr  Flag_Had_Signal

setup_dshot:
    ; Setup timers for DShot
    mov  TCON, #51h                     ; Timer0/1 run and Int0 edge triggered
    mov  CKCON0, #01h                   ; Timer0/1 clock is system clock divided by 4 (for DShot150)
    mov  TMOD, #0AAh                    ; Timer0/1 set to 8-bits auto reload and gated by Int0/1
    mov  TH0, #0                        ; Auto reload value zero
    mov  TH1, #0

    mov  TMR2CN0, #04h                  ; Timer2 enabled (system clock divided by 12)
    mov  TMR3CN0, #04h                  ; Timer3 enabled (system clock divided by 12)

    Initialize_PCA                      ; Initialize PCA
    Set_Pwm_Polarity                    ; Set pwm polarity
    Enable_Power_Pwm_Module             ; Enable power pwm module
    Enable_Damp_Pwm_Module              ; Enable damping pwm module
    Initialize_Comparator               ; Initialize comparator
    Initialize_Adc                      ; Initialize ADC operation
    call wait1ms

    call detect_rcp_level               ; Detect RCP level (normal or inverted DShot)

    ; Route RCP according to detected DShot signal (normal or inverted)
    mov  IT01CF, #(80h + (RTX_PIN SHL 4) + RTX_PIN) ; Route RCP input to Int0/1,with Int1 inverted
    jnb  Flag_Rcp_DShot_Inverted, setup_dshot_clear_flags
    mov  IT01CF, #(08h + (RTX_PIN SHL 4) + RTX_PIN) ; Route RCP input to Int0/1,with Int0 inverted

setup_dshot_clear_flags:
    clr  Flag_Demag_Notify              ; Clear motor events
    clr  Flag_Desync_Notify
    clr  Flag_Stall_Notify
    clr  Flag_Telemetry_Pending         ; Clear DShot telemetry flag
    clr  Flag_Ext_Tele                  ; Clear extended telemetry enabled flag

    ; Setup interrupts
    mov  IE, #2Dh                       ; Enable Timer1/2 interrupts and Int0/1 interrupts
    mov  EIE1, #80h                     ; Enable Timer3 interrupts
    mov  IP, #03h                       ; High priority to Timer0 and Int0 interrupts

    setb IE_EA                          ; Enable all interrupts

; Setup variables for DShot150 (Only on 24MHz because frame length threshold cannot be scaled up)
IF MCU_TYPE == MCU_BB1
    mov  DShot_Timer_Preset, #-64       ; Load DShot sync timer preset (for DShot150)
    mov  DShot_Pwm_Thr, #8              ; Load DShot qualification pwm threshold (for DShot150)
    mov  DShot_Frame_Length_Thr, #160   ; Load DShot frame length criteria

    Set_DShot_Tlm_Bitrate 187500        ; = 5/4 * 150000

    ; Test whether signal is DShot150
    mov  Rcp_Outside_Range_Cnt, #10     ; Set out of range counter
    call wait100ms                      ; Wait for new RC pulse
    mov  A, Rcp_Outside_Range_Cnt       ; Check if pulses were accepted
    jz   arming_begin
ENDIF

    mov  CKCON0, #0Ch                   ; Timer0/1 clock is system clock (for DShot300/600)

    ; Setup variables for DShot300
    mov  DShot_Timer_Preset, #-128      ; Load DShot sync timer preset (for DShot300)
    mov  DShot_Pwm_Thr, #16             ; Load DShot pwm threshold (for DShot300)
    mov  DShot_Frame_Length_Thr, #80    ; Load DShot frame length criteria

    Set_DShot_Tlm_Bitrate 375000        ; = 5/4 * 300000

    ; Test whether signal is DShot300, if so begin arming
    mov  Rcp_Outside_Range_Cnt, #10     ; Set out of range counter
    call wait100ms                      ; Wait for new RC pulse
    mov  A, Rcp_Outside_Range_Cnt       ; Check if pulses were accepted
    jz   arming_begin

; Setup variables for DShot600 (Only on 48MHz for performance reasons)
IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    mov  DShot_Timer_Preset, #-64       ; Load DShot sync timer preset (for DShot600)
    mov  DShot_Pwm_Thr, #8              ; Load DShot pwm threshold (for DShot600)
    mov  DShot_Frame_Length_Thr, #40    ; Load DShot frame length criteria

    Set_DShot_Tlm_Bitrate 750000        ; = 5/4 * 600000

    ; Test whether signal is DShot600, if so begin arming
    mov  Rcp_Outside_Range_Cnt, #10     ; Set out of range counter
    call wait100ms                      ; Wait for new RC pulse
    mov  A, Rcp_Outside_Range_Cnt       ; Check if pulses were accepted
    jz   arming_begin
ENDIF

    ; No valid signal detected, try again
    ljmp init_no_signal

arming_begin:
    push PSW
    mov  PSW, #10h                      ; Temp8 in register bank 2 holds value
    mov  Temp8, CKCON0                  ; Save DShot clock settings for telemetry
    pop  PSW

    setb Flag_Had_Signal                ; Mark that a signal has been detected
    mov  Startup_Stall_Cnt, #0          ; Reset stall count

    clr  IE_EA
    call beep_f1_short                  ; Confirm RC pulse detection by beeping
    setb IE_EA

; Make sure RC pulse has been zero for ~300ms
arming_wait:
    clr  C
    mov  A, Rcp_Stop_Cnt
    subb A, #10
    jc   arming_wait

    clr  IE_EA
    call beep_f2_short                  ; Confirm arm state by beeping
    setb IE_EA

; Armed and waiting for power on (RC pulse > 0)
wait_for_start:
    clr  A
    mov  Comm_Period4x_L, A             ; Reset commutation period for telemetry
    mov  Comm_Period4x_H, A
    mov  DShot_Cmd, A                   ; Reset DShot command (only considered in this loop)
    mov  DShot_Cmd_Cnt, A
    mov  Beacon_Delay_Cnt, A            ; Clear beacon wait counter
    mov  Timer2_X, A                    ; Clear Timer2 extended byte

wait_for_start_loop:
    clr  C
    mov  A, Timer2_X
    subb A, #94
    jc   wait_for_start_no_beep         ; Counter wrapping (about 3 sec)

    mov  Timer2_X, #0
    inc  Beacon_Delay_Cnt               ; Increment beacon wait counter

    mov  Temp1, #Pgm_Beacon_Delay
    mov  A, @Temp1
    mov  Temp1, #20                     ; 1 min
    dec  A
    jz   beep_delay_set

    mov  Temp1, #40                     ; 2 min
    dec  A
    jz   beep_delay_set

    mov  Temp1, #100                    ; 5 min
    dec  A
    jz   beep_delay_set

    mov  Temp1, #200                    ; 10 min
    dec  A
    jz   beep_delay_set

    mov  Beacon_Delay_Cnt, #0           ; Reset beacon counter for infinite delay

beep_delay_set:
    clr  C
    mov  A, Beacon_Delay_Cnt
    subb A, Temp1                       ; Check against chosen delay
    jc   wait_for_start_no_beep         ; Has delay elapsed?

    dec  Beacon_Delay_Cnt               ; Decrement counter for continued beeping

    mov  Temp1, #4                      ; Beep tone 4
    clr  IE_EA                          ; Disable all interrupts
    call switch_power_off               ; Switch power off in case braking is set
    call beacon_beep
    setb IE_EA                          ; Enable all interrupts

wait_for_start_no_beep:
    jb   Flag_Telemetry_Pending, wait_for_start_check_rcp
    call dshot_tlm_create_packet        ; Create telemetry packet (0 rpm)
    call scheduler_run

wait_for_start_check_rcp:
    ; If RC pulse is higher than stop (>0) then proceed to start the motor
    jnb  Flag_Rcp_Stop, wait_for_start_nonzero

    mov  A, Rcp_Timeout_Cntd            ; Load RC pulse timeout counter value
    ljz  init_no_signal                 ; If pulses are missing - go back to detect input signal

    call dshot_cmd_check                ; Check and process DShot command

    sjmp wait_for_start_loop            ; Go back to beginning of wait loop

wait_for_start_nonzero:
    call wait100ms                      ; Wait to see if start pulse was glitch

    ; If RC pulse returned to stop (0) - start over
    jb   Flag_Rcp_Stop, wait_for_start_loop

    ; If no safety arm jump to motor start
    mov  Temp1, #Pgm_Safety_Arm
    cjne @Temp1, #001h, motor_start

    ; If EDT flag is set start motor
    jb  Flag_Ext_Tele, motor_start

    ; Safety is enabled. Check Flag_Ext_Tele is set
    ; If not set beep and wait again
    call beep_safety_no_arm
    jmp  wait_for_start_loop



;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Motor start entry point
;**** **** **** **** **** **** **** **** **** **** **** **** ****
motor_start:
    clr  IE_EA                          ; Disable interrupts

    call switch_power_off

motor_start_seam:                       ; stay-energised down-handoff entry (FETs still live from run1)
    clr  A
    mov  Flags0, A                      ; Clear run time flags (A==0 from clr A above; re-encoded to save bytes)
    mov  Flags1, A
    mov  Demag_Detected_Metric, A       ; Clear demag metric
    mov  Demag_Detected_Metric_Max, A   ; Clear demag metric max
    mov  Ext_Telemetry_H, A             ; Clear extended telemetry data

    jnb  Flag_Sine_Handoff, motor_start_cold   ; cold starts fall through unchanged
    ljmp sine_run                              ; seam: skip Pwm_Limit block, clock/DShot rescale,
                                               ; dir derive, startup flags, init pair
motor_start_cold:
    ; Set up start operating conditions
    mov  Temp2, #Pgm_Startup_Power_Max
    mov  Pwm_Limit_Beg, @Temp2          ; Set initial pwm limit
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_Beg
    mov  Pwm_Limit_By_Erpm, #0FFh       ; BlueGill: reset eRPM cap governor to full power

    ; Set temperature PWM limit and setpoint to the maximum value
    mov  Pwm_Limit, Pwm_Limit_Beg
    mov  Temp_Pwm_Level_Setpoint, Pwm_Limit_Beg

; Begin startup sequence
IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    Set_MCU_Clk_48MHz                   ; Enable 48MHz clock frequency

    ; Scale DShot criteria for 48MHz
    clr  C
    rlca DShot_Timer_Preset             ; Scale sync timer preset

    clr  C
    rlca DShot_Frame_Length_Thr         ; Scale frame length criteria

    clr  C
    rlca DShot_Pwm_Thr                  ; Scale pulse width criteria

    ; Scale DShot telemetry for 48MHz
    xcha DShot_GCR_Pulse_Time_1, DShot_GCR_Pulse_Time_1_Tmp
    xcha DShot_GCR_Pulse_Time_2, DShot_GCR_Pulse_Time_2_Tmp
    xcha DShot_GCR_Pulse_Time_3, DShot_GCR_Pulse_Time_3_Tmp

    mov  DShot_GCR_Start_Delay, #DSHOT_TLM_START_DELAY_48
ENDIF

    mov  C, Flag_Pgm_Dir_Rev            ; Read spin direction setting
    mov  Flag_Motor_Dir_Rev, C

    jnb  Flag_Pgm_Bidir, motor_start_bidir_done ; Check if bidirectional operation

    mov  C, Flag_Rcp_Dir_Rev            ; Read force direction
    mov  Flag_Motor_Dir_Rev, C          ; Set spinning direction

;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Motor start beginning
;**** **** **** **** **** **** **** **** **** **** **** **** ****
motor_start_bidir_done:
    ; Set initial motor state
    setb Flag_Startup_Phase             ; Set startup phase flags
    setb Flag_Initial_Run_Phase
    mov  Startup_Cnt, #0                ; Reset startup phase run counter
    mov  Initial_Run_Rot_Cntd, #12      ; Set initial run rotation countdown

    ; Initialize commutation
    call comm5_comm6                    ; Initialize commutation
    call comm6_comm1

    ; BlueGill S1: forced-commutation stepper mode. The init pair above already ran (so
    ; interrupts are enabled and sector 1 is briefly energised at the stale reload, as on
    ; the stock path); sine_run re-establishes a clean hold state before driving. It never
    ; returns (it exits via exit_run_mode).
    jnb  Flag_Sine_Mode, motor_start_no_sine
    ljmp sine_run                       ; (long jump: sine_run is out of jb range)

motor_start_no_sine:
    call initialize_timing              ; Initialize timing
    call calc_next_comm_period          ; Set virtual commutation point
    call initialize_timing              ; Initialize timing
    call calc_next_comm_period
    call initialize_timing              ; Initialize timing

    setb IE_EA                          ; Enable interrupts

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Run entry point
;
; Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
; Out_cA changes from low to high
;**** **** **** **** **** **** **** **** **** **** **** **** ****
run1:
    call wait_for_comp_out_high         ; Wait for high
    ; setup_comm_wait                    ; Setup wait time from zero cross to commutation
    ; evaluate_comparator_integrity      ; Check whether comparator reading has been normal
    call wait_for_comm                  ; Wait from zero cross to commutation
    call comm1_comm2                    ; Commutate
    call calc_next_comm_period          ; Calculate next timing and wait advance timing wait
    ; wait_advance_timing                ; Wait advance timing and start zero cross wait
    ; calc_new_wait_times
    ; wait_before_zc_scan                ; Wait zero cross wait and start zero cross timeout

; Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
; Out_cB changes from high to low
run2:
    call wait_for_comp_out_low
    ; setup_comm_wait
    ; evaluate_comparator_integrity
    call set_pwm_limit                  ; Set pwm power limit for low or high rpm
    call update_lowspeed_damping        ; BlueGill: switch braking strength at low speed
    call wait_for_comm
    call comm2_comm3
    call calc_next_comm_period
    ; wait_advance_timing
    ; calc_new_wait_times
    ; wait_before_zc_scan

; Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
; Out_cC changes from low to high
run3:
    call wait_for_comp_out_high
    ; setup_comm_wait
    ; evaluate_comparator_integrity
    call wait_for_comm
    call comm3_comm4
    call calc_next_comm_period
    ; wait_advance_timing
    ; calc_new_wait_times
    ; wait_before_zc_scan

; Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
; Out_cA changes from high to low
run4:
    call wait_for_comp_out_low
    ; setup_comm_wait
    ; evaluate_comparator_integrity
    call wait_for_comm
    call comm4_comm5
    call calc_next_comm_period
    ; wait_advance_timing
    ; calc_new_wait_times
    ; wait_before_zc_scan

; Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
; Out_cB changes from low to high
run5:
    call wait_for_comp_out_high
    ; setup_comm_wait
    ; evaluate_comparator_integrity
    call wait_for_comm
    call comm5_comm6
    call calc_next_comm_period
    ; wait_advance_timing
    ; calc_new_wait_times
    ; wait_before_zc_scan

; Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
; Out_cC changes from high to low
run6:
    call wait_for_comp_out_low
    ; setup_comm_wait
    ; evaluate_comparator_integrity
    call wait_for_comm
    call comm6_comm1
    call calc_next_comm_period
    call scheduler_run
    ; wait_advance_timing
    ; calc_new_wait_times
    ; wait_before_zc_scan

    ; Check if it is startup phases
    jnb  Flag_Initial_Run_Phase, normal_run_checks
    jnb  Flag_Startup_Phase, initial_run_phase

    ; Startup phase
    mov  Pwm_Limit, Pwm_Limit_Beg       ; Set initial max power
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_Beg; Set initial max power
    clr  C
    mov  A, Startup_Cnt                 ; Load startup counter
    subb A, #24                         ; Is counter above requirement?
    jnc  startup_phase_done

    jnb  Flag_Rcp_Stop, run1            ; If pulse is above stop value - Continue to run
    ljmp exit_run_mode                  ; (long jump: BlueGill S3 down-handoff widened this span)

startup_phase_done:
    ; Clear startup phase flag & remove pwm limits
    clr  Flag_Startup_Phase

initial_run_phase:
    ; If it is a direction change - branch
    jb   Flag_Dir_Change_Brake, normal_run_checks

    ; Decrement startup rotation count
    mov  A, Initial_Run_Rot_Cntd
    dec  A
    ; Check number of initial rotations
    jz   initial_run_phase_done         ; Branch if counter is zero

    mov  Initial_Run_Rot_Cntd, A        ; Not zero - store counter

    jnb  Flag_Rcp_Stop, run1            ; Check if pulse is below stop value
    jb   Flag_Pgm_Bidir, run1           ; Check if bidirectional operation

    ljmp exit_run_mode                  ; (long jump: BlueGill neutral-stop insert widened this span)

initial_run_phase_done:
    clr  Flag_Initial_Run_Phase         ; Clear initial run phase flag

    ; Lift startup power restrictions
    ; Temperature protection acts until this point
    ; as a max startup power limiter.
    ; This plus the power limits applied in set_pwm_limit function
    ; act as a startup power limiter to protect the esc and the motor
    ; during startup, jams produced after crashes and desyncs recovery
    mov  Pwm_Limit, #255                ; Reset temperature level pwm limit
    mov  Temp_Pwm_Level_Setpoint, #255  ; Reset temperature level setpoint

    setb Flag_Motor_Started             ; Set motor started
    jmp  run1                           ; Continue with normal run

normal_run_checks:
    ; Reset stall count
    mov  Startup_Stall_Cnt, #0
    setb Flag_Motor_Running

    ; [BlueGill] Sine-config neutral STOP by throttle MAGNITUDE. In bidir the host's neutral does NOT
    ; reliably set Flag_Rcp_Stop, and run6_bidir below false-reverses at neutral, so the rotor limps at
    ; the ~386 BEMF floor ("0 rpm keeps spinning"). The ISR snapshots the clean 11-bit magnitude to
    ; Sine_Rcp for sine configs; a near-zero magnitude = neutral -> leave to exit_run_mode ->
    ; wait_for_start (ARMED, signal still present) for a real stop and a re-arm-free restart. Only fires
    ; at true neutral (any RUNNING target keeps the magnitude well above the threshold). Gated on
    ; Flag_Sine_Mode, so stock bidir stays byte-identical.
    jnb  Flag_Sine_Mode, nrc_no_neutral
    mov  A, Sine_Rcp_H
    jnz  nrc_no_neutral                 ; magnitude >= 256 -> driving
    clr  C
    mov  A, Sine_Rcp_L
    subb A, #16                         ; magnitude < 16 (of 2047) -> treat as neutral
    jnc  nrc_no_neutral
    ljmp exit_run_mode

nrc_no_neutral:
    jnb  Flag_Rcp_Stop, run6_check_bidir ; Check if stop
    jb   Flag_Pgm_Bidir, run6_check_timeout ; Check if bidirectional operation

    mov  Temp2, #Pgm_Brake_On_Stop      ; Check if using brake on stop
    mov  A, @Temp2
    jz   run6_check_timeout

    ; Exit run mode after 100ms when using brake on stop
    clr  C
    mov  A, Rcp_Stop_Cnt                ; Load stop RC pulse counter value
    subb A, #3                          ; Is number of stop RC pulses above limit?
    jnc  exit_run_mode                  ; Yes - exit run mode

run6_check_timeout:
    ; Exit run mode immediately if timeout
    mov  A, Rcp_Timeout_Cntd            ; Load RC pulse timeout counter value
    jz   exit_run_mode                  ; If it is zero - go back to wait for power on

run6_check_bidir:
    jb   Flag_Pgm_Bidir, run6_bidir     ; Check if bidirectional operation

run6_check_speed:
    ; BlueGill S3: BEMF -> forced-sine down-handoff. Only when sine mode is configured and
    ; Cross_Dn != 0. Fires when the rotor has slowed to/below the down threshold (larger
    ; Comm_Period4x_H = slower), BEFORE the stock 0xF0 min-speed exit. Cross_Dn is clamped
    ; <= 0xEF at decode so it always precedes 0xF0. Hands to sine via the stay-energised seam
    ; (motor_start_seam -> sine_run) WITHOUT switch_power_off or the clock/DShot rescale, so the
    ; rotor stays driven across the handoff; Sine_Inc is seeded from the live period via Flag_Sine_Handoff.
    jnb  Flag_Sine_Mode, run6_check_speed_stock
    mov  Temp1, #Pgm_Sine_Cross_Dn
    mov  A, @Temp1
    jz   run6_check_speed_stock         ; Cross_Dn == 0 -> down-handoff disabled
    clr  C
    mov  A, Comm_Period4x_H             ; Comm_Period4x_H >= Cross_Dn (slow enough for sine)?
    subb A, @Temp1
    jc   run6_check_speed_stock         ; still fast enough -> stay in BEMF
    setb Flag_Sine_Handoff              ; (Flags3 survives motor_start's Flags0/1 wipe)
    clr  IE_EA                          ; atomic seam; FETs stay LIVE at the run1 topology (post comm6_comm1)
    ljmp motor_start_seam               ; stay-energised seam: skip switch_power_off + clock/DShot rescale, keep rotor driven

run6_check_speed_stock:
    clr  C
    mov  A, Comm_Period4x_H             ; Is Comm_Period4x below minimum speed?
    subb A, #0F0h                       ; Default minimum speed (~1330 erpm)
    jnc  exit_run_mode                  ; Yes - exit run mode
    jmp  run1                           ; No - go back to run 1

run6_bidir:
    ; Check if direction change braking is in progress
    jb   Flag_Dir_Change_Brake, run6_bidir_braking

    ; [S3 direction fix] Sine-config 6-step runs with the field convention inverted vs stock,
    ; so the expected spinning direction is (Flag_Rcp_Dir_Rev XOR Flag_Sine_Mode): == NOT
    ; Flag_Rcp_Dir_Rev while in a sine config (Flag_Sine_Mode set), == Flag_Rcp_Dir_Rev
    ; otherwise. Build that expected direction into C; the jb/jnb below never touch C, so it
    ; survives the interleaved bit tests. With Flag_Sine_Mode clear this reduces to stock.
    mov  C, Flag_Rcp_Dir_Rev            ; Read force direction
    jnb  Flag_Sine_Mode, run6_bidir_dir_ready
    cpl  C                              ; sine config: expected = NOT Flag_Rcp_Dir_Rev
run6_bidir_dir_ready:

    ; Check if actual rotation direction matches the expected direction (C)
    jb   Flag_Motor_Dir_Rev, run6_bidir_check_reversal
    jc   run6_bidir_reversal            ; Motor fwd, expected rev -> mismatch, reverse
    sjmp run6_check_speed

run6_bidir_check_reversal:
    jc   run6_check_speed               ; Motor rev, expected rev -> match

run6_bidir_reversal:
    ; Initiate direction and start braking
    setb Flag_Dir_Change_Brake          ; Set brake flag
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_Beg; Set max power while braking to initial power limit
    jmp  run4                           ; Go back to run 4,thereby changing force direction

run6_bidir_braking:
    mov  Pwm_Limit_By_Rpm, Pwm_Limit_Beg; Set max power while braking to initial power limit

    clr  C
    mov  A, Comm_Period4x_H             ; Is Comm_Period4x below minimum speed?
    subb A, #40h                        ; Bidirectional braking termination speed (~5000 erpm)
    jc   run6_bidir_continue            ; No - continue braking

    ; Braking done, set new spinning direction
    clr  Flag_Dir_Change_Brake          ; Clear braking flag
    ; [S3 direction fix] set the new spinning direction to the conjugated expected direction
    ; (Flag_Rcp_Dir_Rev XOR Flag_Sine_Mode): sine-config 6-step runs Motor == NOT Rcp, else
    ; Motor == Rcp. With Flag_Sine_Mode clear this is the stock `Flag_Motor_Dir_Rev := Rcp`.
    mov  C, Flag_Rcp_Dir_Rev            ; Read force direction
    jnb  Flag_Sine_Mode, run6_bidir_brake_dir_ready
    cpl  C                              ; sine config: Motor := NOT Flag_Rcp_Dir_Rev
run6_bidir_brake_dir_ready:
    mov  Flag_Motor_Dir_Rev, C          ; Set spinning direction
    setb Flag_Initial_Run_Phase
    mov  Initial_Run_Rot_Cntd, #18

run6_bidir_continue:
    jmp  run1                           ; Go back to run 1

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Exit run mode and power off
;
; Happens on normal stop (RC pulse == 0) or comparator timeout
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
exit_run_mode_on_timeout:
    jb   Flag_Motor_Running, exit_run_mode
    inc  Startup_Stall_Cnt              ; Increment stall count if motors did not properly start

exit_run_mode:
    ; Disable all interrupts (they will be disabled for a while, be aware)
    clr  IE_EA

    clr  Flag_Sine_Run                  ; BlueGill S1: leave sine mode (Flags3 is not cleared below)
    call switch_power_off
    mov  Flags0, #0                     ; Clear run time flags (in case they are used in interrupts)
    mov  Flags1, #0

IF MCU_TYPE == MCU_BB2 or MCU_TYPE == MCU_BB51
    Set_MCU_Clk_24MHz

    ; Scale DShot criteria for 24MHz
    setb C
    rrca DShot_Timer_Preset             ; Scale sync timer preset

    clr  C
    rrca DShot_Frame_Length_Thr         ; Scale frame length criteria

    clr  C
    rrca DShot_Pwm_Thr                  ; Scale pulse width criteria

    ; Scale DShot telemetry for 24MHz
    xcha DShot_GCR_Pulse_Time_1, DShot_GCR_Pulse_Time_1_Tmp
    xcha DShot_GCR_Pulse_Time_2, DShot_GCR_Pulse_Time_2_Tmp
    xcha DShot_GCR_Pulse_Time_3, DShot_GCR_Pulse_Time_3_Tmp

    mov  DShot_GCR_Start_Delay, #DSHOT_TLM_START_DELAY
ENDIF

    ; Check if RCP is zero, then it is a normal stop or signal timeout
    jb   Flag_Rcp_Stop, exit_run_mode_no_stall

    ; It is a stall!
    ; Signal stall
    setb Flag_Stall_Notify

    ; Check max consecutive stalls and exit if stall counter > 3
    clr  C
    mov  A, Startup_Stall_Cnt
    subb A, #3
    jnc  exit_run_mode_is_stall

    ; At this point there was a desync event, and a new try is to be done.
    ; The program will jump to motor_start. Interrupts are disabled at this
    ; point so it is safe to jump to motor start, where a new initial state
    ; will be set

    call wait100ms                      ; Wait for a bit between stall restarts

    ljmp motor_start                    ; Go back and try starting motors again

exit_run_mode_is_stall:
    ; Enable all interrupts (disabled above, in exit_run_mode)
    setb IE_EA

    ; Clear extended DSHOT telemetry flag if turtle mode is not active
    ; This flag is also used for EDT safety arm flag
    ; We don't want to deactivate extended telemetry during turtle mode
    ; Extended telemetry flag is important because it is involved in
    ; EDT safety feature. We don't want to disable EDT arming during
    ; turtle mode.
    jb Flag_User_Reverse_Requested, exit_run_mode_is_stall_beep
    clr Flag_Ext_Tele

exit_run_mode_is_stall_beep:
    ; Stalled too many times
    clr  IE_EA
    call beep_motor_stalled
    setb IE_EA

    ljmp arming_begin                   ; Go back and wait for arming

exit_run_mode_no_stall:
    ; Enable all interrupts (disabled above, in exit_run_mode)
    setb IE_EA

    ; Clear extended DSHOT telemetry flag if turtle mode is not active
    ; This flag is also used for EDT safety arm flag
    ; We don't want to deactivate extended telemetry during turtle mode
    ; Extended telemetry flag is important because it is involved in
    ; EDT safety feature. We don't want to disable EDT arming during
    ; turtle mode.
    jb Flag_User_Reverse_Requested, exit_run_mode_no_stall_no_beep
    clr Flag_Ext_Tele

exit_run_mode_no_stall_no_beep:
    ; Clear stall counter
    mov  Startup_Stall_Cnt, #0

    mov  Temp1, #Pgm_Brake_On_Stop      ; Check if using brake on stop
    mov  A, @Temp1
    jz   exit_run_mode_brake_done

    A_Com_Fet_On                        ; Brake on stop
    B_Com_Fet_On
    C_Com_Fet_On

exit_run_mode_brake_done:
    ljmp wait_for_start                 ; Go back to wait for power on

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Reset
;
; Should execution ever reach this point the ESC will be reset,
; as code flash after offset 1A00 is used for EEPROM storage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
CSEG AT CSEG_RESET
reset:
    ljmp pgm_start

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Bootloader
;
; Include source code for BLHeli bootloader
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;CSEG AT 1C00h
$include (BLHeliBootLoad.inc)

END
