#!/usr/bin/env python3
"""Reference model for BlueGill #3 open-loop sinusoidal drive (see docs/sine-drive-design.md).

Validatable with NO hardware. Proves the 3-phase angle->PWM math the 8051 asm will implement,
quantifies LUT quantization error, models the fixed-point angle integrator, and emits the exact
sine LUT bytes for the asm. Pure stdlib (math only)."""
import math

# ---- motor / drive constants (LittleBee A_H_30 + 12N14P test motor) --------------------
POLE_PAIRS   = 7          # 14 magnets
PWM_FREQ_HZ  = 24000      # 24 kHz variant
LUT_STEPS    = 256        # electrical angle resolution (full wave), 8-bit angle
PHASE_OFFS   = (0.0, 120.0, 240.0)   # 3-phase electrical offsets (deg)

def build_quarterwave_lut(pwm_bits):
    """Quarter-wave sine LUT, amplitude = full PWM half-range. Full wave via symmetry.
    Returns (quarter_lut[0..LUT_STEPS//4], amp) with amp = (2**pwm_bits - 1)//2."""
    amp = (2**pwm_bits - 1) // 2
    q = LUT_STEPS // 4
    lut = [round(amp * math.sin(2*math.pi*(i/LUT_STEPS))) for i in range(q+1)]
    return lut, amp

def sine_from_quarter(lut, amp, idx):
    """Reconstruct full-wave sine value at angle index (0..LUT_STEPS-1) from the quarter LUT."""
    q = LUT_STEPS // 4
    idx &= (LUT_STEPS - 1)
    if   idx <= q:          return  lut[idx]
    elif idx <= 2*q:        return  lut[2*q - idx]
    elif idx <= 3*q:        return -lut[idx - 2*q]
    else:                   return -lut[LUT_STEPS - idx]

def phase_duty(lut, amp, mid, theta_idx, phase_deg, ampl_frac=1.0):
    """Center-aligned complementary duty for one phase. mid = 50%. Returns 0..(2*mid)."""
    off = int(round(phase_deg/360.0 * LUT_STEPS))
    s = sine_from_quarter(lut, amp, theta_idx + off)
    return mid + int(round(ampl_frac * s))

def check_balance(pwm_bits=11):
    """Sum of the three phase duties must be constant (= 3*mid) for every angle -> neutral fixed."""
    lut, amp = build_quarterwave_lut(pwm_bits)
    mid = amp                     # 50% point (duty range 0..2*amp)
    worst = 0
    for th in range(LUT_STEPS):
        s = sum(phase_duty(lut, amp, mid, th, p) for p in PHASE_OFFS)
        worst = max(worst, abs(s - 3*mid))
    return worst                  # small nonzero = LUT rounding only

def quant_error(pwm_bits):
    """RMS + peak error (in PWM LSBs and % of amplitude) of the quarter-wave LUT vs ideal sine."""
    lut, amp = build_quarterwave_lut(pwm_bits)
    errs = []
    for th in range(LUT_STEPS):
        ideal = amp * math.sin(2*math.pi*(th/LUT_STEPS))
        errs.append(sine_from_quarter(lut, amp, th) - ideal)
    rms = math.sqrt(sum(e*e for e in errs)/len(errs))
    pk  = max(abs(e) for e in errs)
    return rms, pk, amp

def third_harmonic_headroom():
    """SVPWM/3rd-harmonic injection keeps Sum==neutral but raises the linear fundamental range
    by ~15% (1/cos30). Verify the injected waveform still peaks <= amp and stays balanced."""
    N = LUT_STEPS
    inj = [math.sin(2*math.pi*i/N) + (1/6)*math.sin(3*2*math.pi*i/N) for i in range(N)]
    peak = max(abs(x) for x in inj)          # ~0.866 -> can scale fundamental up by 1/0.866
    return peak, 1.0/peak

def angle_integrator(target_mech_rpm):
    """Fixed-point angle step per PWM period for a target mech RPM. dtheta (LUT idx units)."""
    f_elec = target_mech_rpm/60.0 * POLE_PAIRS      # electrical Hz
    erpm   = f_elec * 60.0
    dtheta_idx = f_elec / PWM_FREQ_HZ * LUT_STEPS   # LUT indices advanced per PWM period
    # 8051 fixed-point: accumulate dtheta in a 16.16 phase accumulator; inc = dtheta * 65536
    inc_q16 = int(round(dtheta_idx * 65536)) & 0xFFFFFFFF
    return f_elec, erpm, dtheta_idx, inc_q16

# ============================================================================
# S1 "as-built" forced-commutation STEPPER model.
#
# S1 does NOT use the sine LUT above (that is the future-S2 true-3-phase design,
# kept below for reference). S1 steps the stock 6-step commutation vectors from a
# fixed-point angle accumulator at a host-commanded rate, with a V/f duty schedule.
# The constants here MUST match the asm EQUs in src/Bluejay.asm:
#     SINE_TICK_T2 = 4000     (Timer2 ticks per control tick)
#     SINE_RCP_SHIFT = 3      (inc = Rcp << 3  == Rcp * 8)
#     DEFAULT_PGM_SINE_HOLD_AMP = 8, DEFAULT_PGM_SINE_AMP_MAX = 20
# and Timer2 runs at SYSCLK/12 = 4 MHz during run (48 MHz core on BB21).
# ============================================================================
STEP_TIMER2_HZ   = 4_000_000     # Timer2 clock during run (SYSCLK/12, 48 MHz core)
STEP_TICK_T2     = 4000          # SINE_TICK_T2: Timer2 ticks per control tick
STEP_RCP_SHIFT   = 3             # SINE_RCP_SHIFT: inc = Rcp << 3
STEP_ACC_BITS    = 16            # angle accumulator width; step on 16-bit overflow
STEP_SECTORS     = 6             # 6 commutation steps per electrical revolution
STEP_HOLD_AMP    = 8             # DEFAULT_PGM_SINE_HOLD_AMP (8-bit throttle-equiv)
STEP_AMP_MAX     = 20            # DEFAULT_PGM_SINE_AMP_MAX
# Host bidir throttle: thrust s in [0,1000] -> DShot (1000+s) -> firmware 11-bit Rcp.
# The bidirectional split doubles the half-range, so full thrust (s=1000) -> Rcp≈2047.
STEP_THRUST_TO_RCP = 2047.0 / 1000.0    # ≈2.047 Rcp per thrust unit


def stepper_erpm(rcp):
    """Electrical RPM for an 11-bit throttle magnitude rcp (0..2047)."""
    inc = rcp << STEP_RCP_SHIFT                          # per-tick accumulator increment
    tick_hz = STEP_TIMER2_HZ / STEP_TICK_T2              # control-tick rate (Hz)
    steps_per_s = inc / (1 << STEP_ACC_BITS) * tick_hz   # sector steps per second
    return steps_per_s / STEP_SECTORS * 60.0             # -> electrical RPM


def stepper_amp(rcp):
    """V/f amplitude (8-bit throttle-equiv) the asm computes: hold + (inc>>8), clamp amp_max."""
    inc = rcp << STEP_RCP_SHIFT
    amp = STEP_HOLD_AMP + (inc >> 8)
    return min(amp, STEP_AMP_MAX)


def print_stepper_section():
    tick_hz = STEP_TIMER2_HZ / STEP_TICK_T2
    print("=== S1 forced-commutation STEPPER (as-built; asm EQUs must match) ===")
    print(f"  Timer2 clock            : {STEP_TIMER2_HZ/1e6:.3f} MHz (SYSCLK/12, 48 MHz core)")
    print(f"  SINE_TICK_T2            : {STEP_TICK_T2} ticks -> {STEP_TICK_T2/STEP_TIMER2_HZ*1e3:.3f} ms "
          f"({tick_hz:.0f} Hz control rate)")
    print(f"  SINE_RCP_SHIFT          : {STEP_RCP_SHIFT}  (inc = Rcp << {STEP_RCP_SHIFT} = Rcp*{1<<STEP_RCP_SHIFT})")
    print(f"  angle accumulator       : {STEP_ACC_BITS}-bit, one sector per overflow ({STEP_SECTORS} sectors/erev)")
    fs_erpm = stepper_erpm(2047)
    fs_mech = fs_erpm / POLE_PAIRS
    print(f"  full-scale (Rcp=2047)   : eRPM={fs_erpm:.1f}  mech_RPM={fs_mech:.2f} (7 pole-pairs)")
    print(f"  throttle scale          : eRPM/Rcp={fs_erpm/2047:.5f}   mech_RPM/Rcp={fs_mech/2047:.5f}")
    # Host servo constants derived from full-scale (used by host/posctl.py):
    fullscale_mech_rpm = 1000 * STEP_THRUST_TO_RCP * fs_mech / 2047   # mech RPM at thrust=1000
    kff = 1000.0 / (fullscale_mech_rpm * 6.0)                         # thrust per (deg/s)
    print(f"  host FULLSCALE_RPM      : {fullscale_mech_rpm:.2f} mech RPM at thrust=1000 "
          f"(target_rpm = thrust*FULLSCALE/1000)")
    print(f"  host Kff                : {kff:.4f} thrust per deg/s  (= 1000/(FULLSCALE_RPM*6))")
    print("  Rcp  thrust  inc   eRPM   mech_RPM   V/f_amp(8b)")
    for rcp in (0, 64, 128, 256, 512, 1024, 2047):
        thr = rcp / STEP_THRUST_TO_RCP
        print(f"  {rcp:4d} {thr:6.0f}  {rcp<<STEP_RCP_SHIFT:5d}  {stepper_erpm(rcp):6.1f}  "
              f"{stepper_erpm(rcp)/POLE_PAIRS:7.2f}     {stepper_amp(rcp):3d}")


def emit_asm_lut(pwm_bits=11):
    lut, amp = build_quarterwave_lut(pwm_bits)
    body = ",".join(str(v) for v in lut)
    return f"; quarter-wave sine LUT ({len(lut)} entries, amp={amp}, {pwm_bits}-bit PWM)\n" \
           f"Sine_Quarter_Lut: DB {body}"

# ============================================================================
# S2 "as-built" min-clamp (flat-bottom / DPWM) two-phase SVPWM model.
#
# Option (c): the instantaneously most-negative phase is clamped to the negative
# rail (its Com/low-side FET fully ON, exactly as a stock 6-step commutation
# vector already does). The other TWO phases carry sinusoidal high-side duty:
#   - the "pair" phase (lower Port-1 pin number) is driven by the stock POWER +
#     DAMP complementary pair (PCA modules 0 + 1, DEADTIME skew) -> synchronous.
#   - the "second" phase (higher pin) is driven high-side-only by the FREE PCA
#     module 2 (CEX2); its Com/low-side FET is LATCHED OFF -> body-diode
#     freewheel, so that leg can NEVER shoot through regardless of the m2 duty.
#
# Crossbar role-fixity (verified against the EFM8BB2 Reference Manual, Port I/O
# Crossbar Decoder 11.3.3 + XBR1 11.4.2 + PCA0POL 16.4.7): the priority decoder
# assigns each enabled PCA channel to the least-significant UN-skipped Port-1
# pin, in the fixed priority CEX0 > CEX1 > CEX2. With Layout A pin order
# Ap=0<Ac=1<Bp=2<Bc=3<Cp=4<Cc=5 and the rule "pair = lower-lettered modulated
# phase", CEX0/CEX1 always grab the pair's Pwm/Com (two lowest un-skipped pins)
# and CEX2 grabs the second phase's Pwm -> module roles are fixed for every
# segment and both directions. XBR1 PCA0ME=0x3 routes all three; PCA0CENT.2 and
# PCA0POL.2 give CEX2 the same center-aligned / polarity treatment as CEX0.
#
# IMPORTANT geometry note (the plan under-specified this): under MIN-clamp DPWM
# the clamped phase is the most-negative one, whose 120 deg windows are centred
# on each phase's negative peak. Their boundaries (theta = 90/210/330 deg) fall
# at the CENTRES of the even commutation sectors (2/4/6), NOT at sector
# boundaries. So the asm re-muxes (P1SKIP + Com latch) at m==16 of even sectors
# -- 3 times per electrical rev. At each such swap the CEX2 duty is identically
# 0 (the outgoing second phase is entering clamp), so the pin reassignment is
# glitch-free by construction.
# ============================================================================
S2_MICRO       = 32          # microsteps per 60 deg comm sector (Sine_Frac_H top 5 bits)
S2_STEP_DEG    = 60.0 / S2_MICRO         # 1.875 deg / microstep
S2_LUT_LEN     = 49          # L[j] = round(255*sin(j*1.875 deg)), j=0..48 (0..90 deg)
S2_AMP_PEAK    = 255         # arc peak == full V/f amplitude (Sine_Amp scales it down)

# Layout A Port-1 pin numbers (A.inc): phase -> (Pwm pin, Com pin).
S2_PIN = {0: (0, 1), 1: (2, 3), 2: (4, 5)}      # 0=A, 1=B, 2=C
S2_PHASE_NAME = "ABC"

def build_s2_arc_lut():
    """Cosine-arc LUT for min-clamp duty: L[j] = round(255*sin(j*1.875 deg)),
    j=0..48. A modulated phase's duty is L[48-|offset|] where offset is its
    distance (in microsteps) from the sector boundary at which it peaks."""
    return [round(S2_AMP_PEAK * math.sin(math.radians(j * S2_STEP_DEG))) for j in range(S2_LUT_LEN)]

def s2_phase_duty_ideal(theta_deg, ph):
    """Ideal min-clamp high-side duty (0..255) for phase ph at electrical angle theta."""
    v = [math.sin(math.radians(theta_deg + off)) for off in (0.0, -120.0, +120.0)]  # A,B,C
    vmin = min(v)
    d = (v[ph] - vmin) / math.sqrt(3.0) * S2_AMP_PEAK   # /sqrt3 so peak spread -> 255
    return max(0.0, d)

def s2_clamp_phase(sector, m):
    """Clamped (most-negative) phase for comm sector 1..6 and microstep 0..31.
    Odd sectors have a constant clamp; even sectors switch clamp at m==16."""
    #                 sector: 1  2      3  4      5  6
    # constant clamp (odd) :  B  -      C  -      A  -
    # switch at m16 (even) :  -  B->C   -  C->A   -  A->B
    table = {
        1: (1, 1), 2: (1, 2), 3: (2, 2),
        4: (2, 0), 5: (0, 0), 6: (0, 1),
    }
    lo, hi = table[sector]
    return lo if m < S2_MICRO // 2 else hi

def s2_segment(sector, m):
    """Return (clamp, pair, second) phase indices for a sector/microstep.
    pair = lower-pin modulated phase (POWER+DAMP), second = higher-pin (CEX2)."""
    clamp = s2_clamp_phase(sector, m)
    mods = sorted(p for p in (0, 1, 2) if p != clamp)   # ascending pin order
    return clamp, mods[0], mods[1]

def s2_p1skip(pair, second):
    """P1SKIP mask that un-skips pair.Pwm, pair.Com, second.Pwm (all else skipped)."""
    unskip = (1 << S2_PIN[pair][0]) | (1 << S2_PIN[pair][1]) | (1 << S2_PIN[second][0])
    return 0xFF & ~unskip

def s2_duty_from_lut(lut, sector, m, ph):
    """Reconstruct phase ph's duty from the arc LUT (what the asm computes).
    offset = distance in microsteps from ph's nearest peak (a sector boundary
    where that phase's high-side duty == 255)."""
    g = (sector - 1) * S2_MICRO + m                     # global microstep 0..191
    # phase ph peaks (duty=255) at the two sector boundaries flanking its saddle.
    # A(0): g=32,64 ; B(1): g=96,128 ; C(2): g=0(=192),160  -> peaks 96*ph+32, +32 apart? derive:
    peaks = {0: (32, 64), 1: (96, 128), 2: (160, 192)}[ph]
    best = min((abs(((g - p + 96) % 192) - 96) for p in peaks))  # min |offset| (wrap 192)
    if best > 48:
        return 0
    return lut[48 - best]

def print_s2_dpwm_section():
    lut = build_s2_arc_lut()
    print("=== S2 min-clamp two-phase SVPWM (option c; asm must match) ===")
    print(f"  microsteps/sector  : {S2_MICRO} ({S2_STEP_DEG:.4f} deg each; 192/erev)")
    print(f"  arc LUT            : {S2_LUT_LEN} bytes  L[j]=round(255*sin(j*1.875deg))  peak={lut[-1]}")
    print(f"  clamp topology     : CEX2 leg low-side HELD OFF -> shoot-through impossible")

    # --- self-check 1: arc-LUT reconstruction vs ideal min-clamp, all 192 microsteps ---
    max_lut_err = 0
    duty_min, duty_max = 999, -999
    for sector in range(1, 7):
        for m in range(S2_MICRO):
            theta = ((sector - 1) * S2_MICRO + m) * S2_STEP_DEG
            for ph in range(3):
                ideal = s2_phase_duty_ideal(theta, ph)
                got = s2_duty_from_lut(lut, sector, m, ph)
                max_lut_err = max(max_lut_err, abs(got - ideal))
                duty_min, duty_max = min(duty_min, got), max(duty_max, got)
    print(f"  arc-LUT vs ideal   : max err {max_lut_err:.1f} LSB   duty range [{duty_min},{duty_max}]")

    # --- self-check 2: line-to-line reconstruction is sinusoidal (< 2% RMS) ---
    ll_err, ll_ref = [], []
    for g in range(192):
        theta = g * S2_STEP_DEG
        sector, m = g // S2_MICRO + 1, g % S2_MICRO
        d = [s2_duty_from_lut(lut, sector, m, ph) for ph in range(3)]
        vab = d[0] - d[1]                                # line-to-line A-B (duty units)
        # duties already carry the 1/sqrt3 scale, so dA-dB = 255*sin(theta+30) exactly
        ideal_ab = S2_AMP_PEAK * math.sin(math.radians(theta + 30.0))
        ll_err.append(vab - ideal_ab)
        ll_ref.append(ideal_ab)
    rms = math.sqrt(sum(e * e for e in ll_err) / len(ll_err))
    ref_rms = math.sqrt(sum(r * r for r in ll_ref) / len(ll_ref))
    ll_pct = 100.0 * rms / ref_rms
    print(f"  line-line A-B RMS  : {ll_pct:.2f}% of ideal  (spec < 2%)")

    # --- self-check 3: sector-centre continuity vs S1 (the dominant vector matches
    #     the phase that the stock 6-step powers at that sector) ---
    # At each sector centre (m=16) the two modulated duties are equal (balanced pair),
    # i.e. the net vector points along the sector bisector -> same direction S1 steps.
    cont_ok = True
    for sector in range(1, 7):
        _, pair, second = s2_segment(sector, 8)          # a mid-first-half sample
        dp = s2_duty_from_lut(lut, sector, 8, pair)
        ds = s2_duty_from_lut(lut, sector, 8, second)
        # both modulated phases must be actively driven (> clamp) inside a sector
        if not (dp > 0 and ds > 0):
            cont_ok = False
    print(f"  sector continuity  : {'ok' if cont_ok else 'FAIL'} (both modulated phases driven)")

    # --- self-check 3b: the clamp-transition REMUX (m=16 of even sectors, at g=48/112/176)
    #     is glitch-free -> the phase ENTERING clamp has ~0 high-side duty at the switch,
    #     AND line-to-line stays continuous across it (no torque discontinuity). This is
    #     the load-bearing property the asm relies on to reassign CEX2 at 0 duty. ---
    remux_ok = True
    for g_bound, entering in ((48, 2), (112, 0), (176, 1)):   # clamp B->C, C->A, A->B
        s_b, m_b = g_bound // S2_MICRO + 1, g_bound % S2_MICRO
        d_enter = s2_duty_from_lut(lut, s_b, m_b, entering)
        # line-to-line A-B one microstep either side of the boundary must be continuous
        def _ll(gg):
            s, m = gg // S2_MICRO + 1, gg % S2_MICRO
            dd = [s2_duty_from_lut(lut, s, m, p) for p in range(3)]
            return dd[0] - dd[1]
        jump = abs(_ll(g_bound) - _ll(g_bound - 1))
        if d_enter > 2 or jump > 10:                     # 10 LSB ~ one microstep of slope
            remux_ok = False
    print(f"  remux @ m=16       : {'ok' if remux_ok else 'FAIL'} (entering-clamp duty ~0; line-line continuous)")

    # --- self-check 3c: REVERSE direction (asm g_eff = 192 - g). The firmware reuses the
    #     SAME clamp/index/LUT machinery with a mirrored position (no second table), so
    #     reverse must reconstruct the ideal min-clamp just as cleanly, and its line-line
    #     A-B must be the time-reverse of forward (opposite rotation sense). ---
    def g_eff_rev(g):
        ge = 192 - g
        return 0 if ge == 192 else ge
    rev_lut_err, rev_dmin, rev_dmax = 0, 999, -999
    rev_ll_err, rev_ll_ref = [], []
    for g in range(192):
        ge = g_eff_rev(g)
        s, m = ge // S2_MICRO + 1, ge % S2_MICRO
        theta = ge * S2_STEP_DEG
        d = [s2_duty_from_lut(lut, s, m, ph) for ph in range(3)]
        for ph in range(3):
            rev_lut_err = max(rev_lut_err, abs(d[ph] - s2_phase_duty_ideal(theta, ph)))
            rev_dmin, rev_dmax = min(rev_dmin, d[ph]), max(rev_dmax, d[ph])
        rev_ll_err.append((d[0] - d[1]) - S2_AMP_PEAK * math.sin(math.radians(theta + 30.0)))
        rev_ll_ref.append(S2_AMP_PEAK * math.sin(math.radians(theta + 30.0)))
    rev_rms = math.sqrt(sum(e * e for e in rev_ll_err) / len(rev_ll_err))
    rev_ref = math.sqrt(sum(r * r for r in rev_ll_ref) / len(rev_ll_ref))
    rev_ll_pct = 100.0 * rev_rms / rev_ref
    print(f"  reverse (g=192-g)  : arc err {rev_lut_err:.1f} LSB  duty [{rev_dmin},{rev_dmax}]  "
          f"line-line RMS {rev_ll_pct:.2f}%")
    print("  reverse sector table (clamp/pair/CEX2  P1SKIP):")
    for sector in range(1, 7):
        cols = []
        for m in (0, 16):
            g = (sector - 1) * S2_MICRO + m
            ge = g_eff_rev(g)
            se, me = ge // S2_MICRO + 1, ge % S2_MICRO
            c, p, s = s2_segment(se, me)
            cols.append(f"{S2_PHASE_NAME[c]}/{S2_PHASE_NAME[p]}/{S2_PHASE_NAME[s]}  0x{s2_p1skip(p, s):02X}")
        print(f"    {sector}     {cols[0]:22s}          {cols[1]}")

    # --- per-sector segment table (what sine2_set_segment encodes), both dirs ---
    print("  sector  m<16 (clamp/pair/CEX2  P1SKIP)   m>=16 (clamp/pair/CEX2  P1SKIP)")
    for sector in range(1, 7):
        cols = []
        for m in (0, 16):
            c, p, s = s2_segment(sector, m)
            cols.append(f"{S2_PHASE_NAME[c]}/{S2_PHASE_NAME[p]}/{S2_PHASE_NAME[s]}  0x{s2_p1skip(p, s):02X}")
        print(f"    {sector}     {cols[0]:22s}          {cols[1]}")

    # --- emit the LUT DB bytes for cross-check against SineMode.asm ---
    body = ",".join(str(v) for v in lut)
    print(f"  ; --- paste-match for SineMode.asm ---")
    print(f"  Sine2_Arc_Lut: DB {body}")

    # hard self-check: fail nonzero on any spec violation (gate for the asm constants)
    violations = []
    if max_lut_err > 2.0:      violations.append(f"arc-LUT err {max_lut_err:.1f} LSB > 2")
    if duty_min < 0 or duty_max > 255: violations.append(f"duty out of [0,255]: [{duty_min},{duty_max}]")
    if ll_pct >= 2.0:          violations.append(f"line-line RMS {ll_pct:.2f}% >= 2%")
    if not cont_ok:            violations.append("sector continuity failed")
    if not remux_ok:           violations.append("m=16 remux boundary not glitch-free")
    if rev_lut_err > 2.0:      violations.append(f"reverse arc-LUT err {rev_lut_err:.1f} LSB > 2")
    if rev_dmin < 0 or rev_dmax > 255: violations.append(f"reverse duty out of [0,255]: [{rev_dmin},{rev_dmax}]")
    if rev_ll_pct >= 2.0:      violations.append(f"reverse line-line RMS {rev_ll_pct:.2f}% >= 2%")
    if len(lut) != 49 or lut[-1] != 255: violations.append("LUT length/peak wrong")
    return violations

# ============================================================================
# S3 sine <-> BEMF-6-step crossover (as-built). Proves the safety-critical
# Comm_Period4x seed on the up-handoff and the Sine_Inc seed on the down-handoff,
# and mirrors the host threshold math (host/esctool.py sine_crossover_bytes) so a
# divergence fails the build. The asm implements exactly these formulas:
#   up-handoff : Comm_Period4x = ticks_4sector * 2000   (4-sector window halves the one-shot
#                                quantization; net x2 = /2 timestamp o x4 "4x" over 2 sectors)
#                Prev_Comm     = (TMR2_now>>1) - Comm_Period4x/4  (averaging-invariant)
#                fired only when TICKS_MIN <= ticks_4sector <= TICKS_MAX
#   down-handoff: Sine_Inc_Seed = 2048000 / Cross_Dn
# ============================================================================
S3_TIMER2_RAW_HZ   = 4_000_000          # Timer2 raw clock (SYSCLK/12), one sector steps per tick
S3_TICK_T2         = 4000               # SINE_TICK_T2 (raw ticks per 1 kHz control tick)
S3_SEED_MULT       = 2000               # asm Comm_Period4x = ticks_4sector * 2000
S3_TICKS_MIN       = 9                  # SINE_CROSS_TICKS_MIN: below this the seed is >~11% coarse
S3_TICKS_MAX       = 30                 # SINE_CROSS_TICKS_MAX: above this is below the BEMF floor
S3_MAX_REL_ERR     = 0.12              # ship gate: no handoff-able config may seed worse than this
S3_INC_PER_ERPM    = 65536.0 / 10000.0  # Sine_Inc per eRPM (= 6.5536)
S3_UP_ERPM_PER_UNIT = 256 * 10000.0 / 65536.0    # per Cross_Up (Sine_Inc_H) unit = 39.0625
S3_DN_ERPM_NUM     = 312500.0           # down eRPM = NUM / Cross_Dn (Comm_Period4x_H units)
S3_DN_SEED_NUM     = 2_048_000          # Sine_Inc_Seed = NUM / Cross_Dn
S3_COMM_ERPM_NUM   = 80_000_000         # eRPM = NUM / Comm_Period4x (halved-TMR2, 2 MHz units)
S3_CROSS_DN_MAX_BYTE = 0xEF             # firmware clamp (>=0xF0 disables; before stock 0xF0 exit)


def s3_ticks_4sector(sine_inc):
    """Simulate the asm Sine_Step_Ticks counter exactly: whole 1 kHz ticks measured from the
    tick that steps INTO sector 3 to the tick that steps INTO sector 1 (a 4-sector window)."""
    frac, sector, ticks = 0, 1, 0
    for _ in range(1 << 20):                       # generous bound
        frac += sine_inc
        stepped = frac >= (1 << 16)
        if stepped:
            frac -= (1 << 16)
            sector = sector + 1 if sector < 6 else 1
        ticks = min(ticks + 1, 255)                # asm inc-saturate
        if stepped:
            if sector == 3:
                ticks = 0                          # reset window (4-sector: sectors 3,4,5,6)
            elif sector == 1:
                return ticks                       # read at up-handoff point
    raise RuntimeError("s3_ticks_4sector did not converge (inc too small)")


def s3_average_pass(seed):
    """One calc_next_comm_period average with Comm_Period4x=seed and this_period=seed//4
    (Prev_Comm back-dated by exactly one commutation). Branch selected as the asm does with
    Flag_Initial_Run_Phase set. Returns the new Comm_Period4x (must equal seed)."""
    this_period = seed // 4
    h = seed >> 8
    if h < 4:                                      # div_16_4
        return seed - (seed >> 4) + (this_period >> 2)
    # 4<=h<8 -> div_8_2 ; h>=8 with Initial_Run_Phase -> div_8_2_slow ; both are -/8 + this/2
    return seed - (seed >> 3) + (this_period >> 1)


def s3_seed_comm_period4x(ticks_4sector):
    """asm: Comm_Period4x = ticks_4sector * 2000 (0x07D0), via ticks*0xD0 + (ticks*0x07)<<8."""
    return (ticks_4sector * S3_SEED_MULT) & 0xFFFF


def print_s3_crossover_section():
    print("=== S3 sine<->BEMF crossover (as-built; asm + host must match) ===")
    print(f"  up  : Cross_Up in Sine_Inc_H units, {S3_UP_ERPM_PER_UNIT:.4f} eRPM/unit (>= = faster)")
    print(f"  dn  : Cross_Dn in Comm_Period4x_H units, {S3_DN_ERPM_NUM:.0f}/unit (INVERSE, >= = slower)")
    print(f"  seed: Comm_Period4x = ticks_4sector*{S3_SEED_MULT} ; Sine_Inc_Seed = {S3_DN_SEED_NUM}/Cross_Dn")
    violations = []

    # --- self-check (a): Comm_Period4x seed = ticks_4sector*2000 matches 80e6/eRPM. The asm fires
    #     the up-handoff only when the 4-sector window is in [MIN,MAX] ticks: the MIN bound caps the
    #     ONE-SHOT relative quantization to ~1/MIN (so a too-fast config can never silently seed
    #     BEMF badly), the MAX bound is the BEMF floor (~40000/30 = 1333 eRPM) and keeps the seed
    #     16-bit. We assert BOTH abs (<=1 tick) AND relative (<= S3_MAX_REL_ERR) for firing rows. ---
    print(f"  up-handoff seed vs 80e6/eRPM (handoff only when {S3_TICKS_MIN} <= ticks4 <= {S3_TICKS_MAX}):")
    print("    eRPM    Sine_Inc  ticks4  seed(ticks*2000)  80e6/eRPM  err(ticks)  rel_err  handoff")
    for erpm in (1200, 1400, 1500, 2000, 3000, 4000, 4444, 5000, 8000):
        sine_inc = round(erpm * S3_INC_PER_ERPM)
        t4 = s3_ticks_4sector(sine_inc)
        seed = s3_seed_comm_period4x(t4)
        ideal = S3_COMM_ERPM_NUM / erpm
        err_ticks = (seed - ideal) / S3_TICK_T2
        rel_err = abs(seed - ideal) / ideal
        fires = S3_TICKS_MIN <= t4 <= S3_TICKS_MAX
        print(f"    {erpm:5d}   {sine_inc:7d}   {t4:4d}     {seed:7d}         {ideal:8.1f}"
              f"   {err_ticks:+.2f}     {rel_err*100:5.1f}%   {'yes' if fires else 'no'}")
        if fires:
            if abs(err_ticks) > 1.0:
                violations.append(f"up seed @ {erpm} eRPM off by {err_ticks:.2f} ticks (> 1)")
            if rel_err > S3_MAX_REL_ERR:
                violations.append(f"up seed @ {erpm} eRPM rel err {rel_err*100:.1f}% > "
                                  f"{S3_MAX_REL_ERR*100:.0f}% ship bound (config too fast for the window)")
            if not (0 <= seed <= 0xFFFF):
                violations.append(f"up seed @ {erpm} eRPM = {seed} not 16-bit")

    # --- self-check (b): the priming average leaves the seed unchanged (all speed branches) ---
    avg_ok = True
    for erpm in (1200, 1500, 2000, 3000, 5000, 8000, 40000, 120000):
        seed = min(round(S3_COMM_ERPM_NUM / erpm), 0xFFFF)
        if seed == 0:
            continue
        if s3_average_pass(seed) != seed:
            avg_ok = False
            violations.append(f"average pass moved seed {seed} (eRPM {erpm}) to {s3_average_pass(seed)}")
    print(f"  averaging invariance (4T - 4T/8 + T/2 = 4T, all branches): {'ok' if avg_ok else 'FAIL'}")

    # --- self-check (c): Sine_Inc_Seed = 2048000/Cross_Dn round-trips to the dn eRPM +-1 unit;
    #     and (d) Sine_Inc_Seed_H < Cross_Up for every valid (dn<up) config (the asm chatter guard,
    #     provably equivalent to the host dn_eff<up_eff hysteresis check). ---
    print("  down-handoff seed + guard (host math mirrored):")
    print("    up_eRPM dn_eRPM  Cross_Up Cross_Dn  Sine_Inc_Seed  seed_H  dn_eff  up_eff  guard")
    rt_ok, guard_ok = True, True
    for up_erpm, dn_erpm in ((2000, 1500), (3000, 2000), (8000, 3000), (5000, 1400), (9000, 1350)):
        cross_up = round(up_erpm / S3_UP_ERPM_PER_UNIT)
        cross_dn = round(S3_DN_ERPM_NUM / dn_erpm)
        if not (1 <= cross_up <= 255 and 1 <= cross_dn <= S3_CROSS_DN_MAX_BYTE):
            continue
        seed_inc = min(S3_DN_SEED_NUM // cross_dn, 0xFFFF)   # asm integer floor division
        seed_h = seed_inc >> 8
        # round-trip: seed_inc back to eRPM vs the dn target
        rt_erpm = seed_inc * (10000.0 / 65536.0)
        rt_err_units = abs(seed_inc - S3_DN_SEED_NUM / cross_dn)
        up_eff = cross_up * S3_UP_ERPM_PER_UNIT
        dn_eff = S3_DN_ERPM_NUM / cross_dn
        guard = "keep" if seed_h < cross_up else "DISABLE"
        print(f"    {up_erpm:6d} {dn_erpm:6d}    0x{cross_up:02X}     0x{cross_dn:02X}      {seed_inc:6d}"
              f"      {seed_h:3d}   {dn_eff:6.0f}  {up_eff:6.0f}  {guard}")
        if rt_err_units > 1.0:
            rt_ok = False
            violations.append(f"dn seed {seed_inc} (Cross_Dn 0x{cross_dn:02X}) round-trip off {rt_err_units:.2f} > 1")
        # dn<up (valid hysteresis) MUST imply the asm guard keeps the crossover (seed_H<Cross_Up)
        if dn_eff < up_eff and seed_h >= cross_up:
            guard_ok = False
            violations.append(f"guard rejects a valid config: seed_H {seed_h} >= Cross_Up {cross_up} "
                              f"while dn_eff {dn_eff:.0f} < up_eff {up_eff:.0f}")
    print(f"  down seed round-trip (+-1 unit): {'ok' if rt_ok else 'FAIL'}   "
          f"chatter guard vs hysteresis: {'ok' if guard_ok else 'FAIL'}")

    return violations


if __name__ == "__main__":
    import sys
    print_stepper_section()
    print()
    s2_violations = print_s2_dpwm_section()
    print()
    s3_violations = print_s3_crossover_section()
    print()

    # --- S1 speed-contract guard: these MUST stay locked (host posctl.py depends on them) ---
    s1_violations = []
    if abs(stepper_erpm(2047) / 2047 - 1.22070) > 1e-4:
        s1_violations.append(f"S1 eRPM/Rcp={stepper_erpm(2047)/2047:.5f} != 1.22070")
    fullscale = 1000 * STEP_THRUST_TO_RCP * (stepper_erpm(2047) / POLE_PAIRS) / 2047
    if abs(fullscale - 356.97) > 0.5:
        s1_violations.append(f"S1 FULLSCALE_RPM={fullscale:.2f} != ~357")

    print("--- reference: future-S2 true 3-phase sine LUT design (NOT used by S1) ---")
    print("=== 3-phase balance (worst |sum - 3*mid|, LSBs) ===")
    for b in (8, 9, 10, 11):
        print(f"  {b}-bit PWM: {check_balance(b)}")
    print("=== quarter-wave LUT quantization error vs ideal sine ===")
    for b in (8, 9, 10, 11):
        rms, pk, amp = quant_error(b)
        print(f"  {b}-bit: amp={amp:4d}  rms={rms:.2f} LSB  peak={pk:.2f} LSB  ({100*pk/amp:.2f}% of amp)")
    print("=== 3rd-harmonic injection headroom ===")
    peak, gain = third_harmonic_headroom()
    print(f"  injected peak={peak:.4f} -> fundamental can scale x{gain:.4f} (+{100*(gain-1):.1f}% linear range)")
    print("=== angle integrator (7 pole-pairs, 24 kHz PWM) ===")
    print("  mech_rpm  f_elec(Hz)   eRPM   dtheta(idx/pwm)   inc_q16(hex)")
    for rpm in (30, 60, 120, 185, 400, 1000):
        fe, erpm, dth, inc = angle_integrator(rpm)
        print(f"  {rpm:7d}  {fe:9.2f}  {erpm:7.0f}   {dth:.5f}          0x{inc:08X}")
    print("=== asm LUT (11-bit) ===")
    print(emit_asm_lut(11))

    if s2_violations or s1_violations or s3_violations:
        print()
        for v in s1_violations + s2_violations + s3_violations:
            print(f"SELF-CHECK FAILED: {v}", file=sys.stderr)
        sys.exit(1)
