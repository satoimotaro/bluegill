#!/usr/bin/env python3
# ==============================================================================
# BlueGill S3 "catch a spinning rotor" — independent truth-table derivation
# ==============================================================================
# The S3 up-handoff catches a *coasting* (all-FETs-off) spinning rotor by reading
# the sign of each phase's back-EMF through the comparator, forming a 3-bit sector
# pattern, and looking up which 6-step run state to enter so that closed-loop BEMF
# resumes with FORWARD torque and the correct NEXT zero-cross (no reversal).
#
# This script re-derives that lookup table FROM FIRST PRINCIPLES (the phase-BEMF
# sinusoids + the stock run1..run6 comparator wait polarities) and asserts it
# matches the table hand-coded in src/Modules/SineMode.asm (Sine_Catch_Table).
# It is standalone (only math/sys) and exits non-zero on ANY mismatch, so it can
# gate CI. If this and the firmware ever disagree, ONE of them is wrong and a
# wrong entry reverse-locks the rotor on live FETs — treat a failure as blocking.
#
# BEMF model (the ONLY convention whose zero-cross order matches the hardware
# A^0 B_60 C^120 A_180 B^240 C_300 — NOT the S2 sine_drive_model.py convention):
#       Ea = sin(theta)
#       Eb = sin(theta - 240deg)
#       Ec = sin(theta - 120deg)
# Bit order in the pattern: bit2 = sign(Ea), bit1 = sign(Eb), bit0 = sign(Ec),
# with 1 = phase > neutral (comparator CMP_CN0 bit 0x40 set, COMPARATOR_INVERT=0).
#
# Run -> (watched phase, edge) map, straight out of Bluejay.asm:957-1034:
#       run1 comp A rising   run2 comp B falling  run3 comp C rising
#       run4 comp A falling  run5 comp B rising   run6 comp C falling
# Reverse (Flag_Motor_Dir_Rev): the _rev comm routines swap the comparator phase
# A<->C on runs 1/3/4/6 (runs 2/5 keep B); the run EDGE polarity is NOT dir-swapped.
# That is exactly a bit A(2)<->C(0) swap on the READ pattern followed by the SAME
# forward lookup — which is what the firmware does. This script proves that by
# deriving the reverse table from the reverse *physics* and checking it equals the
# swap-then-forward-lookup result.
# ==============================================================================

import math
import sys

# --- expected tables (must equal what SineMode.asm ships) ---------------------
EXPECT_FWD = [0, 5, 1, 6, 3, 4, 2, 0]   # DB Sine_Catch_Table
EXPECT_REV = [0, 3, 1, 2, 5, 4, 6, 0]   # forward table indexed by A<->C-swapped pattern

# phase index: 0 = A, 1 = B, 2 = C ; per-phase BEMF angular offset (deg)
_OFFSET = {0: 0.0, 1: 240.0, 2: 120.0}


def bemf(theta_deg, phase):
    """Phase back-EMF at electrical angle theta (deg). +ve => phase above neutral."""
    return math.sin(math.radians(theta_deg - _OFFSET[phase]))


def sgn(x):
    return 1 if x > 0.0 else 0


def sector_pattern(theta_deg):
    """3-bit sign pattern bit2=A, bit1=B, bit0=C at electrical angle theta."""
    return (sgn(bemf(theta_deg, 0)) << 2) \
         | (sgn(bemf(theta_deg, 1)) << 1) \
         | (sgn(bemf(theta_deg, 2)))


def zero_cross_at(boundary_deg, direction):
    """The (phase, edge) of the zero-cross the rotor crosses at boundary_deg.
    direction = +1 forward (theta increasing in time), -1 reverse (decreasing).
    edge is 'rising'/'falling' as TIME advances. Exactly one phase crosses at each
    60deg boundary."""
    eps = 0.75
    hits = []
    for ph in range(3):
        before = bemf(boundary_deg - direction * eps, ph)   # slightly earlier in time
        after = bemf(boundary_deg + direction * eps, ph)     # slightly later in time
        if sgn(before) != sgn(after):
            hits.append((ph, 'rising' if after > before else 'falling'))
    assert len(hits) == 1, f"expected 1 ZC at {boundary_deg} dir {direction}, got {hits}"
    return hits[0]


# run -> (watched phase, edge). Reverse swaps comparator phase A(0)<->C(2).
FWD_RUN = {
    1: (0, 'rising'),
    2: (1, 'falling'),
    3: (2, 'rising'),
    4: (0, 'falling'),
    5: (1, 'rising'),
    6: (2, 'falling'),
}
_SWAP_PH = {0: 2, 1: 1, 2: 0}
REV_RUN = {s: (_SWAP_PH[ph], edge) for s, (ph, edge) in FWD_RUN.items()}


def run_for_zc(zc, run_map):
    for state, watched in run_map.items():
        if watched == zc:
            return state
    raise AssertionError(f"no run waits for ZC {zc}")


def swap_ac_bits(pattern):
    """Swap bit2 (A) and bit0 (C); bit1 (B) unchanged."""
    return ((pattern & 0b010)
            | ((pattern & 0b100) >> 2)
            | ((pattern & 0b001) << 2))


def derive():
    fwd = [0] * 8
    rev = [0] * 8
    rows = []
    for k in range(6):
        lo = k * 60.0
        hi = lo + 60.0
        mid = lo + 30.0
        pat = sector_pattern(mid)
        # forward: rotor advances, the NEXT crossing in time is the UPPER boundary
        f_state = run_for_zc(zero_cross_at(hi % 360.0, +1), FWD_RUN)
        # reverse: rotor retreats, the NEXT crossing in time is the LOWER boundary,
        # evaluated with the _rev comparator-phase map
        r_state = run_for_zc(zero_cross_at(lo % 360.0, -1), REV_RUN)
        fwd[pat] = f_state
        rev[pat] = r_state
        rows.append((int(lo), int(hi), pat, f_state, r_state))
    return fwd, rev, rows


def fmt3(p):
    return format(p, '03b')


def main():
    fwd, rev, rows = derive()

    print("BlueGill S3 catch-spinning-rotor truth table (independent derivation)")
    print("  model: Ea=sin(t), Eb=sin(t-240), Ec=sin(t-120); bit2=A bit1=B bit0=C")
    print()
    print("  sector(deg) | pat ABC | fwd state/run | rev state/run")
    print("  ------------+---------+---------------+--------------")
    for lo, hi, pat, f, r in rows:
        print(f"   {lo:3d}-{hi:3d}    |  {fmt3(pat)}={pat:d}  |    run{f}       |    run{r}")
    print()
    print(f"  fwd table [idx0..7] = {fwd}")
    print(f"  rev table [idx0..7] = {rev}   (= fwd[swap_A<->C(idx)])")
    print(f"  invalid patterns    = 000(0), 111(7)  -> retry then fallback")

    ok = True

    if fwd != EXPECT_FWD:
        print(f"\nFAIL: forward table {fwd} != expected {EXPECT_FWD}")
        ok = False
    if rev != EXPECT_REV:
        print(f"\nFAIL: reverse table {rev} != expected {EXPECT_REV}")
        ok = False

    # the load-bearing firmware rule: reverse == swap A<->C then SAME forward table
    for p in range(8):
        if p in (0, 7):
            continue
        want = fwd[swap_ac_bits(p)]
        if rev[p] != want:
            print(f"\nFAIL: swap rule broken at {fmt3(p)}: rev={rev[p]} "
                  f"but fwd[swap]={want}")
            ok = False

    # invalid codes must be reserved (never emitted as a sector pattern)
    emitted = {sector_pattern(k * 60.0 + 30.0) for k in range(6)}
    if 0 in emitted or 7 in emitted:
        print("\nFAIL: an all-0/all-1 pattern was emitted as a real sector")
        ok = False
    if fwd[0] or fwd[7] or rev[0] or rev[7]:
        print("\nFAIL: invalid indices 0/7 must map to 0")
        ok = False

    if not ok:
        print("\nRESULT: MISMATCH -- firmware Sine_Catch_Table must be fixed.")
        sys.exit(1)

    print("\nRESULT: OK -- matches SineMode.asm Sine_Catch_Table [0,5,1,6,3,4,2,0]")
    print("        and the reverse A<->C-swap rule.")
    sys.exit(0)


if __name__ == "__main__":
    main()
