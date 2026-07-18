#!/usr/bin/env bash
# BlueGill build — Linux/Wine + Keil C51 (AX51/LX51/Ohx51), mirroring upstream
# bluejay/Makefile. BlueGill is a hard fork of Bluejay v0.21.0, so the modified 8051
# assembly lives directly in src/ (no vendor submodule + overlay stage anymore).
#
# Flow:  stage src/ -> build/stage/src
#        AX51 (assemble) -> LX51 (link) -> Ohx51 (hex)
#        -> dist/BlueGill_<LAYOUT>_<MCU>_<DEADTIME>_<PWM>_<VERSION>.hex
#
# `git diff v0.21.0-base` shows exactly what BlueGill changed vs the upstream base.
#
# Requires: wine, and the Keil eval C51 tools installed under $KEIL_PATH
# (see README.md — the Keil download is a manual, one-time step).
set -euo pipefail

# --- locate repo root (this script lives in tools/build/) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC="${ROOT}/src"

# --- config: target.env may pre-seed LAYOUT/MCU/DEADTIME/PWM/VERSION ---
TARGET_ENV="${TARGET_ENV:-${ROOT}/targets/littlebee-spring-30a/target.env}"
if [ -f "${TARGET_ENV}" ]; then
  # shellcheck disable=SC1090
  source "${TARGET_ENV}"
fi

usage() {
  cat >&2 <<EOF
Usage: $0 [-l LAYOUT] [-m MCU] [-d DEADTIME] [-p PWM] [-s VERSION]
  -l  layout letter A..W|Z|OA   (default from target.env: ${LAYOUT:-unset})
  -m  MCU  H(BB21)|X(BB51)|L(BB1)  (default: ${MCU:-H})
  -d  deadtime integer          (default from target.env: ${DEADTIME:-unset})
  -p  PWM 24|48|96              (default: ${PWM:-24})
  -s  version string           (default: ${VERSION:-pinned tag})
Env: KEIL_PATH (default ~/.wine/drive_c/Keil_v5/C51/BIN)
EOF
  exit 2
}

while getopts ":l:m:d:p:s:h" o; do
  case "${o}" in
    l) LAYOUT="${OPTARG}" ;;
    m) MCU="${OPTARG}" ;;
    d) DEADTIME="${OPTARG}" ;;
    p) PWM="${OPTARG}" ;;
    s) VERSION="${OPTARG}" ;;
    *) usage ;;
  esac
done

MCU="${MCU:-H}"
PWM="${PWM:-24}"
# Default version = the fork base tag (hex names keep the v0.21.0 base scheme).
if [ -z "${VERSION:-}" ]; then
  VERSION="$(git -C "${ROOT}" describe --tags --abbrev=0 --match 'v*-base' 2>/dev/null | sed 's/-base$//' || true)"
  VERSION="${VERSION:-v0.21.0}"
fi

# --- validate ---
[ -d "${SRC}" ] || { echo "ERROR: source tree missing (${SRC})" >&2; exit 1; }
[ -n "${LAYOUT:-}" ]   || { echo "ERROR: LAYOUT unset (read it off the hardware — see targets/littlebee-spring-30a/README.md)" >&2; usage; }
[ -n "${DEADTIME:-}" ] || { echo "ERROR: DEADTIME unset (read it off the hardware — see targets/littlebee-spring-30a/README.md)" >&2; usage; }
case "${MCU}" in H|X|L) ;; *) echo "ERROR: MCU must be H|X|L" >&2; usage ;; esac
case "${PWM}" in 24|48|96) ;; *) echo "ERROR: PWM must be 24|48|96" >&2; usage ;; esac
# Valid layout letters (from src/Modules/Common.asm — X and Y are commented
# out upstream) and the upstream deadtime set. Reject anything else early with a pointer.
VALID_LAYOUTS="A B C D E F G H I J K L M N O P Q R S T U V W Z OA"
VALID_DEADTIMES="0 5 10 15 20 25 30 40 50 70 90 120"
case " ${VALID_LAYOUTS} " in *" ${LAYOUT} "*) ;; *) echo "ERROR: LAYOUT='${LAYOUT}' invalid (must be one of: ${VALID_LAYOUTS}). See targets/littlebee-spring-30a/README.md" >&2; exit 1 ;; esac
case " ${VALID_DEADTIMES} " in *" ${DEADTIME} "*) ;; *) echo "ERROR: DEADTIME='${DEADTIME}' invalid (must be one of: ${VALID_DEADTIMES}). See targets/littlebee-spring-30a/README.md" >&2; exit 1 ;; esac

KEIL_PATH="${KEIL_PATH:-${HOME}/.wine/drive_c/Keil_v5/C51/BIN}"
AX51_BIN="${KEIL_PATH}/AX51.exe"
LX51_BIN="${KEIL_PATH}/LX51.exe"
OX51_BIN="${KEIL_PATH}/Ohx51.exe"
for exe in "${AX51_BIN}" "${LX51_BIN}" "${OX51_BIN}"; do
  [ -f "${exe}" ] || { echo "ERROR: missing Keil tool: ${exe}" >&2; echo "       Install the Keil eval C51 tools under Wine (see tools/build/README.md)." >&2; exit 1; }
done
command -v wine >/dev/null 2>&1 || { echo "ERROR: wine not found (see tools/build/README.md)" >&2; exit 1; }

# --- derive the numeric DEFINEs exactly as upstream Makefile does ---
if [ "${LAYOUT}" = "OA" ]; then
  ESCNO=27
else
  ESCNO=$(( $(printf '%d' "'${LAYOUT}") - 65 + 1 ))   # A->1 .. Z->26
fi
case "${MCU}" in L) MCU_TYPE=0 ;; H) MCU_TYPE=1 ;; X) MCU_TYPE=2 ;; esac
case "${PWM}" in 24) PWM_FREQ=0 ;; 48) PWM_FREQ=1 ;; 96) PWM_FREQ=2 ;; esac

# --- stage src/ (a clean copy so the Keil tools write into build/, not the tree) ---
STAGE="${ROOT}/build/stage"
rm -rf "${STAGE}"
mkdir -p "${STAGE}"
cp -a "${SRC}" "${STAGE}/src"
echo "staged src/ (BlueGill fork of Bluejay ${VERSION})"

BASE="${LAYOUT}_${MCU}_${DEADTIME}_${PWM}_${VERSION}"
OUT="${ROOT}/build"
OBJ="${OUT}/${BASE}.OBJ"
LST="${OUT}/${BASE}.LST"
OMF="${OUT}/${BASE}.OMF"
HEX="${OUT}/${BASE}.hex"
DIST="${ROOT}/dist"
mkdir -p "${DIST}"

# AX51 mixes up input defines when run in parallel — build one target at a time.
AX51_FLAGS="NOMOD51 REGISTERBANK(0,1,2) NOLIST NOSYMBOLS"

echo "AX51 : ${BASE}  (ESCNO=${ESCNO} MCU_TYPE=${MCU_TYPE} DEADTIME=${DEADTIME} PWM_FREQ=${PWM_FREQ})"
( cd "${STAGE}/src" && wine "${AX51_BIN}" "Bluejay.asm" \
    "DEFINE(ESCNO=${ESCNO})" \
    "DEFINE(MCU_TYPE=${MCU_TYPE})" \
    "DEFINE(DEADTIME=${DEADTIME})" \
    "DEFINE(PWM_FREQ=${PWM_FREQ})" \
    "OBJECT(${OBJ})" \
    "PRINT(${LST})" \
    ${AX51_FLAGS} ) > /dev/null 2>&1 \
  || { echo "AX51 failed:"; grep -B3 -E '\*\*\* (ERROR|WARNING)' "${LST}" || true; exit 1; }

echo "LX51 : linking -> ${OMF}"
# LX51 names the map after the OMF basename, UPPERCASED (incl. the version), exactly like
# upstream's Makefile ($(shell echo ... | tr 'a-z' 'A-Z')). On a case-sensitive FS the file
# is e.g. O_H_15_24_V0.21.0.MAP — grepping the lowercase name would silently miss it.
MAP="${OUT}/$(echo "${BASE}.MAP" | tr 'a-z' 'A-Z')"
# LX51 exit status: <2 = ok (0, or 1 for the expected single warning); >=2 = real error.
set +e
wine "${LX51_BIN}" "${OBJ}" TO "${OMF}" > /dev/null 2>&1
lx_rc=$?
set -e
# Fall back to a glob if the uppercase guess doesn't match what LX51 actually wrote.
[ -f "${MAP}" ] || MAP="$(ls "${OUT}/${BASE}".[Mm][Aa][Pp] 2>/dev/null | head -n1 || true)"
if [ "${lx_rc}" -ge 2 ] || { [ -n "${MAP}" ] && grep -qE '\*\*\* ERROR' "${MAP}"; }; then
  echo "LX51 failed (exit ${lx_rc}):"
  [ -n "${MAP}" ] && grep -A3 -E '\*\*\* (ERROR|WARNING)' "${MAP}" || true
  exit 1
fi

echo "OHX  : ${HEX}"
wine "${OX51_BIN}" "${OMF}" "HEXFILE(${HEX})" > /dev/null 2>&1 \
  || { echo "Ohx51 failed to produce hex"; exit 1; }

cp -f "${HEX}" "${DIST}/BlueGill_${BASE}.hex"
echo "OK   : ${DIST}/BlueGill_${BASE}.hex"
