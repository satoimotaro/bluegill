# Target: LittleBee Spring 30A (EFM8BB21)

Reference ESC for BlueGill. This file is the **bench procedure** to fill the two
PLACEHOLDERS in `target.env` (`LAYOUT`, `DEADTIME`) and the motor facts — they are
**read off the hardware, never guessed**.

## What is fixed vs. what must be measured

| Field | Value | Source |
|---|---|---|
| `MCU` | `H` (EFM8BB21, sig `E8 B2`) | Confirmed by pico-esc-tool `esc_flash` / bootloader signature. |
| `PWM` | `24` default (+ build a `48` variant) | Compile-time choice; low-KV motors favour 24 kHz duty resolution. |
| `LAYOUT` | **PLACEHOLDER** | Read off the ESC's stored layout tag (below). |
| `DEADTIME` | **PLACEHOLDER** | Read off the ESC's stored layout tag (below). |
| `MOTOR_POLES` / `MOTOR_KV` | **PLACEHOLDER** | Motor datasheet / bench count. |

## Why LAYOUT and DEADTIME cannot be guessed (compat-guard fact)

BlueJay bakes a 16-byte **layout tag** into flash at `0x1A40`, generated (in
`vendor/bluejay/src/Modules/Common.asm`) as:

    #<LAYOUT>_<MCU>_<DEADTIME>#           e.g.  #O_H_15#

pico-esc-tool's CLI flash guard (`host/esctool.py flash`, the inline check at ~:355-372)
does a **byte-exact `strcmp`** of the ESC's *currently stored* `0x1A40` tag against the tag
in the firmware HEX. Because it encodes **layout letter AND deadtime**, picking the wrong
letter or deadtime makes the guard report `layout MISMATCH` and blocks the flash unless
you pass `--force`. So both must equal what the board actually reports.

> **What `esctool.py flash` writes (verified in source).** The CLI flash is **not**
> app-region-only: `cmd_flash` writes the app region `[0x0000,0x1A00)` **and** the
> config/identity page `0x1A00–0x1BFF` (assembled from the HEX's defaults by `_pages_from`,
> `host/esctool.py` ~:337-348,:379-392); only the bootloader (`≥0x1C00`) is preserved, and
> it prints `"firmware default config applied"`. (The separate C++ `lib/esc_flash` path is
> app-only, but the CLI does not use it to program.) Two consequences:
> 1. **First flash off a stock ESC may need `--force` ONCE — because that flash
>    *overwrites* the stored `0x1A40` tag**, not because the tag persists. If this board's
>    *stock* tag is a manufacturer descriptor (pico-esc-tool `lib/esc_setup/EEPROM.md`
>    shows an example like `#FVTLibee30A#` at `0x40`) rather than the `#<L>_H_<DT>#` form,
>    the guard reports `layout MISMATCH` and you flash once with `--force`. After that the
>    stored tag is the firmware's `#<L>_H_<DT>#`, so every later flash of the same target
>    passes un-forced. Read and record the stock tag first (below); if it already reads
>    `#<L>_H_<DT>#`, no force is needed even the first time.
> 2. **Every re-flash RESETS all EEPROM params to firmware defaults.** Any `esctool.py set`
>    tuning is wiped by a flash and must be re-applied afterwards (see the tuning workflow
>    in `docs/low-speed-tuning.md`).

## Procedure — read LAYOUT / DEADTIME off the hardware

From the pico-esc-tool checkout, with the ESC on a configured signal pin:

    python host/esctool.py list

Each ESC prints `sig`, `layout` (the `0x1A40` tag), `name` (`0x1A60`) and `fw`. Record
the **layout** column verbatim.

- If it reads `#<L>_H_<DD>#` (e.g. `#O_H_15#`): `LAYOUT=<L>`, `DEADTIME=<DD>` directly.
- If it reads a manufacturer descriptor (e.g. `#FVTLibee30A#`): map the board to a
  BlueJay layout letter via the esc-configurator layout database and cross-check against
  the stock BLHeli-S hex name in `bitdump/BLHeli` → `BLHeli_S SiLabs/Hex files/`
  (filenames are `<L>_H_<DD>...`). Confirm the chosen `<L>_H_<DD>` by building it and
  reading the guard output before a non-forced flash.

`DEADTIME` must be one of the upstream set: `0 5 10 15 20 25 30 40 50 70 90 120`.

Then fill `target.env`:

    LAYOUT=<letter>
    DEADTIME=<value>

## Procedure — motor facts (for eRPM math)

`eRPM = mechanical_RPM * pole_pairs`, `pole_pairs = pole_count / 2`.

- Count magnet poles on the rotor bell (or from the motor datasheet) → `MOTOR_POLES`.
- Record `MOTOR_KV` if known. A typical low-KV thruster is 14-pole (7 pole-pairs);
  at 2000 mech RPM that is 14 000 eRPM. These numbers drive the tuning targets in
  `docs/low-speed-tuning.md` and the `Pgm_Max_Erpm` cap (a future patch).

## Build once filled

    ./tools/build/build.sh                 # uses target.env (24 kHz)
    ./tools/build/build.sh -p 48           # 48 kHz A/B variant

Output: `dist/BlueGill_<LAYOUT>_H_<DEADTIME>_<PWM>_<version>.hex`. See
`tools/build/README.md` for byte-comparing against the official v0.21.0 release hex.

## Milestone 1 — BEFORE the first (`--force`) flash: back up the stock ESC

The stock LittleBee firmware hex is **not vendored**, and the CLI flash overwrites the
`0x1A00–0x1BFF` config page and resets EEPROM params (see the flash note above), so there
is **no clean rollback to stock** unless you capture the ESC first. Do this once, per ESC:

1. **Save the current config/EEPROM page** (settings + identity + tags), so you can read
   the stock `0x1A40` tag and restore params later:

       python host/esctool.py read <i> -o stock-esc<i>-config.yaml

2. **Dump the app + config region** using the Pico's raw flash primitive (there is no
   packaged "backup" subcommand). Enter a session and read `0x0000–0x1BFF` in 256-byte
   chunks, e.g. via `connect` + repeated `readflash <i> <addr> 256` device commands, and
   save the bytes. This is your only image of the stock app if you ever need to revert.
   (Recovering a bricked BL still requires a C2 debugger — the bootloader `≥0x1C00` is
   never touched by any flash path, so a normal reflash cannot brick it.)
3. Record the stock `0x1A40` tag verbatim (from step 1 / `esctool.py list`) in this file's
   notes, then proceed with the build + `flash` (using `--force` only if the guard reports
   `layout MISMATCH` — see the flash note above).
