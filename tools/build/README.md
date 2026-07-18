# tools/build — BlueGill build toolchain (Linux / Wine + Keil C51)

BlueGill assembles with the **same toolchain as upstream BlueJay**: the Keil C51
assembler/linker (`AX51` → `LX51` → `Ohx51`) run under **Wine**. This mirrors
`vendor/bluejay/Makefile`; our `build.sh` adds one step — staging BlueGill's `src/`
overlays on top of the pinned vendor sources before assembling.

> There is no SDCC path. BlueJay's 8051 source is written for the Keil assembler; do not
> attempt a generic SDCC build.

## Files

- `build.sh` — build one target: stage `vendor/bluejay/src` → overlay `src/**` →
  `AX51`/`LX51`/`Ohx51` → `dist/BlueGill_<LAYOUT>_<MCU>_<DEADTIME>_<PWM>_<VERSION>.hex`.
  Reads defaults from `targets/littlebee-spring-30a/target.env`; flags `-l -m -d -p -s`
  override. Refuses to run if `LAYOUT`/`DEADTIME` are unset (they must be read off the
  hardware — see the target README).
- `diff-vendor.sh` — print each `src/` overlay's diff vs the pinned vendor file (the
  review view of exactly what BlueGill changes). Exit 1 if any overlay differs.

Iteration 1 ships **no** `src/` overlays, so `build.sh` reproduces the clean upstream
image — byte-comparable to the official v0.21.0 release hex (see below).

## Install the toolchain on Ubuntu 24.04

### 1. Wine (scriptable)

    sudo dpkg --add-architecture i386
    sudo apt-get update
    sudo apt-get install -y wine wine32:i386 wine64

(The 32-bit runtime is required — the Keil eval binaries are Win32. Upstream's
`vendor/bluejay/tools/Dockerfile` installs `wine wine32` on Debian for the same reason.)

### 2. Keil C51 eval assembler (MANUAL — cannot be scripted)

The Keil C51 tools require registration/download from Keil and are **not** redistributable,
so this step is manual and one-time. The eval edition's **assembler is not size-limited**,
which is all BlueJay/BlueGill needs (pure asm, no C compiler).

1. Download the **Keil C51** development tools (the "C51" / MDK-legacy package) from
   <https://www.keil.com/demo/eval/c51.htm> (free eval; requires a Keil account).
2. Install it under Wine so the binaries land at the upstream-default path:

       wine <C51_installer>.exe        # install into C:\Keil_v5

   Result should be:

       ~/.wine/drive_c/Keil_v5/C51/BIN/AX51.exe
       ~/.wine/drive_c/Keil_v5/C51/BIN/LX51.exe
       ~/.wine/drive_c/Keil_v5/C51/BIN/Ohx51.exe

3. If you install elsewhere, point the build at it:

       export KEIL_PATH="$HOME/.wine/drive_c/Keil_v5/C51/BIN"

   Alternatively, reuse an existing Simplicity Studio / BLHeli build VM's `Keil_v5`
   directory by copying it into `~/.wine/drive_c/` (exactly what upstream's Dockerfile
   does — it `COPY`s a pre-populated `.wine/drive_c/Keil_v5`).

`build.sh` verifies all three binaries exist and errors with the missing path if not.

## Build

    ./tools/build/build.sh                    # target.env defaults (LittleBee, 24 kHz)
    ./tools/build/build.sh -p 48              # 48 kHz variant
    ./tools/build/build.sh -l O -m H -d 15 -p 24 -s bluegill-v0.1.0

Intermediates go to `build/` (git-ignored); the final hex is copied to `dist/`.

### DEFINE mapping (identical to upstream Makefile)

| Symbol | From | Mapping |
|---|---|---|
| `ESCNO` | layout letter | `A`→1 … `Z`→26, `OA`→27 |
| `MCU_TYPE` | `MCU` | `L`→0, `H`→1 (BB21), `X`→2 (BB51) |
| `DEADTIME` | `DEADTIME` | passed as-is |
| `PWM_FREQ` | `PWM` | `24`→0, `48`→1, `96`→2 |

## What the pico-esc-tool CLI flash actually writes (verified in source)

There are **two** flash code paths in pico-esc-tool, and they differ — the docs describe
the CLI one, which is what you actually run:

- **`host/esctool.py flash` (the CLI — what these docs use):** writes the **app region
  `[0x0000,0x1A00)` AND the config/identity page `0x1A00–0x1BFF`**; only the bootloader
  (`≥0x1C00`) is preserved. `parse_hex` splits the HEX into app/ident/boot
  (`host/esctool.py` ~:297,:302), `_pages_from` assembles the app pages **plus** the
  `0x1A00` config page from the HEX's identity section (~:337-348), and `cmd_flash`
  erases+writeflash+verifies every one of those pages (~:379-392), then prints
  `"firmware default config applied"` (~:394). So each flash **resets the ESC's EEPROM
  params to the firmware's defaults** and **overwrites the stored `0x1A40` tag** with the
  firmware's `#<L>_<M>_<DT>#`.
- **`lib/esc_flash` (the C++ HexImage path):** *that* one is app-region-only
  (`kAppEnd = 0x1A00`, ident captured for the compat check but never flashed). The CLI
  does not use it for programming, so do not rely on its "settings preserved" property.

### Invariants the build MUST preserve (for the harness)

1. **Identity tags present in the hex.** The Ohx51 output must contain the `0x1A40` layout
   tag (`#<L>_<M>_<DT>#`) and the `0x1A50` MCU tag (`#BLHELI$EFM8B21#`) — the CLI both
   *reads* them for the compat check and *writes* the layout tag into the config page.
   Keep `MCU=H` and the correct `LAYOUT`/`DEADTIME`.
2. **Compat guard (CLI).** `host/esctool.py flash` (~:355-372) compares the ESC's
   **currently stored** `0x1A40` tag against the firmware's tag (byte-exact) and derives
   the MCU signature from `0x1A50`. `--force` is needed only for the **first** flash off a
   stock ESC whose stored tag differs from `#<L>_H_<DT>#` — because that flash then
   *overwrites* the stored tag with `#<L>_H_<DT>#`, every subsequent flash of the same
   target passes un-forced. (See the target README.)
3. **Do not reorder EEPROM params / bump the layout revision without a controller
   branch.** Upstream `EEPROM_LAYOUT_REVISION = 208` (in `vendor/bluejay/src/Bluejay.asm`).
   Any new BlueGill param is *appended* (never reordered) and needs a matching decode
   branch in pico-esc-tool `lib/esc_setup`. Not relevant to iteration 1 (no param changes).

## KNOWN LIMITATION / TODO — overlay vs submodule bump

BlueGill's whole-file `src/` overlays (none ship yet) replace an entire vendor module. If
we later bump the `vendor/bluejay` submodule, an upstream fix to a module we overlay is
**silently shadowed** — `diff-vendor.sh` only diffs against the *current* pin and records
no base commit. **Before adding any `src/` overlay:** record the vendor base commit the
overlay was forked from, and on every submodule bump 3-way re-merge each overlay against
its recorded base. (No code this pass — zero overlays exist.)

## Byte-compare against the official v0.21.0 release

Milestone 1: prove the unmodified LittleBee target rebuilds identically to the published
asset.

1. Build the exact target: `./tools/build/build.sh -l <L> -m H -d <DT> -p <PWM> -s v0.21.0`.
2. Download the matching asset from the BlueJay v0.21.0 release
   (<https://github.com/bird-sanctuary/bluejay/releases/tag/v0.21.0>); its name follows the
   same scheme, `<L>_H_<DT>_<PWM>_v0.21.0.hex`.
3. Compare. Intel-HEX is line-oriented ASCII, so normalise line endings first:

       # semantic byte compare (order/record-length independent)
       diff <(objcopy -I ihex -O binary ours.hex   /dev/stdout | xxd) \
            <(objcopy -I ihex -O binary theirs.hex /dev/stdout | xxd)
       # or a plain text diff after CRLF strip:
       diff <(tr -d '\r' < dist/BlueGill_<L>_H_<DT>_<PWM>_v0.21.0.hex) \
            <(tr -d '\r' < <L>_H_<DT>_<PWM>_v0.21.0.hex)

   Expect byte-identical, or a documented benign diff (e.g. record ordering / EOF record).
   No BlueGill tuning patches land until this passes.

## Status in this environment (native Ubuntu 24.04)

- `wine` is available via apt but was **not installable here** (no passwordless sudo in
  the build sandbox) and the **Keil eval assembler requires a manual, account-gated
  download** that cannot be scripted. So the baseline hex was **not** reproduced in this
  environment. `build.sh` is complete and was exercised up to the toolchain gate (vendor
  staging + overlay + DEFINE derivation verified); it is ready to run once wine + Keil are
  installed per the steps above. See the dev report for details.
