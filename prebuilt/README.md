# Prebuilt BlueGill images

Ready-to-flash `.hex` for the reference ESC — **LittleBee Spring 30A**, SiLabs **EFM8BB21**, **Layout A**,
deadtime **30**. Filename is `BlueGill_<LAYOUT>_<MCU>_<DEADTIME>_<PWM>_<VERSION>.hex`.

| File | PWM | Status |
|---|---|---|
| `BlueGill_A_H_30_48_v0.21.0.hex` | 48 kHz | **Hardware-verified** — this is the exact image used for the closed-loop velocity + proper-neutral-stop bench runs. |
| `BlueGill_A_H_30_24_v0.21.0.hex` | 24 kHz | `target.env`'s documented thruster default (more duty resolution at the tiny low-speed duties). Builds clean; **bench-verify before relying on it.** |

```
sha256  A_H_30_48 : 0648fc06896ca681869ca9582b3ea44d0133cf47a029ffd2ed20c7271df3e3c0
sha256  A_H_30_24 : 8f925f31eb508ac645471ae725a08fd34668b63a9163d8097b42fd2a8dba99d3
```

## Flash (via pico-esc-tool)

```
python host/esctool.py flash 1 BlueGill_A_H_30_48_v0.21.0.hex --yes
python host/esctool.py apply 1 host/profiles/rpm_930kv_sine2.yaml   # a flash resets config to defaults
```

Flashing is app-only (the bootloader is preserved) and the compat guard does a byte-exact check of the
ESC's stored layout tag (`#A_H_30#`) against the image — a mismatched ESC is refused unless `--force`.
After any flash, re-apply the full profile (a partial config won't spin the low-KV motor).

## Layout A only

BlueGill's sine drive is **Layout-A-specific**: the SVPWM commutation is hardwired to Layout A's PWM /
comparator pin order (and the BB1/BB2 comparator setup), and `SineMode.asm` *refuses to assemble* on any
other layout rather than emit a wrong-FET (shoot-through-capable) image. So there is no `J_H_30` etc.
here — a different layout needs the sine commutation ported to that pin map first.

To build other deadtimes/PWMs for Layout A: `tools/build/build.sh -l A -m H -d <dt> -p <24|48>`.
