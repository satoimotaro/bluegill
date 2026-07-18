# BlueGill

Digital ESC firmware for **low-KV brushless thrusters** — a hard fork of
[**Bluejay**](https://github.com/bird-sanctuary/bluejay) `v0.21.0` (BLHeli-S + bidirectional DShot) for
SiLabs **EFM8BB2x (8051)** ESCs, tuned for **underwater ROV thrusters**.

Bluejay (and BLHeli-S) target multirotor motors, which spin fast; a low-KV thruster spends its life at
low RPM where stock 6-step commutation loses BEMF lock. BlueGill adds a low-speed drive path and makes
the telemetry/stop behaviour usable for closed-loop **velocity control from a host** (no flight
controller). Its companion host tool is [**pico-esc-tool**](https://github.com/satoimotaro/pico-esc-tool)
(an RP2040 that configures, flashes, and drives these ESCs over DShot).

## What BlueGill adds over Bluejay v0.21.0

- **Forced-sine low-speed drive** (`Pgm_Sine_Mode`) — open-loop sinusoidal commutation that runs the
  motor smoothly below the speed where 6-step BEMF locks.
- **Automatic sine ↔ 6-step crossover** — the ESC climbs out of sine into BEMF 6-step at `Cross_Up`
  and (attempts to) hand back at `Cross_Dn`, with a smooth-handoff duty rescale so thrust→speed stays
  continuous across the seam.
- **Continuous virtual-eRPM telemetry in sine** — DShot eRPM is reported from the forced field rate
  while in sine, so a host velocity loop stays closed across the whole 0→sine→6-step range.
- **Proper-neutral armed stop** — a magnitude-based neutral detection parks the ESC in `wait_for_start`
  (armed, signal alive) so a `rpm 0` really stops the rotor and the next command restarts warm — instead
  of limping at the weak-BEMF floor as a stock 3D ESC does at DShot 0.

Every change is gated on `Flag_Sine_Mode`, so with sine mode **off** the firmware's runtime behaviour is
the stock Bluejay 6-step path. See the design notes in [`docs/`](docs/).

## Relationship to upstream Bluejay (and how to see the diff)

This repo is a **hard fork**: the git history is Bluejay's, up to `v0.21.0`, which is tagged
**`v0.21.0-base`**. Everything after that tag is BlueGill. To see exactly what the firmware changes:

```
git diff v0.21.0-base -- src/      # or: tools/build/diff-vendor.sh
```

Modified: `src/Bluejay.asm`, `src/Modules/{Settings,Power,Isrs,Fx,Timing}.asm`; new: `src/Modules/SineMode.asm`.
To pull upstream fixes later: `git remote add upstream https://github.com/bird-sanctuary/bluejay` then
cherry-pick / merge.

## Prebuilt images

Ready-to-flash hex for the reference ESC (LittleBee Spring 30A, EFM8BB21, **Layout A**, deadtime 30) are
committed in [`prebuilt/`](prebuilt/) so you don't have to set up the toolchain:

- `BlueGill_A_H_30_48_v0.21.0.hex` — **48 kHz**, the hardware-verified image (closed-loop velocity +
  neutral-stop bench runs).
- `BlueGill_A_H_30_24_v0.21.0.hex` — **24 kHz**, `target.env`'s documented thruster default (more duty
  resolution at low speed); bench-verify before relying on it.

Both are provided — pick the PWM you want. See [`prebuilt/README.md`](prebuilt/README.md) for sha256 and
flashing. **Layout A only:** the sine drive is hardwired to Layout A's pin order (`SineMode.asm` refuses
to assemble on other layouts to avoid a wrong-FET image), so other layouts must build from source after
porting the sine commutation.

## Build

8051 assembly built with the **Keil C51** toolchain (`AX51`/`LX51`/`Ohx51`) under **Wine** — same as
upstream Bluejay (a generic SDCC build will not work). The Keil eval install is a one-time manual step;
see [`tools/build/README.md`](tools/build/README.md).

```
tools/build/build.sh -l A -m H -d 30 -p 48
# -> dist/BlueGill_A_H_30_48_v0.21.0.hex   (LAYOUT_MCU_DEADTIME_PWM_VERSION)
```

`LAYOUT`/`DEADTIME` are read off the hardware; defaults for the reference ESC live in
[`targets/littlebee-spring-30a/`](targets/littlebee-spring-30a/). Flash the app-only `.hex` with the
pico-esc-tool (`esctool.py flash`); the bootloader is never overwritten.

## License

**GPL-3.0-or-later**, inherited from Bluejay — see [`COPYING`](COPYING). BlueGill is a derivative work of
[bird-sanctuary/bluejay](https://github.com/bird-sanctuary/bluejay) (itself based on
[BLHeli_S](https://github.com/bitdump/BLHeli)); all original copyrights are retained. Firmware `.hex`
images are not distributed here.
