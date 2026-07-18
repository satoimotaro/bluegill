# Telemetry — BlueGill

BlueGill inherits BlueJay's **bidirectional DShot** telemetry path unchanged. This doc
records the eRPM path, which **Extended DShot Telemetry (EDT)** frames this board actually
emits, and the pole-pair conversion the controller applies.

> Verified against the pinned vendor at `vendor/bluejay/src/Modules/DShot.asm` and
> `Modules/Scheduler.asm` (tag v0.21.0). Iteration 1 changes nothing here.

## Bidirectional DShot eRPM path

- The controller (pico-esc-tool) must drive **bidirectional DShot** (`arm <i> bidir`).
  BlueJay supports **DShot300 and DShot600 only — not DShot150**; confirm the controller
  is on 300/600.
- After each valid frame, the ESC replies on the same wire with the motor's electrical
  period. In `Modules/DShot.asm`: `dshot_tlm_create_packet` derives the e-period from
  `Comm_Period4x`, `dshot_12bit_encode` packs it (exponent+mantissa), `dshot_gcr_encode`
  GCR-encodes it for the return line. pico-esc-tool decodes this to eRPM
  (`pico-bidir-dshot`) and it is the primary low-speed feedback signal.

## EDT frames this board emits

EDT is enabled by DShot command **13** (`CMD_EXTENDED_TELEMETRY_ENABLE`) and disabled by
**14**. `Modules/Scheduler.asm` interleaves these frame types (each identified by a high-
byte frame ID) into spare scheduler slots:

| Frame | ID | Content | Emitted? |
|---|---|---|---|
| eRPM | (normal bidir reply) | electrical period → eRPM | **yes** (always, bidir) |
| Temperature | `0x02` | MCU ADC temperature; also drives PWM-limit throttling toward the temp setpoint (ramped gradually to avoid current spikes) | **yes** |
| Status | `0x0E` | demag / desync / stall flags | **yes** |
| Demag metric | `0x0C` | `Demag_Detected_Metric` (commutation quality) | **yes** |
| Debug 1 / Debug 2 | `0x08` / `0x0A` | dev debug values | yes (dev) |
| **Voltage** | `0x04` | — | **NO frame in firmware** |
| **Current** | `0x06` | — | **NO frame in firmware** |

So on this class of board expect **eRPM + temperature + status (+ demag)** and nothing
else. `Scheduler.asm` has no voltage/current frame code.

### Voltage / current — bench-check TODO

The LittleBee Spring 30A (BB21) almost certainly has **no voltage/current sense divider
routed to an ADC input**, so there is nothing to report even if a frame existed.

- **TODO (bench):** inspect the board for a battery-voltage divider / current-shunt into
  an EFM8BB21 ADC channel.
- If — and only if — a divider exists, adding an EDT voltage frame (type `0x04`) in a
  spare scheduler slot is a small, optional, self-contained patch (a late-phase item).
  Do not promise voltage/current telemetry until the sense line is confirmed.

## Pole-pair conversion (mech RPM ↔ eRPM)

Firmware and DShot work in **electrical** RPM (pole-agnostic). The controller converts to
mechanical RPM using the motor pole count:

    pole_pairs      = MOTOR_POLES / 2
    mechanical_RPM  = eRPM / pole_pairs
    eRPM            = mechanical_RPM * pole_pairs

pico-esc-tool holds the pole count in `src/apps/esc_config.h` (`MOTOR_POLES`). Record the
actual thruster's pole count in `targets/littlebee-spring-30a/target.env` so both sides
agree; a wrong pole count only mis-scales the *displayed* mechanical RPM, not the control.

## Interaction with the future eRPM cap

The planned hard `Pgm_Max_Erpm` safety ceiling (a later patch) works in **eRPM**, so it is
pole-agnostic; the controller converts a mechanical-RPM limit to eRPM when writing the
param. See `docs/low-speed-tuning.md` for where the cap reuses the temperature PWM-limit
mechanism. Not implemented in iteration 1.
