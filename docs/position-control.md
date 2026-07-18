# Position control — architecture decision

**Decision (checkpoint 2026-07-13): encoder-based position control is read and closed on
the CONTROLLER (pico-esc-tool), NOT on the ESC. BlueGill firmware stays sensorless.**

## The split

```
  encoder ──► pico-esc-tool (RP2040 controller)
                 │  reads position, runs the position loop
                 │  commands thrust/velocity over DShot ─────► BlueGill ESC
                 │                                              (sensorless BLDC drive)
                 └──◄ eRPM / EDT telemetry ◄─────────────────────────┘
```

- The **ESC (BlueGill)** remains a sensorless BLHeli-S/BlueJay-style drive. It does **no**
  encoder handling, no position loop. This keeps the 8051 firmware small, keeps us close
  to upstream BlueJay (cheap rebasing), and avoids burdening the commutation-critical code
  with sensor I/O.
- The **controller (pico-esc-tool)** owns the encoder input and closes the position loop,
  commanding velocity/thrust to the ESC over DShot. It already reads per-thruster eRPM
  telemetry, so it has the feedback to combine encoder position with commutation rate.

## Why this way

- Sensorless commutation at the very low RPM a position servo needs is exactly the hard,
  possibly-unreachable regime (see the 60–240 RPM stretch in `docs/low-speed-tuning.md`).
  An encoder on the controller side is the honest, robust path to precise slow/holding
  motion — it does not depend on BEMF being detectable.
- Firmware stays generic and upstream-compatible; the "cheap BLDC servo" capability lives
  where the sensor and the higher-level control already are.

## Phasing

This is a **late phase (B2/B3)**. Iteration 1 does nothing here — the decision is recorded
so BlueGill firmware work is not scoped to include encoder handling. The firmware's job for
the servo use-case is only to (a) run smoothly at low RPM and (b) accept fast velocity/
thrust commands with the (future) accel-slew limiter — both already covered by the
low-speed and robustness tracks.
