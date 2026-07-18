# BlueGill startup-melody editor

Design the BlueGill startup jingle by ear on the PC, then paste the exact bytes into firmware.
Pure stdlib — the GUI runs in your browser (Web Audio for sound), no tkinter/audio libs.

```
python3 melody_editor.py                 # opens the editor in your browser (http://127.0.0.1:8770)
python3 melody_editor.py --decode "2,58,4,32,94,51,13,0,..."   # print a byte-melody as REAL notes
python3 melody_editor.py --port 8771
```

## Accurate preview (what you hear ≈ what the ESC makes)
The preview uses the REAL Bluejay beep timing derived from `Modules/Fx.asm beep` + the code's own
calibration (`150 djnz = 25 µs`):

    period_us = FIXED_US + SLOPE_US·pitch        (defaults 406.7 and 33.6 at beep_strength=40)
    freq_hz   = 1e6 / period_us                  (LOWER pitch byte ⇒ HIGHER note)
    dur_ms    = pulses · period_us / 1000

Because of the fixed per-pulse overhead, the ESC can only make **~111–2271 Hz** — notes above/below
that are flagged and clamped in the UI, and durations are quantized to whole pulses (min ≈ one
period, max 255 pulses). This is why a naïve `freq = K/pitch` model (my first version) sounded wrong:
it ignored the overhead and let you pick notes the ESC can't produce. Tweak `FIXED_US`/`SLOPE_US`
in the UI to match your ESC by ear if the tone is off.

## Features
- **Compose**: pick note (name+octave) + duration, **▶ Play** (square wave), reorder ↑/↓, delete.
- **Insert anywhere**: click a row to set the insertion point (▶); +note/+rest insert after it.
- **Presets + save/load**: built-in tunes (chime, power-on ding, fanfare, the current "under the
  sea") plus your **saved** melodies (★) — *save as…* stores to `melodies/<name>.json` and it
  appears in the dropdown.
- **Export**: live `Eep_Pgm_Beep_Melody: DB …` bytes — copy and paste over that line in
  `src/Bluejay.asm`, rebuild (`build.sh -l A -m H -d 30 -p 24`), reflash, power-cycle to hear it.

Saved melodies in `melodies/` are gitignored (personal); `git add -f` a good one to keep it.
