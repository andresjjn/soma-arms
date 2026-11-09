# SOMA's eyes

Firmware for a **Waveshare ESP32-S3-DualEye-Touch-LCD-1.28**: one
ESP32-S3 (8 MB PSRAM, 16 MB flash) driving two round 240x240 GC9A01A
LCDs, with CST816 touch on each and an ES8311/ES7210 audio pair. Off the
v0.3 to v1.0 ladder (see `docs/plan.md`, section 8) and off the safety
path: the eyes cannot arm or command anything.

## How it works

- Both panels share one SPI bus (clock, data and D/C); a chip select each.
- The eye is math, not sprites (`eyes/eye_model.h`): gaze, pupil, blink
  and mood decide every pixel, so the gaze moves continuously.
- Drawn in stripes with two DMA buffers: one stripe goes out over SPI
  while the CPU computes the next.
- Fast path (`eyes/fast_render.h`): color tables rebuilt only when the
  design changes, lids per column, no square root per pixel; checked
  against the reference `eye_model.h` on 864 states (0 px off). A still
  eye is not redrawn; a gaze-only change redraws just the box around the
  iris. Measured on the board 2026-11-09: about 50 fps while the gaze
  moves, about 34 fps for full redraws (blinks, moods); about 5 to 8
  before (the reference model per pixel, every frame). `stats` reports it.
- Idle life of its own: small saccades and blinks; a command pauses it.
- Commands are lines over the USB serial port (`eyes/protocol.h`):

| Line | Effect |
|---|---|
| `ping` | answers `pong soma-eyes 0.1` |
| `look X Y` | gaze, each -1..1 (X toward the picture's right) |
| `blink` | one blink |
| `mood NAME` | neutral, happy, sad, angry, surprised, sleepy |
| `pupil S` | pupil scale 0.5..1.5 |
| `bright N` | backlight 0..100 |
| `idle on/off` | wandering gaze and blinks of their own |
| `set KEY VALUE` | one design field, live: `set iris_r 58`, `set colors.iris_in #ff8800`, `set happy.low 80` |
| `style` | the whole live design as JSON, in `style.json`'s shape |
| `stats` | frames drawn and skipped since the last `stats`, and the rate |
| `save` / `reset` | keep the live design in NVS across reboots / back to `style.json` |

Anything else answers `err <why>`.

## Changing the design

How the eyes look is data, in one place: **`style.json`** (sizes, colors,
the lid shape of each mood, blink and saccade timing). Never edit the
numbers in C++.

1. Preview on the Mac, with the board's own eye model compiled for the
   host: `python3 firmware/eyes/tools/preview.py [KEY=VALUE ...]` writes
   `firmware/eyes/preview.png` (rows: moods; columns: gaze and a half blink).
2. Tune it live on the board: `set KEY VALUE`, then `save`.
3. Bring it home: `style` prints the JSON; paste it into `style.json`, run
   `python3 firmware/eyes/tools/gen_style.py`, commit. A test fails if the
   JSON and the generated `eyes/style_default.h` ever disagree.

## Pins

From Waveshare's own examples (waveshareteam/ESP32-S3-DualEye-Touch-LCD-1.28,
Arduino `01_LCD_Driver` and `02_Drawing_board`):

| Signal | Eye 1 | Eye 2 |
|---|---|---|
| SPI SCLK / MOSI / MISO | 41 / 42 / 40 (shared) | shared |
| CS | 47 | 38 |
| D/C | 45 (shared) | 45 |
| RST | 48 | 8 |
| Backlight (PWM) | 46 | 39 |
| Touch I2C SDA / SCL | 11 / 10 (bus 0) | 3 / 2 (bus 1) |
| Touch INT / RST | 5 / 4 | 7 / 6 |

## Build and flash

```bash
arduino-cli compile --fqbn "$FQBN" firmware/eyes/eyes
arduino-cli upload --fqbn "$FQBN" -p /dev/cu.usbmodem11101 firmware/eyes/eyes
```

with the esp32 core 3.2.0 and the FQBN in `firmware/eyes/fqbn.txt`.

## Tests

`eye_model.h` and `protocol.h` have no hardware in them; `test/test_eyes.cpp`
builds them for the host and `soma_driver/test/test_eyes_firmware.py`
runs it with the rest of the suite.

## The factory firmware

Backed up whole (16 MB) and verified against the board on 2026-11-09,
outside this repository (vendor firmware). It looks like Waveshare's
xiaozhi voice assistant: a 3 MB app and a 5.9 MB speech-model partition.

`esp_lcd_gc9a01.c/.h` are Espressif's GC9A01 driver, Apache-2.0, kept with
their license header.
