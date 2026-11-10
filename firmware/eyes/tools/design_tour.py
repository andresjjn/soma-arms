#!/usr/bin/env python3
"""Show eye design proposals on the real panels, as still pictures.

Takes contact sheets where each eye is drawn at panel size (a circle of
about 240 px), in rows of four pairs (left eye, right eye), and sends each
eye to its panel with the firmware's `img` command. Nothing is implemented
on the board: it only displays what the sheet shows. `live` brings the
animated eyes back.

    .venv/bin/python firmware/eyes/tools/design_tour.py \\
        --sheet "APERTURA=/path/A.png" --sheet "HALO=/path/B.png"

The sheets of 2026-11-09 (Claude Design, rendered with headless Chrome at
2352x1440) hold three rows: neutral, happy, sad, angry / surprised,
sleepy, thinking, focused / gaze left, right, up, down. The tour starts
with the current design, animated, for comparison.
"""
import argparse
import glob
import sys
import time

LABELS = ('neutral', 'happy', 'sad', 'angry', 'surprised', 'sleepy', 'thinking', 'focused',
          'gaze left', 'gaze right', 'gaze up', 'gaze down')
SIZE = 240
CURRENT = (('mood neutral', 1.2), ('mood happy', 1.4), ('mood sad', 1.4), ('mood angry', 1.4),
           ('mood surprised', 1.4), ('mood sleepy', 1.4), ('mood neutral', 0.2), ('look -0.8 0', 0.8),
           ('look 0.8 0', 0.8), ('look 0 0', 0.6))


def circles(png_path):
    """Each eye of a sheet as a SIZE x SIZE RGB image, in reading order."""
    import numpy as np
    from PIL import Image
    img = Image.open(png_path).convert('RGB')
    a = np.asarray(img).astype(int)
    content = np.abs(a - a[5, 5]).sum(axis=2) > 18

    def runs(mask_1d, min_len):
        idx = np.where(mask_1d)[0]
        out, start, prev = [], idx[0], idx[0]
        for i in idx[1:]:
            if i != prev + 1:
                out.append((start, prev))
                start = i
            prev = i
        out.append((start, prev))
        return [r for r in out if r[1] - r[0] > min_len]

    rows = runs(content.sum(axis=1) > 200, 150)
    cols = runs(content[rows[0][0]:, :].sum(axis=0) > 150, 150)
    eyes = []
    for r0, r1 in rows:
        for c0, c1 in cols:
            d = c1 - c0 + 1
            cx, cy = (c0 + c1) / 2, (r0 + r1) / 2
            box = (int(round(cx - d / 2)), int(round(cy - d / 2)), int(round(cx + d / 2)), int(round(cy + d / 2)))
            eyes.append(img.crop(box).resize((SIZE, SIZE), Image.LANCZOS))
    return eyes


def rgb565(image) -> bytes:
    import numpy as np
    a = np.asarray(image).astype(np.uint16)
    v = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
    return v.astype('>u2').tobytes()          # high byte first, as the panel takes it


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--sheet', action='append', default=[], help='NAME=path.png, in tour order')
    ap.add_argument('--hold', type=float, default=1.8, help='seconds per picture')
    ap.add_argument('--port')
    args = ap.parse_args(argv)
    import serial
    port = args.port or next(iter(sorted(glob.glob('/dev/cu.usbmodem*'))), None)
    if port is None:
        print('no /dev/cu.usbmodem* found', file=sys.stderr)
        return 2
    sheets = []
    for spec in args.sheet:
        name, _, path = spec.partition('=')
        eyes = circles(path)
        pairs = [(eyes[i], eyes[i + 1]) for i in range(0, len(eyes) - 1, 2)]
        sheets.append((name, pairs))
        print(f'{name}: {len(pairs)} pairs from {path}')
    with serial.Serial(port, 115200, timeout=6) as s:
        time.sleep(0.3)
        s.reset_input_buffer()

        def send(line, payload=b''):
            s.write((line + '\n').encode())
            for i in range(0, len(payload), 4096):         # one block per `k` from the board
                if s.read(1) != b'k':
                    return 'err: the board stopped asking for blocks'
                s.write(payload[i:i + 4096])
            return s.readline().decode(errors='replace').strip()

        print('== current design (animated)')
        send('idle off')
        for c, hold in CURRENT:
            send(c)
            time.sleep(hold)
        for name, pairs in sheets:
            print(f'== {name}')
            for label, (left, right) in zip(LABELS, pairs):
                r0 = send('img 0', rgb565(left))
                r1 = send('img 1', rgb565(right))
                print(f'  {label:11} {r0} {r1}')
                time.sleep(args.hold)
        print('== back to the live eyes', send('live'), send('mood neutral'), send('idle on'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
