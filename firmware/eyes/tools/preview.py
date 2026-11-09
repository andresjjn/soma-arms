#!/usr/bin/env python3
"""Preview SOMA's eyes on the Mac, with the board's own eye model.

Compiles tools/preview.cpp (which includes eyes/eye_model.h, the very file
the firmware draws with) and writes a contact sheet: one row per mood,
columns for the gaze and a half blink, both eyes in each cell.

    python3 firmware/eyes/tools/preview.py                       # the design in style.json
    python3 firmware/eyes/tools/preview.py iris_r=54 colors.iris_in=#ff8800
    python3 firmware/eyes/tools/preview.py --out /tmp/eyes.png

Overrides use the same keys as the board's `set`. Regenerates the default
header from style.json first, so the preview always shows the JSON.
"""
import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
EYES = HERE.parent


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('overrides', nargs='*', help='KEY=VALUE, as the board\'s `set`')
    ap.add_argument('--out', default=str(EYES / 'preview.png'))
    args = ap.parse_args(argv)
    cxx = shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
    if cxx is None:
        print('no C++ compiler found', file=sys.stderr)
        return 2
    subprocess.run([sys.executable, str(HERE / 'gen_style.py')], check=True, capture_output=True)
    with tempfile.TemporaryDirectory() as tmp:
        exe, ppm = Path(tmp) / 'preview', Path(tmp) / 'eyes.ppm'
        subprocess.run([cxx, '-std=c++17', '-O2', '-o', str(exe), str(HERE / 'preview.cpp')], check=True)
        run = subprocess.run([str(exe), str(ppm), *args.overrides], capture_output=True, text=True)
        if run.returncode:
            print(run.stderr.strip(), file=sys.stderr)
            return run.returncode
        out = Path(args.out)
        if shutil.which('sips'):                                 # macOS
            subprocess.run(['sips', '-s', 'format', 'png', str(ppm), '--out', str(out)],
                           check=True, capture_output=True)
        else:
            out = out.with_suffix('.ppm')
            shutil.copy(ppm, out)
    print(f'preview: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
