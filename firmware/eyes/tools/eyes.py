#!/usr/bin/env python3
"""Talk to SOMA's eyes over USB serial: each argument is one command line.

    .venv/bin/python firmware/eyes/tools/eyes.py ping
    .venv/bin/python firmware/eyes/tools/eyes.py "set neutral.up 0" "set neutral.low 0" save
    .venv/bin/python firmware/eyes/tools/eyes.py style
    .venv/bin/python firmware/eyes/tools/eyes.py --tour        # every mood, 1.5 s each

The port defaults to the first /dev/cu.usbmodem* (the board's native USB).
Commands are in firmware/eyes/README.md. Needs pyserial.
"""
import argparse
import glob
import sys
import time

TOUR = ('mood happy', 'mood sad', 'mood angry', 'mood surprised', 'mood sleepy', 'blink',
        'look -0.8 0', 'look 0.8 0', 'look 0 -0.7', 'look 0 0.6', 'mood neutral', 'look 0 0')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('commands', nargs='*')
    ap.add_argument('--port')
    ap.add_argument('--tour', action='store_true', help='show every mood and gaze')
    ap.add_argument('--pause', type=float, default=1.5, help='seconds between --tour steps')
    args = ap.parse_args(argv)
    import serial
    port = args.port or next(iter(sorted(glob.glob('/dev/cu.usbmodem*'))), None)
    if port is None:
        print('no /dev/cu.usbmodem* found: is the board plugged in?', file=sys.stderr)
        return 2
    commands = list(TOUR) if args.tour else args.commands
    if not commands:
        commands = ['ping']
    with serial.Serial(port, 115200, timeout=0.5) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for c in commands:
            s.write((c + '\n').encode())
            reply = s.readline().decode(errors='replace').strip()
            print(f'{c:28} -> {reply}')
            if args.tour:
                time.sleep(args.pause)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
