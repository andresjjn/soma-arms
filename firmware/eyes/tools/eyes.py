#!/usr/bin/env python3
"""Talk to SOMA's eyes over USB serial: each argument is one command line.

    .venv/bin/python firmware/eyes/tools/eyes.py ping
    .venv/bin/python firmware/eyes/tools/eyes.py "set neutral.up 0" "set neutral.low 0" save
    .venv/bin/python firmware/eyes/tools/eyes.py style
    .venv/bin/python firmware/eyes/tools/eyes.py --tour        # every mood, 1.5 s each
    .venv/bin/python firmware/eyes/tools/eyes.py --story       # a 20 s take for a video

The port defaults to the first /dev/cu.usbmodem* (the board's native USB).
Commands are in firmware/eyes/README.md. Needs pyserial.
"""
import argparse
import glob
import sys
import time

TOUR = ('mood happy', 'mood sad', 'mood angry', 'mood surprised', 'mood sleepy', 'blink',
        'look -0.8 0', 'look 0.8 0', 'look 0 -0.7', 'look 0 0.6', 'mood neutral', 'look 0 0')

# A short take with a story: it wakes up, looks around, finds the camera,
# shows what it feels, and ends happy, looking at you. (command, seconds held)
STORY = (
    ('idle off', 0.0), ('mood sleepy', 0.0), ('look 0 0.35', 1.8),
    ('blink', 0.5), ('mood neutral', 0.3), ('look 0 0', 0.7),
    ('look -0.8 0.05', 0.9), ('look 0.8 0.05', 0.9), ('look 0 0', 0.4),
    ('mood surprised', 1.3), ('blink', 0.4), ('mood happy', 1.8),
    ('mood neutral', 0.0), ('look 0.55 -0.6', 1.2),
    ('look 0 0', 0.3), ('mood sad', 1.6), ('mood angry', 1.6),
    ('blink', 0.4), ('mood happy', 0.0), ('look 0 0', 2.5),
    ('mood neutral', 0.0), ('idle on', 0.0),
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('commands', nargs='*')
    ap.add_argument('--port')
    ap.add_argument('--tour', action='store_true', help='show every mood and gaze')
    ap.add_argument('--pause', type=float, default=1.5, help='seconds between --tour steps')
    ap.add_argument('--story', action='store_true', help='a 20 s take for a video, after a countdown')
    args = ap.parse_args(argv)
    import serial
    port = args.port or next(iter(sorted(glob.glob('/dev/cu.usbmodem*'))), None)
    if port is None:
        print('no /dev/cu.usbmodem* found: is the board plugged in?', file=sys.stderr)
        return 2
    if args.story:
        return story(port)
    commands = list(TOUR) if args.tour else args.commands
    if not commands:
        commands = ['ping']
    with serial.Serial(port, 115200, timeout=0.5) as s:
        time.sleep(0.3)
        drain(s)
        for c in commands:
            s.write((c + '\n').encode())
            reply = s.readline().decode(errors='replace').strip()
            print(f'{c:28} -> {reply}')
            if args.tour:
                time.sleep(args.pause)
    return 0


def drain(s, quiet_s: float = 0.15, limit_s: float = 1.5) -> None:
    """Discard what the board left in the buffers (stale audio frames after a
    stream ended with the port closed) until it has been quiet for quiet_s."""
    old, s.timeout = s.timeout, quiet_s
    end = time.monotonic() + limit_s
    while time.monotonic() < end and s.read(65536):
        pass
    s.timeout = old


def story(port: str) -> int:
    import serial
    with serial.Serial(port, 115200, timeout=0.5) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for c in ('idle off', 'mood neutral', 'look 0 0'):
            s.write((c + '\n').encode())
            s.readline()
        for n in (3, 2, 1):
            print(f'recording in {n}...', flush=True)
            time.sleep(1)
        print('go', flush=True)
        for c, hold in STORY:
            s.write((c + '\n').encode())
            s.readline()
            time.sleep(hold)
    print('done')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
