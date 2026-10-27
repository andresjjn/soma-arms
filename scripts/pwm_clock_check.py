#!/usr/bin/env python3
"""Measure a PCA9685's real PWM frequency and pulse width with the Jetson.

No meter, no scope: the Jetson's GPIO timestamps every edge in the kernel
(gpiomon, nanoseconds), which is plenty for a 50 Hz signal. The boards'
logic runs on the Jetson's 3.3 V (docs/wiring.md), so a PWM output can go
straight into a header GPIO.

Why: the joint probe of 2026-10-26 read every right arm joint (board 0x40)
at 0.81 to 0.87 of its commanded angle and the left arm (0x43) at 0.87 to
1.05. A board whose internal oscillator runs fast shortens every pulse by
the same factor. Prescale 121 assumes 25 MHz and was never measured.

    python3 scripts/pwm_clock_check.py --board 0x40
    python3 scripts/pwm_clock_check.py --board 0x43

Wiring, once per board: one jumper from the PWM pin of ANY free channel of
that board to header pin 29 (GPIO PQ.05, gpiochip0 line 105). Ground is
already common through the I2C harness. Every free channel of the board
gets the same test pulse, so which one the jumper sits on does not matter
(the boards have no channel silkscreen, never count pins).

Safety: it refuses while the soma_driver container is running (two
processes on one bus can misdirect writes, see servo_workbench.py). It
first puts EVERY output of the board in FULL_OFF, then drives only the
channels SERVO_MAP leaves free (and not channel 0, out of service). No
servo gets a pulse. Every output goes back to FULL_OFF on the way out.

Only the pure helpers sit at module level, so the tests import this file
without smbus2 or a Jetson.
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'soma_driver'))
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

MODE1, PRESCALE, LED0, ALL_LED_OFF_H = 0x00, 0xFE, 0x06, 0xFD
PRESCALE_VALUE = 121               # what the driver and the workbench write
NOMINAL_OSC_HZ = 25_000_000
OUT_OF_SERVICE = {0}               # channel 0: suspected in a fault, never cleared
CHIP, LINE = 'gpiochip0', 105      # header pin 29, PQ.05
TEST_US = 1500.0
EVENTS = 400                       # 200 periods, 4 s


def free_channels(address: int) -> list[int]:
    """Channels of this board that no joint owns, channel 0 excluded."""
    used = {s.channel for s in SERVO_MAP.values() if s.address == address}
    return [c for c in range(16) if c not in used and c not in OUT_OF_SERVICE]


def counts_for_us(us: float, prescale: int = PRESCALE_VALUE,
                  osc_hz: float = NOMINAL_OSC_HZ) -> int:
    """The 12-bit OFF count the board needs for `us`, if its clock is nominal."""
    period_us = 1e6 * 4096 * (prescale + 1) / osc_hz
    return round(us / period_us * 4096)


def parse_gpiomon(text: str) -> list[tuple[int, int]]:
    """'%e %s %n' lines to [(edge, t_ns)], edge 1 rising, 0 falling."""
    events = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 3 and all(p.isdigit() for p in parts):
            events.append((int(parts[0]), int(parts[1]) * 1_000_000_000 + int(parts[2])))
    return events


def analyze(events: list[tuple[int, int]], commanded_us: float,
            prescale: int = PRESCALE_VALUE) -> dict | None:
    """Frequency, pulse width and what they say about the oscillator."""
    rises = [t for e, t in events if e == 1]
    periods = [b - a for a, b in zip(rises, rises[1:])]
    widths = []
    for i, (e, t) in enumerate(events[:-1]):
        nxt_e, nxt_t = events[i + 1]
        if e == 1 and nxt_e == 0:
            widths.append(nxt_t - t)
    if len(periods) < 10 or len(widths) < 10:
        return None
    period_us = sorted(periods)[len(periods) // 2] / 1000    # median: robust to a lost edge
    width_us = sorted(widths)[len(widths) // 2] / 1000
    freq = 1e6 / period_us
    osc = freq * 4096 * (prescale + 1)
    return {'frequency_hz': round(freq, 3), 'period_us': round(period_us, 1),
            'width_us': round(width_us, 1), 'commanded_us': commanded_us,
            'width_ratio': round(width_us / commanded_us, 4),
            'oscillator_mhz': round(osc / 1e6, 3),
            'oscillator_error_pct': round(100 * (osc / NOMINAL_OSC_HZ - 1), 2),
            'periods': len(periods)}


def driver_running() -> bool:
    try:
        out = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'],
                             capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return 'soma_driver' in out.split()


def all_off(bus, address: int) -> None:
    # One byte, so it works whatever MODE1's auto-increment says.
    bus.write_byte_data(address, ALL_LED_OFF_H, 0x10)               # FULL_OFF, every output


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--board', required=True, type=lambda s: int(s, 0), choices=(0x40, 0x43))
    ap.add_argument('--bus', type=int, default=7)
    ap.add_argument('--us', type=float, default=TEST_US)
    ap.add_argument('--events', type=int, default=EVENTS)
    args = ap.parse_args(argv)
    if driver_running():
        print('REFUSED: the soma_driver container is running. Disarm, then '
              '`docker stop soma_driver`, then run this again.', file=sys.stderr)
        return 3
    channels = free_channels(args.board)
    count = counts_for_us(args.us)
    print(f'board 0x{args.board:02x}: test pulse {args.us:g} us (OFF count {count}) on '
          f'free channels {channels}; every other output FULL_OFF. '
          f'Jumper from any of them to header pin 29.', flush=True)

    from smbus2 import SMBus
    with SMBus(args.bus) as bus:
        try:
            all_off(bus, args.board)
            bus.write_byte_data(args.board, MODE1, 0x10)              # sleep to set prescale
            bus.write_byte_data(args.board, PRESCALE, PRESCALE_VALUE)
            bus.write_byte_data(args.board, MODE1, 0x20)              # wake, auto-increment
            for c in channels:
                bus.write_i2c_block_data(args.board, LED0 + 4 * c,
                                         [0, 0, count & 0xFF, count >> 8])
            done = subprocess.run(
                ['gpiomon', f'--num-events={args.events}', '--format=%e %s %n', CHIP, str(LINE)],
                capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            print('no edges on pin 29 in 15 s: is the jumper on the PWM row of a '
                  'free channel, and is the board awake?', file=sys.stderr)
            return 1
        finally:
            all_off(bus, args.board)
    result = analyze(parse_gpiomon(done.stdout), args.us)
    if result is None:
        print('too few clean edges to judge:\n' + done.stdout[-400:], file=sys.stderr)
        return 1
    print(result)
    print(f"verdict: the board runs at {result['frequency_hz']} Hz (nominal 50.03) and "
          f"stretches pulses by {result['width_ratio']} (1.0 = exact); its oscillator "
          f"is about {result['oscillator_mhz']} MHz ({result['oscillator_error_pct']:+.1f} %).")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
