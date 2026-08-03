#!/usr/bin/env python3
"""Turn servo_calibration.json into a reviewed SERVO_MAP + URDF proposal.

Reads the zeros and mechanical limits captured with servo_workbench.py and
computes, per joint: the asymmetric joint limits in radians around the new
mechanical zero, the ServoSpec row that encodes them, and the URDF limit
values. It WRITES NOTHING into the code: it emits a markdown proposal to
review, because two things cannot come from the workbench alone:

  - SIGNS. The workbench captures pulses, not directions. Whether more
    microseconds means positive URDF rotation must be verified joint by
    joint on the hardware (it depends on how each servo sits in the
    frame). Every sign below defaults to +1 and is flagged UNVERIFIED.
  - The current xacro uses one shared symmetric limit for all arm joints.
    Asymmetric limits require per joint properties in soma_arm.xacro;
    that surgery happens at integration, with this report as input.

Conventions:
  - Arm joints: 500 to 2500 us over 180 degrees -> 2000/pi us per radian.
    zero_us becomes the joint's 0 rad.
  - Fingers: the captured zero is CLOSED (0.0), the captured limit
    farther from zero is OPEN (1.0).
  - Torso L16 (ch 3): meters, measured inverted (2000 us = retracted =
    0 mm). Captured stops get a 5 mm soft margin on each side, exactly
    like the current anchors, just measured instead of assumed.

Usage:
    python3 scripts/apply_calibration.py servo_calibration.json
    python3 scripts/apply_calibration.py servo_calibration.json -o out.md
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'soma_driver'))
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

US_PER_RAD = 2000.0 / math.pi     # 180 deg over 2000 us
L16_CH = 3
L16_US_RETRACTED, L16_US_EXTENDED = 2000.0, 1000.0
L16_STROKE_M = 0.140
L16_MARGIN_M = 0.005

# +1 means "more microseconds = positive URDF direction". EVERY entry is a
# placeholder until verified on hardware; the report repeats this loudly.
DEFAULT_SIGN = 1


def l16_us_to_m(us: float) -> float:
    return (L16_US_RETRACTED - us) / (L16_US_RETRACTED - L16_US_EXTENDED) * L16_STROKE_M


def l16_m_to_us(m: float) -> float:
    return L16_US_RETRACTED - m / L16_STROKE_M * (L16_US_RETRACTED - L16_US_EXTENDED)


def joint_for_channel(ch: int):
    for name, spec in SERVO_MAP.items():
        if spec.channel == ch:
            return name, spec
    return None, None


def propose(cal: dict) -> dict:
    """Compute the proposal. Returns {joint: row_dict}, ready to render."""
    rows = {}
    for ch_str, entry in sorted(cal.items(), key=lambda kv: -int(kv[0])):
        ch = int(ch_str)
        name, spec = joint_for_channel(ch)
        if name is None:
            rows[f'channel {ch}'] = {'error': 'channel not in SERVO_MAP'}
            continue
        zero = entry.get('zero')
        lo_us, hi_us = entry.get('min'), entry.get('max')
        missing = [k for k, v in (('zero', zero), ('min', lo_us), ('max', hi_us)) if v is None]
        if missing:
            rows[name] = {'channel': ch, 'error': f'missing {", ".join(missing)}'}
            continue
        lo_us, hi_us = min(lo_us, hi_us), max(lo_us, hi_us)

        if ch == L16_CH:
            # Measured stops in meters, then pull the soft band 5 mm in.
            stop_a, stop_b = sorted((l16_us_to_m(lo_us), l16_us_to_m(hi_us)))
            lower_m, upper_m = stop_a + L16_MARGIN_M, stop_b - L16_MARGIN_M
            rows[name] = {
                'channel': ch, 'kind': 'l16',
                'stops_m': (round(stop_a * 1000, 1), round(stop_b * 1000, 1)),
                'lower': round(lower_m, 4), 'upper': round(upper_m, 4),
                'min_us': round(l16_m_to_us(lower_m), 1),
                'max_us': round(l16_m_to_us(upper_m), 1),
                'max_rate': spec.max_rate,
            }
            continue

        if 'finger' in name:
            # zero = closed. The limit farther from zero is fully open.
            open_us = hi_us if abs(hi_us - zero) >= abs(lo_us - zero) else lo_us
            rows[name] = {
                'channel': ch, 'kind': 'gripper', 'zero_us': zero,
                'lower': 0.0, 'upper': 1.0,
                'min_us': float(zero), 'max_us': float(open_us),
                'max_rate': spec.max_rate,
            }
            continue

        sign = DEFAULT_SIGN
        a = sign * (lo_us - zero) / US_PER_RAD
        b = sign * (hi_us - zero) / US_PER_RAD
        lower, upper = min(a, b), max(a, b)
        rows[name] = {
            'channel': ch, 'kind': 'arm', 'zero_us': zero, 'sign': sign,
            'lower': round(lower, 4), 'upper': round(upper, 4),
            'lower_deg': round(math.degrees(lower), 1),
            'upper_deg': round(math.degrees(upper), 1),
            'min_us': float(lo_us if sign > 0 else hi_us),
            'max_us': float(hi_us if sign > 0 else lo_us),
            'max_rate': spec.max_rate,
        }
    return rows


def render(rows: dict) -> str:
    out = [
        '# Calibration proposal (generated, review before integrating)',
        '',
        'Signs default to +1 and are UNVERIFIED: confirm on hardware, per',
        'joint, that more microseconds moves the joint in the positive URDF',
        'direction. A wrong sign flips min_us/max_us (that inversion is',
        'supported, the L16 already uses it).',
        '',
        '## Summary',
        '',
        '| Joint | ch | zero us | range (rad) | range (deg) | note |',
        '|---|---|---|---|---|---|',
    ]
    for name, r in rows.items():
        if 'error' in r:
            out.append(f'| {name} | {r.get("channel", "?")} | - | - | - | ERROR: {r["error"]} |')
        elif r['kind'] == 'l16':
            out.append(f'| {name} | {r["channel"]} | n/a | {r["lower"]} to {r["upper"]} m '
                       f'| n/a | measured stops {r["stops_m"][0]} to {r["stops_m"][1]} mm, 5 mm margin |')
        elif r['kind'] == 'gripper':
            out.append(f'| {name} | {r["channel"]} | {r["zero_us"]} (closed) | 0.0 to 1.0 '
                       f'| n/a | open at {r["max_us"]} us |')
        else:
            out.append(f'| {name} | {r["channel"]} | {r["zero_us"]} | {r["lower"]} to {r["upper"]} '
                       f'| {r["lower_deg"]} to {r["upper_deg"]} | sign UNVERIFIED |')

    out += ['', '## SERVO_MAP rows (paste after review)', '', '```python']
    for name, r in rows.items():
        if 'error' in r:
            continue
        out.append(f"    '{name}': ServoSpec({r['channel']}, {r['min_us']}, "
                   f"{r['max_us']}, {r['lower']}, {r['upper']}, {r['max_rate']}),")
    out += ['```', '', '## URDF limits (per joint, for the xacro surgery)', '', '```']
    for name, r in rows.items():
        if 'error' in r or r['kind'] == 'gripper':
            continue
        out.append(f'{name}: lower="{r["lower"]}" upper="{r["upper"]}"')
    out += ['```', '']
    return '\n'.join(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('calibration', help='servo_calibration.json from the workbench')
    ap.add_argument('-o', '--out', default=None, help='write the report here')
    args = ap.parse_args()

    cal = json.loads(Path(args.calibration).read_text())
    report = render(propose(cal))
    if args.out:
        Path(args.out).write_text(report)
        print(f'wrote {args.out}')
    else:
        print(report)


if __name__ == '__main__':
    main()
