#!/usr/bin/env python3
"""Check that the URDF and the driver agree about every joint.

The URDF and SERVO_MAP hold the same physical facts in two places: joint
limits, the lift speed, which joints are mimic. If they drift apart, the
digital twin starts lying about the robot, and with the L16 that is how an
actuator ends up wedged against a stop. So CI compares them.

Usage:
  python3 scripts/check_model_driver_sync.py path/to/soma_bench.urdf
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'soma_driver'))

from soma_driver.servo_map import MIMIC_JOINTS, SERVO_MAP  # noqa: E402

TOL = 1e-6


def main(urdf_path: str) -> int:
    root = ET.parse(urdf_path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    problems = []

    for name, spec in SERVO_MAP.items():
        j = joints.get(name)
        if j is None:
            problems.append(f'{name}: in SERVO_MAP but missing from the URDF')
            continue
        limit = j.find('limit')
        if limit is None:
            problems.append(f'{name}: URDF joint has no <limit>')
            continue
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if abs(lower - spec.lower) > TOL or abs(upper - spec.upper) > TOL:
            problems.append(
                f'{name}: URDF limits [{lower}, {upper}] do not match '
                f'SERVO_MAP [{spec.lower}, {spec.upper}]')

    # The lift is the one joint whose speed is a hard hardware fact.
    lift = joints.get('torso_lift_joint')
    if lift is not None:
        vel = float(lift.find('limit').get('velocity'))
        want = SERVO_MAP['torso_lift_joint'].max_rate
        if abs(vel - want) > TOL:
            problems.append(
                f'torso_lift_joint: URDF velocity {vel} does not match the '
                f'driver rate limit {want}')

    for name, (master, mult) in MIMIC_JOINTS.items():
        j = joints.get(name)
        if j is None:
            problems.append(f'{name}: mimic joint missing from the URDF')
            continue
        mimic = j.find('mimic')
        if mimic is None:
            problems.append(f'{name}: driver treats it as mimic, URDF does not')
            continue
        if mimic.get('joint') != master:
            problems.append(
                f'{name}: URDF mimics {mimic.get("joint")}, driver expects {master}')
        if abs(float(mimic.get('multiplier', 1.0)) - mult) > TOL:
            problems.append(f'{name}: mimic multiplier disagrees with the driver')
        if name in SERVO_MAP:
            problems.append(f'{name}: mimic joints must not own a PWM channel')

    if problems:
        print('URDF and driver are out of sync:')
        for p in problems:
            print(f'  - {p}')
        return 1

    print(f'URDF and driver agree on {len(SERVO_MAP)} commanded joints '
          f'and {len(MIMIC_JOINTS)} mimic joints')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
