"""The model draws every joint the way the metal moves: bench facts, executable.

On 2026-10-12 the sign check passed 12 of 12 on the metal (the hug
convention holds on both arms). The model side of the same question
("does RViz draw it the same way?") needs no hardware: it is forward
kinematics of the URDF. This file runs that FK on the real bench model
and pins what was seen at the bench:

- yaw, shoulder, elbow and wrist pitch: a negative command swings the
  tool FORWARD, away from the column (both arms);
- wrist roll: a negative command turns the gripper INWARD, which seen from
  above is counterclockwise on the right arm and clockwise on the left
  (Andres, 2026-10-12, looking down at the metal);
- fingers: a positive command OPENS the gripper (the tips move apart and
  never cross).

The URDF comes from the xacro (the unit job of CI installs it), or from a
file named by SOMA_URDF, e.g. one produced by the Jetson's container.
"""
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
ARMS = ('right', 'left')
PLANAR = ('yaw', 'shoulder', 'elbow', 'wrist_pitch')
STEP = 0.25


@pytest.fixture(scope='module')
def urdf(tmp_path_factory):
    if os.environ.get('SOMA_URDF'):
        return ET.parse(os.environ['SOMA_URDF']).getroot()
    xacro = pytest.importorskip('xacro')
    src = REPO / 'soma_description' / 'urdf'
    work = tmp_path_factory.mktemp('xacro')
    for f in src.glob('*.xacro'):
        (work / f.name).write_text(
            f.read_text().replace('$(find soma_description)/urdf', str(work)))
    return ET.fromstring(xacro.process_file(str(work / 'soma_bench.urdf.xacro')).toxml())


# ---- a small, generic forward kinematics over the URDF tree ----

def _vec(text, default='0 0 0'):
    return [float(v) for v in (text or default).split()]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _axis_angle(axis, a):
    n = math.sqrt(sum(v * v for v in axis))
    x, y, z = (v / n for v in axis)
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _app(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def frame(root, link, q):
    """World rotation and origin of `link` for joint angles `q` (rad)."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain = []
    while link in joints:
        chain.append(joints[link])
        link = joints[link].find('parent').get('link')
    rot, pos = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    for j in reversed(chain):
        origin = j.find('origin')
        xyz = _vec(origin.get('xyz') if origin is not None else None)
        rpy = _vec(origin.get('rpy') if origin is not None else None)
        pos = [p + d for p, d in zip(pos, _app(rot, xyz))]
        rot = _mul(rot, _rpy(*rpy))
        if j.get('type') in ('revolute', 'continuous'):
            mimic = j.find('mimic')
            angle = (float(mimic.get('multiplier', 1)) * q.get(mimic.get('joint'), 0.0)
                     if mimic is not None else q.get(j.get('name'), 0.0))
            rot = _mul(rot, _axis_angle(_vec(j.find('axis').get('xyz'), '1 0 0'), angle))
    return rot, pos


def point(root, link, local, q):
    rot, pos = frame(root, link, q)
    return [p + d for p, d in zip(pos, _app(rot, local))]


# ---- the bench facts ----

@pytest.mark.parametrize('arm', ARMS)
@pytest.mark.parametrize('joint', PLANAR)
def test_a_negative_planar_command_swings_the_tool_forward(urdf, arm, joint):
    home = point(urdf, f'{arm}_arm_tool0', [0, 0, 0], {})
    moved = point(urdf, f'{arm}_arm_tool0', [0, 0, 0], {f'{arm}_arm_{joint}_joint': -STEP})
    assert moved[0] > home[0] + 0.01, f'{arm} {joint}: x {home[0]:.3f} -> {moved[0]:.3f}'


@pytest.mark.parametrize('arm, sense', [('right', 'CCW'), ('left', 'CW')])
def test_a_negative_roll_turns_the_gripper_inward(urdf, arm, sense):
    # Seen from above (world +Z toward the eye). The finger_l gear axis sits
    # off the roll axis, so its path around it shows the sense of rotation.
    center = point(urdf, f'{arm}_arm_gripper_base_link', [0, 0, 0], {})
    before = point(urdf, f'{arm}_arm_finger_l_link', [0, 0, 0], {})
    after = point(urdf, f'{arm}_arm_finger_l_link', [0, 0, 0],
                  {f'{arm}_arm_wrist_roll_joint': -STEP})
    a = (before[0] - center[0], before[1] - center[1])
    b = (after[0] - center[0], after[1] - center[1])
    turn = a[0] * b[1] - a[1] * b[0]
    assert ('CCW' if turn > 0 else 'CW') == sense


@pytest.mark.parametrize('arm', ARMS)
def test_a_positive_finger_command_opens_and_never_crosses(urdf, arm):
    # Separation of the two fingertips along the gripper's own z axis, signed:
    # opening makes it grow; a closing drawing would shrink it and cross zero.
    rot, _ = frame(urdf, f'{arm}_arm_gripper_base_link', {})
    z = [rot[0][2], rot[1][2], rot[2][2]]

    def separation(q):
        cmd = {f'{arm}_arm_finger_l_joint': q}
        tip_l = point(urdf, f'{arm}_arm_finger_l_link', [0.06, 0, 0], cmd)
        tip_r = point(urdf, f'{arm}_arm_finger_r_link', [0.06, 0, 0], cmd)
        return sum((l - r) * zz for l, r, zz in zip(tip_l, tip_r, z))

    closed, quarter, full = separation(0.0), separation(0.25), separation(1.0)
    assert 0 < closed < quarter < full, (closed, quarter, full)
