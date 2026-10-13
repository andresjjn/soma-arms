"""The camera coverage sweep: one straight arm at a time, base only, 40 deg at most."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.primitives import HOME, POSES, SEQUENCES, pose_targets  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

CAM_POSES = sorted(p for p in POSES if p.startswith('cam_'))


def test_six_poses_three_per_arm():
    assert CAM_POSES == sorted(f'cam_{s}_{d}' for s in ('left', 'right') for d in (15, 30, 40))


@pytest.mark.parametrize('pose', CAM_POSES)
def test_only_the_base_bends_and_only_on_one_arm(pose):
    side = pose.split('_')[1]
    targets = POSES[pose]
    assert set(targets) == {f'{side}_arm_{j}_joint' for j in
                            ('yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'finger_l')}
    bent = {n for n, v in targets.items() if v != 0.0}
    assert bent == {f'{side}_arm_yaw_joint'}


@pytest.mark.parametrize('pose', CAM_POSES)
def test_forward_and_never_past_40_degrees(pose):
    yaw = POSES[pose][f"{pose.split('_')[1]}_arm_yaw_joint"]
    assert -0.70 <= yaw < 0.0                       # negative = forward, verified
    spec = SERVO_MAP[f"{pose.split('_')[1]}_arm_yaw_joint"]
    assert spec.lower + 0.3 <= yaw


def test_one_arm_moves_while_the_other_hangs():
    state, moved_together = dict(HOME), []
    for pose, _ in SEQUENCES['camera_check']:
        state.update(pose_targets(pose))
        raised = {s for s in ('left', 'right') if state[f'{s}_arm_yaw_joint'] != 0.0}
        moved_together.append(len(raised) <= 1)
    assert all(moved_together)


def test_each_arm_comes_down_the_way_it_went_up_and_each_raised_pose_holds_2_s():
    steps = SEQUENCES['camera_check']
    names = [p for p, _ in steps]
    for side in ('right', 'left'):
        seen = [p for p in names if p.startswith(f'cam_{side}')]
        assert seen == [f'cam_{side}_{d}' for d in (15, 30, 40, 30, 15)]
    assert all(dwell >= 2.0 for pose, dwell in steps if pose.startswith('cam_'))
