"""The joint probe: a staircase forward, a reading per stair, home the same way."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver import joint_probe as jp  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402


def drive(walk, answers, fail_at=None):
    """Run a walk against fakes; return (walk, published, raised)."""
    published, script, failed = [], iter(answers), []

    def publish(name, value):
        if fail_at is not None and len(published) == fail_at and not failed:
            failed.append(True)               # one glitch, then the bus is back
            raise RuntimeError('bus glitch')
        published.append((name, value))

    def ask(_prompt):
        item = next(script)
        if isinstance(item, BaseException):
            raise item
        return item

    try:
        jp.run(walk, publish, lambda s: None, ask, log=lambda m: None)
        return walk, published, None
    except Exception as exc:          # noqa: BLE001
        return walk, published, exc


def nominal(targets):
    return [f'{-math.degrees(t):.1f}' for t in targets]


@pytest.mark.parametrize('arm', ['right', 'left'])
@pytest.mark.parametrize('joint', jp.PLANAR)
def test_the_staircase_starts_at_zero_goes_forward_and_respects_the_stop(arm, joint):
    targets = jp.plan(arm, joint)
    spec = SERVO_MAP[f'{arm}_arm_{joint}_joint']
    assert targets[0] == 0.0
    assert all(t < 0 for t in targets[1:])                       # forward only
    assert all(b - a == pytest.approx(-jp.STEP) for a, b in zip(targets, targets[1:]))
    assert targets[-1] >= spec.lower + jp.MARGIN - 1e-9
    assert -targets[-1] <= jp.CEILING[joint] + 1e-9


def test_most_shortens_the_staircase():
    assert jp.plan('left', 'elbow', 0.2, 0.5) == [0.0, -0.2, -0.4]


@pytest.mark.parametrize('kwargs', [dict(joint='wrist_roll'), dict(joint='finger_l'),
                                    dict(arm='both'), dict(step=0.5), dict(step=0.01),
                                    dict(most=-0.2)])
def test_what_a_level_cannot_read_or_is_unsafe_is_refused(kwargs):
    args = dict(arm='left', joint='elbow', step=0.2, most=None) | kwargs
    with pytest.raises(ValueError):
        jp.plan(**args)


def test_a_full_walk_reads_every_stair_and_comes_home_the_same_way():
    walk = jp.Walk('left', 'elbow', jp.plan('left', 'elbow', 0.2, 0.6))
    walk, published, raised = drive(walk, ['0', '13', '25', '38'])
    assert raised is None and walk.stopped == ''
    assert [v for _, v in published] == [-0.2, -0.4, -0.6, -0.4, -0.2, 0.0]
    assert {n for n, _ in published} == {'left_arm_elbow_joint'}
    assert [m for _, m in walk.points] == [0.0, 13.0, 25.0, 38.0]


def test_enter_takes_the_nominal_reading_and_s_skips():
    walk = jp.Walk('right', 'yaw', [0.0, -0.2, -0.4])
    walk, _, _ = drive(walk, ['', 's', '20'])
    assert walk.points == [(0.0, 0.0), (-0.4, 20.0)]


@pytest.mark.parametrize('stop, why', [('q', 'quit'), ('x', 'close to something at -0.40 rad')])
def test_q_and_x_stop_and_walk_home(stop, why):
    walk = jp.Walk('left', 'elbow', [0.0, -0.2, -0.4, -0.6])
    walk, published, raised = drive(walk, ['0', '12', stop])
    assert raised is None and walk.stopped == why
    assert [v for _, v in published] == [-0.2, -0.4, -0.2, 0.0]


@pytest.mark.parametrize('exc, why', [(KeyboardInterrupt(), 'interrupted'),
                                      (EOFError(), 'end of input')])
def test_ctrl_c_and_end_of_input_walk_home(exc, why):
    walk = jp.Walk('left', 'elbow', [0.0, -0.2, -0.4])
    walk, published, raised = drive(walk, ['0', exc])
    assert raised is None and walk.stopped == why
    assert published[-1] == ('left_arm_elbow_joint', 0.0)


def test_an_error_walks_home_then_is_raised_again():
    walk = jp.Walk('left', 'elbow', [0.0, -0.2, -0.4])
    walk, published, raised = drive(walk, ['0', '12', '20'], fail_at=1)
    assert isinstance(raised, RuntimeError)
    assert published[-1] == ('left_arm_elbow_joint', 0.0)


def test_unclear_answers_are_asked_again():
    walk = jp.Walk('right', 'yaw', [0.0, -0.2])
    walk, _, _ = drive(walk, ['zero', '0', 'nan', '999', '11.5'])
    assert walk.points == [(0.0, 0.0), (-0.2, 11.5)]


def test_the_fit_recovers_scale_and_zero():
    pts = [(-r, 1.3 * math.degrees(r) + 2.0) for r in (0.0, 0.2, 0.4, 0.6)]
    f = jp.fit(pts)
    assert f['scale'] == pytest.approx(1.3) and f['offset_deg'] == pytest.approx(2.0)
    assert f['max_residual_deg'] == pytest.approx(0.0, abs=0.01)
    assert f['us_per_rad_real'] == pytest.approx(2000 / math.pi / 1.3, abs=0.1)


def test_no_fit_with_too_few_or_degenerate_readings():
    assert jp.fit([(0.0, 0.0), (-0.2, 11.0)]) is None
    assert jp.fit([(-0.2, 10.0)] * 3) is None


def test_the_module_imports_no_ros_and_cannot_arm():
    source = Path(jp.__file__).read_text()
    assert 'import rclpy' not in source and 'SetBool' not in source
