"""The sign check decides which fix a joint gets, so its promises are tested blind.

Pure tests, no ROS: run() receives fake publish, sleep and ask callables
and the assertions read what it did with them. The contract under test:
every excursion stays clear of the measured stops, every way out of the
walk leaves the current joint commanded to 0.0, nothing is guessed from
a typo, and a reversed joint is answered with a driver row, never with
an axis flip.
"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver import sign_check as sc  # noqa: E402
from soma_driver.player import SETTLE_MARGIN_S  # noqa: E402
from soma_driver.primitives import COMMANDED_JOINTS, settle_time_s  # noqa: E402
from soma_driver.servo_map import SERVO_MAP, ServoSpec  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1] / 'soma_driver'
ALL = sc.joints_for('both')
RIGHT = sc.joints_for('right')


class Bench:
    """Fake publish, sleep and ask; scripted answers may be exceptions."""

    def __init__(self, answers=(), fail_sleep_at=None, fail_with=KeyboardInterrupt):
        self.published: list[tuple[str, float]] = []
        self.slept: list[float] = []
        self.prompts: list[str] = []
        self.answers = list(answers)
        self.fail_sleep_at = fail_sleep_at
        self.fail_with = fail_with

    def publish(self, name, position):
        self.published.append((name, position))

    def sleep(self, seconds):
        self.slept.append(seconds)
        if len(self.slept) - 1 == self.fail_sleep_at:
            raise self.fail_with()

    def ask(self, prompt):
        self.prompts.append(prompt)
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    def run(self, arm='right', **kwargs):
        return sc.run(arm, self.publish, self.sleep, self.ask,
                      log=lambda _: None, **kwargs)


def last_command(bench, name):
    return [p for n, p in bench.published if n == name][-1]


# ---- what gets moved, in which order, and how far ----

def test_order_per_arm_is_most_trusted_first():
    assert RIGHT == ('right_arm_yaw_joint', 'right_arm_finger_l_joint',
                     'right_arm_wrist_roll_joint', 'right_arm_wrist_pitch_joint',
                     'right_arm_elbow_joint', 'right_arm_shoulder_joint')
    assert sc.joints_for('left') == tuple(n.replace('right_', 'left_', 1) for n in RIGHT)
    assert ALL == RIGHT + sc.joints_for('left')
    with pytest.raises(ValueError):
        sc.joints_for('torso')


def test_both_arms_cover_every_commanded_joint_once():
    assert len(ALL) == 12
    assert set(ALL) == set(COMMANDED_JOINTS)


@pytest.mark.parametrize('name', ALL)
def test_no_excursion_within_a_tenth_of_a_radian_of_a_stop(name):
    spec = SERVO_MAP[name]
    target = sc.excursion(name)
    assert target != 0.0
    assert spec.lower + 0.1 - 1e-12 <= target <= spec.upper - 0.1 + 1e-12, name


def test_arm_joints_go_to_the_hug_side_and_fingers_open():
    # The right yaw zero is 0.2749 rad from its backward stop: the old
    # tool sent it to +0.25, 1.4 deg from that stop.
    assert sc.excursion('right_arm_yaw_joint') == -0.25
    for name in ALL:
        expected = 0.25 if 'finger' in name else -0.25
        assert sc.excursion(name) == expected, name


def test_excursion_falls_back_to_the_roomier_side_short_of_its_stop():
    fake = {
        'neg': ServoSpec(1, 500.0, 2500.0, -0.3, 0.2, 2.5),
        'pos': ServoSpec(2, 500.0, 2500.0, -0.1, 0.3, 2.5),
        'none': ServoSpec(3, 500.0, 2500.0, -0.05, 0.08, 2.5),
    }
    assert sc.excursion('neg', servo_map=fake) == pytest.approx(-0.2)
    assert sc.excursion('pos', servo_map=fake) == pytest.approx(0.2)
    with pytest.raises(ValueError, match='no side'):
        sc.excursion('none', servo_map=fake)


@pytest.mark.parametrize('delta', [0.0, -0.25, 0.51, 15.0])
def test_delta_is_refused_outside_its_band(delta):
    # 15 is the degrees-for-radians typo: 15 rad would be two full turns.
    with pytest.raises(ValueError):
        sc.excursion('right_arm_elbow_joint', delta=delta)


def test_the_plan_fails_before_anything_moves():
    bench = Bench()
    with pytest.raises(ValueError):
        bench.run(delta=15.0)
    with pytest.raises(ValueError):
        bench.run(dwell=0.0)
    assert bench.published == []


# ---- what the human is told to expect ----

@pytest.mark.parametrize('kind, negative, positive', [
    ('yaw', 'FORWARD', 'BACKWARD'),
    ('shoulder', 'FRONT', 'BACK'),
    ('elbow', 'FORWARD', 'BACKWARD'),
    ('wrist_pitch', 'FORWARD', 'BACKWARD'),
    ('wrist_roll', 'INWARD', 'OUTWARD'),
    ('finger_l', 'CLOSES', 'OPENS'),
])
def test_the_expectation_names_the_commanded_direction(kind, negative, positive):
    for arm in sc.ARMS:
        name = f'{arm}_arm_{kind}_joint'
        assert negative in sc.expectation(name, -0.25)
        assert positive in sc.expectation(name, +0.25)
        assert positive not in sc.expectation(name, -0.25).replace(negative, '')


def test_the_finger_says_how_far_it_opens():
    assert 'OPENS 25% of its travel' in sc.expectation('left_arm_finger_l_joint', 0.25)


def test_only_the_yaw_claims_to_be_verified():
    for name in ALL:
        for target in (sc.excursion(name), -sc.excursion(name)):
            text = sc.expectation(name, target)
            assert ('verified' in text) == ('yaw' in name), (name, text)
    assert 'verified on the metal 2026-10-01' in sc.expectation('right_arm_yaw_joint', -0.25)


def test_the_printable_table_matches_the_plan():
    table = sc.expectation_table('right').splitlines()
    assert len(table) == 2 + 6
    for row, (name, target) in zip(table[2:], sc.plan('right')):
        assert f'`{name}`' in row
        assert f'{target:+.2f}' in row
        assert sc.expectation(name, target) in row


# ---- the driver fix: mirror a row ----

@pytest.mark.parametrize('name', ALL)
def test_mirror_keeps_the_zero_pulse(name):
    spec = SERVO_MAP[name]
    assert sc.mirror(spec).command_to_us(0.0) == pytest.approx(
        spec.command_to_us(0.0), abs=1e-9)


@pytest.mark.parametrize('name', ALL)
def test_mirror_is_an_involution(name):
    spec = SERVO_MAP[name]
    assert sc.mirror(sc.mirror(spec)) == spec
    assert sc.mirror(spec) != spec


@pytest.mark.parametrize('name', ALL)
def test_mirror_is_the_same_motion_under_the_opposite_sign(name):
    spec, mirrored = SERVO_MAP[name], sc.mirror(SERVO_MAP[name])
    for i in range(11):
        q = spec.lower + i * (spec.upper - spec.lower) / 10
        assert mirrored.command_to_us(-q) == pytest.approx(spec.command_to_us(q), abs=1e-9)
    assert (mirrored.channel, mirrored.address, mirrored.max_rate) == (
        spec.channel, spec.address, spec.max_rate)


def test_servo_map_line_reproduces_the_right_elbow_row_exactly():
    line = sc.servo_map_line('right_arm_elbow_joint', SERVO_MAP['right_arm_elbow_joint'])
    assert line == ("    'right_arm_elbow_joint':       "
                    'ServoSpec(12, 2460.0, 535.0, -2.3955, 0.6283, 2.5),')
    source = (PACKAGE / 'servo_map.py').read_text().splitlines()
    assert any(row.startswith(line) for row in source)


@pytest.mark.parametrize('name', ALL)
def test_servo_map_line_reproduces_every_arm_row(name):
    # The row is printed to be pasted: it must read exactly like the file.
    line = sc.servo_map_line(name, SERVO_MAP[name])
    source = (PACKAGE / 'servo_map.py').read_text().splitlines()
    assert any(row.startswith(line) for row in source), line


# ---- run(): what gets published, and every way out ----

def test_each_joint_goes_to_its_target_then_home_and_waits_for_the_ramp():
    bench = Bench(answers=['', ''] * 6)
    findings = bench.run(dwell=1.5)
    expected = []
    for name, target in sc.plan('right'):
        expected += [(name, target), (name, 0.0)]
    assert bench.published == expected
    assert [f.verdict for f in findings] == ['ok'] * 6
    for i, (name, target) in enumerate(sc.plan('right')):
        hold, back = bench.slept[2 * i], bench.slept[2 * i + 1]
        assert hold >= settle_time_s({name: target}, {name: 0.0}) + 1.5 - 1e-9
        assert back >= settle_time_s({name: 0.0}, {name: target}) + SETTLE_MARGIN_S - 1e-9


def test_both_walks_the_right_arm_then_the_left():
    bench = Bench(answers=['', ''] * 12)
    findings = bench.run('both')
    assert [f.joint for f in findings] == list(ALL)
    assert [n for n, p in bench.published if p != 0.0] == list(ALL)


def test_s_skips_without_publishing():
    bench = Bench(answers=['s'] + ['', ''] * 5)
    findings = bench.run()
    assert 'right_arm_yaw_joint' not in {n for n, _ in bench.published}
    assert [f.joint for f in findings] == list(RIGHT[1:])


def test_q_leaves_with_the_current_joint_at_zero():
    bench = Bench(answers=['', '', 'q'])          # yaw ok, q at the finger
    findings = bench.run()
    assert findings == [sc.Finding('right_arm_yaw_joint', -0.25, 'ok')]
    assert bench.published[-1] == ('right_arm_finger_l_joint', 0.0)
    assert all(p == 0.0 for n, p in bench.published if n == 'right_arm_finger_l_joint')


def test_end_of_input_returns_the_joint_to_zero():
    bench = Bench(answers=['', EOFError()])       # EOF at the verdict
    assert bench.run() == []
    assert bench.published[-1] == ('right_arm_yaw_joint', 0.0)


def test_ctrl_c_during_the_hold_returns_the_joint_to_zero():
    bench = Bench(answers=[''], fail_sleep_at=0)  # interrupted at the target
    assert bench.run() == []
    assert bench.published == [('right_arm_yaw_joint', -0.25),
                               ('right_arm_yaw_joint', 0.0)]
    # The settle sleep after the return keeps the process alive while
    # the command goes out.
    assert len(bench.slept) == 2


def test_ctrl_c_at_a_prompt_returns_the_joint_to_zero():
    bench = Bench(answers=['', '', KeyboardInterrupt()])
    findings = bench.run()
    assert [f.verdict for f in findings] == ['ok']
    assert bench.published[-1] == ('right_arm_finger_l_joint', 0.0)


def test_an_unexpected_error_still_returns_the_joint_to_zero_and_surfaces():
    bench = Bench(answers=[''], fail_sleep_at=0, fail_with=RuntimeError)
    with pytest.raises(RuntimeError):
        bench.run()
    assert bench.published[-1] == ('right_arm_yaw_joint', 0.0)


def test_r_records_a_reversed_joint():
    bench = Bench(answers=['', 'r', 'q'])
    findings = bench.run()
    assert findings == [sc.Finding('right_arm_yaw_joint', -0.25, 'reversed')]


def test_m_records_a_model_finding():
    bench = Bench(answers=['', 'm', 'q'])
    assert bench.run()[0].verdict == 'model'


def test_u_repeats_the_same_joint():
    bench = Bench(answers=['', 'u', '', '', 'q'])
    findings = bench.run()
    yaw = [p for n, p in bench.published if n == 'right_arm_yaw_joint']
    assert yaw == [-0.25, 0.0, -0.25, 0.0]
    assert findings == [sc.Finding('right_arm_yaw_joint', -0.25, 'ok')]


def test_a_joint_left_unclear_is_recorded_as_unclear():
    bench = Bench(answers=['', 'u', 's', 'q'])
    assert bench.run() == [sc.Finding('right_arm_yaw_joint', -0.25, 'unclear')]
    bench = Bench(answers=['', 'u', 'q'])
    assert bench.run() == [sc.Finding('right_arm_yaw_joint', -0.25, 'unclear')]
    assert bench.published[-1] == ('right_arm_yaw_joint', 0.0)


def test_a_typo_never_moves_a_joint_or_becomes_a_verdict():
    bench = Bench(answers=['y', 'go', '', 'ok?', 'r', 'q'])
    findings = bench.run()
    assert [p for n, p in bench.published if n == 'right_arm_yaw_joint'] == [-0.25, 0.0]
    assert findings[0].verdict == 'reversed'
    assert bench.prompts[:3] == [sc.MOVE_PROMPT] * 3
    assert bench.prompts[3:5] == [sc.VERDICT_PROMPT] * 2


def test_the_tool_has_no_way_to_arm_the_driver():
    for module in ('sign_check.py', 'sign_check_cli.py'):
        source = (PACKAGE / module).read_text()
        for word in ('SetBool', 'std_srvs', 'create_client'):
            assert word not in source, f'{module} mentions {word}'


# ---- the summary: which fix each verdict asks for ----

def test_a_reversed_right_elbow_is_a_driver_fix_never_an_axis_flip():
    text = sc.summary([sc.Finding('right_arm_elbow_joint', -0.25, 'reversed')], RIGHT)
    assert 'ServoSpec(12, 535.0, 2460.0, -0.6283, 2.3955, 2.5)' in text
    assert 'driver' in text
    assert 'NOT an axis flip' in text
    assert '<axis' not in text
    assert 'elbow_lower:=-0.6283 elbow_upper:=2.3955' in text
    assert 'stays at 935 us' in text
    # The wave of 2026-10-03 bends this elbow to -0.90 .. -2.09 rad, all
    # outside the mirrored band: it must be redesigned, never played.
    for pose in ('wave_raise_1', 'wave_raise_2', 'wave_open', 'wave_close'):
        assert pose in text
    assert 'sequences: demo, wave' in text


def test_a_reversed_left_elbow_points_at_the_bench_xacro():
    text = sc.summary([sc.Finding('left_arm_elbow_joint', -0.25, 'reversed')])
    assert 'ServoSpec(6, 630.0, 2455.0, -0.9032, 1.9635, 2.5, address=0x43)' in text
    assert 'soma_bench.urdf.xacro' in text
    assert 'elbow_lower="-0.9032" elbow_upper="1.9635"' in text


def test_a_reversed_shoulder_breaks_no_pose():
    text = sc.summary([sc.Finding('right_arm_shoulder_joint', -0.25, 'reversed')])
    assert 'Poses outside' not in text


def test_a_reversed_right_yaw_flags_the_wave_probe_too():
    text = sc.summary([sc.Finding('right_arm_yaw_joint', -0.25, 'reversed')])
    assert 'wave_probe' in text


def test_a_reversed_finger_is_never_mirrored():
    text = sc.summary([sc.Finding('right_arm_finger_l_joint', 0.25, 'reversed')])
    assert 'Do NOT mirror a finger' in text
    assert 'ServoSpec(' not in text
    assert '<axis' not in text


def test_a_model_finding_flips_the_axis_and_warns_about_both_arms():
    text = sc.summary([sc.Finding('right_arm_wrist_roll_joint', -0.25, 'model')])
    assert '<axis xyz="${side} 0 0"/>  ->  <axis xyz="${-side} 0 0"/>' in text
    assert 'BOTH arms' in text
    assert 'ServoSpec(' not in text
    finger = sc.summary([sc.Finding('left_arm_finger_l_joint', 0.25, 'model')])
    assert '<axis xyz="0 1 0"/>  ->  <axis xyz="0 -1 0"/>' in finger
    assert 'finger_r' in finger


def test_all_ok_survives_contact_with_power():
    findings = [sc.Finding(n, sc.excursion(n), 'ok') for n in RIGHT]
    text = sc.summary(findings, RIGHT)
    assert '6 of 6 joints judged' in text
    assert 'no reversals; the hug convention survived contact with power' in text
    assert 'NOT judged' not in text


def test_an_empty_walk_claims_nothing():
    text = sc.summary([], RIGHT)
    assert 'survived' not in text
    assert 'no joint was judged' in text


def test_the_summary_lists_what_was_not_judged():
    findings = [sc.Finding('right_arm_yaw_joint', -0.25, 'ok'),
                sc.Finding('right_arm_finger_l_joint', 0.25, 'unclear')]
    text = sc.summary(findings, RIGHT)
    assert '1 of 6 joints judged' in text
    assert 'UNCLEAR right_arm_finger_l_joint' in text
    missing = text.split('NOT judged (skipped, unclear or not reached): ')[1]
    for name in RIGHT[1:]:
        assert name in missing


# ---- the start condition, the report, the command line ----

def test_the_walk_starts_only_from_home():
    home = {name: 0.0 for name in ALL}
    assert sc.not_at_zero(home, ALL) == []
    assert sc.not_at_zero(dict(home, right_arm_elbow_joint=0.0004), RIGHT) == []
    assert sc.not_at_zero(dict(home, right_arm_elbow_joint=-0.5), RIGHT) == [
        'right_arm_elbow_joint']
    missing = {k: v for k, v in home.items() if k != 'left_arm_yaw_joint'}
    assert sc.not_at_zero(missing, ALL) == ['left_arm_yaw_joint']
    assert sc.not_at_zero(missing, RIGHT) == []


def test_the_report_is_json_with_every_finding():
    options = sc.Options('right', 0.25, 2.0, 'r.json')
    findings = [sc.Finding('right_arm_yaw_joint', -0.25, 'ok')]
    record = json.loads(json.dumps(sc.report(options, findings, RIGHT)))
    assert record['arm'] == 'right'
    assert record['planned'] == list(RIGHT)
    assert record['findings'] == [
        {'joint': 'right_arm_yaw_joint', 'target': -0.25, 'verdict': 'ok'}]


@pytest.mark.parametrize('argv, expected', [
    ([], sc.Options('right', 0.25, 2.0, None)),
    (['--arm', 'both'], sc.Options('both', 0.25, 2.0, None)),
    (['--arm', 'left', '--delta', '0.3', '--dwell', '1.5', '--report', 'r.json'],
     sc.Options('left', 0.3, 1.5, 'r.json')),
    (['--arm', 'left', '--ros-args', '-r', '__node:=x'], sc.Options('left', 0.25, 2.0, None)),
    (['--ros-args', '-p', 'a:=1', '--', '--arm', 'both'], sc.Options('both', 0.25, 2.0, None)),
])
def test_parse_cli(argv, expected):
    assert sc.parse_cli(argv) == expected


@pytest.mark.parametrize('argv', [
    ['--arm', 'torso'], ['--delta', '15'], ['--delta', '0'], ['--dwell', '-1'],
    ['--dwell', '60'], ['--arm'], ['wave'],
])
def test_parse_cli_refuses(argv, capsys):
    with pytest.raises(SystemExit) as exc:
        sc.parse_cli(argv)
    assert exc.value.code == 2


def test_radians_are_radians():
    assert math.degrees(sc.DELTA) == pytest.approx(14.3, abs=0.1)
    assert sc.MAX_DELTA < math.pi / 4
