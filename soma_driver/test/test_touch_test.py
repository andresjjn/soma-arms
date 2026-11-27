"""Tests for soma_driver.touch_test (pure, no ROS, no hardware)."""
import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.kinematics import load_dimensions  # noqa: E402
from soma_driver.touch_test import (  # noqa: E402
    DEFAULT_POINTS_MM,
    arm_home,
    parse_cli,
    parse_points,
    plan_trials,
    run_trials,
    summarize,
    write_csv,
)


@pytest.fixture(scope='module')
def dims():
    return load_dimensions()


def test_parse_cli_defaults_and_custom_points():
    opts = parse_cli([])
    assert opts.arm == 'left'
    assert opts.parsed_points == list(DEFAULT_POINTS_MM)

    custom = parse_cli(['--arm', 'right', '--points', '-20,30', '10,40', '--tilt', '5'])
    assert custom.arm == 'right'
    assert custom.parsed_points == [(-20.0, 30.0), (10.0, 40.0)]
    assert custom.tilt == 5.0


def test_parse_cli_and_points_reject_unsafe_or_malformed_inputs():
    with pytest.raises(ValueError, match='x,z in mm'):
        parse_points(['10'])
    with pytest.raises(ValueError, match='finite'):
        parse_points(['nan,30'])
    with pytest.raises(SystemExit):
        parse_cli(['--dwell', '0'])
    with pytest.raises(SystemExit):
        parse_cli(['--dwell', '-1.5'])
    with pytest.raises(SystemExit):
        parse_cli(['--tilt', 'nan'])


def test_plan_trials_rejects_unreachable_and_unsafe_points_before_motion(dims):
    trials = plan_trials('left', list(DEFAULT_POINTS_MM), dims=dims)
    assert len(trials) == 3

    with pytest.raises(ValueError, match='required'):
        plan_trials('left', [], dims=dims)
    with pytest.raises(ValueError, match='safety floor'):
        plan_trials('left', [(0.0, 5.0)], dims=dims)
    with pytest.raises(ValueError, match='non-finite'):
        plan_trials('left', [(float('nan'), 30.0)], dims=dims)
    with pytest.raises(ValueError, match='unreachable or unsafe'):
        plan_trials('right', [(500.0, 30.0)], dims=dims)


def test_run_trials_records_errors_rejects_nan_and_returns_home(dims):
    trials = plan_trials('left', [(-30.0, 30.0), (0.0, 30.0)], dims=dims)
    published = []
    # Includes an invalid 'nan 30' input that must re-prompt safely.
    answers = iter(['', 'nan 30', '-28.0 31.5', '', '0.5 29.0'])
    logs = []

    results = run_trials(
        trials, 'left', published.append, lambda _: None,
        lambda _: next(answers), logs.append, dwell=0.1)

    assert len(results) == 2
    assert results[0].dx_mm == pytest.approx(2.0)
    assert results[0].dz_mm == pytest.approx(1.5)
    assert results[0].error_mm == pytest.approx(2.5)
    assert published[-1] == arm_home('left')


def test_run_trials_unwinds_home_on_abort_and_interrupt_during_publish(dims):
    trials = plan_trials('left', [(-30.0, 30.0), (0.0, 30.0)], dims=dims)
    published = []
    answers = iter(['', 'q'])
    results = run_trials(
        trials, 'left', published.append, lambda _: None,
        lambda _: next(answers), lambda _: None, dwell=0.1)
    assert results == []
    assert published[-1] == arm_home('left')

    # Interrupt inside publish() on the very first move must still unwind home.
    published2 = []

    def flaky_publish(pose):
        published2.append(pose)
        if len(published2) == 1:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        run_trials(trials, 'left', flaky_publish, lambda _: None,
                   lambda _: '', lambda _: None, dwell=0.1)
    assert len(published2) == 2
    assert published2[-1] == arm_home('left')


def test_write_csv_and_summarize(tmp_path, dims):
    trials = plan_trials('left', [(-30.0, 30.0), (-30.0, 30.0)], dims=dims)
    answers = iter(['', '-29.0 30.0', '', '-27.0 30.0'])
    results = run_trials(
        trials, 'left', lambda _: None, lambda _: None,
        lambda _: next(answers), lambda _: None, dwell=0.1)

    out_csv = tmp_path / 'touch.csv'
    write_csv(out_csv, results)
    rows = list(csv.DictReader(out_csv.open()))
    assert len(rows) == 2
    assert rows[0]['requested_x_mm'] == '-30.00'
    assert rows[0]['error_mm'] == '1.00'
    assert rows[1]['error_mm'] == '3.00'

    text = summarize(results)
    assert 'mean error 2.00 mm' in text
    assert 'max 3.00 mm' in text
    assert 'repeatability max spread: (-30, 30) mm: 2.00 mm (2 trials)' in text
