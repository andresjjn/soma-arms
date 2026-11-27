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


def test_parse_points_rejects_malformed_input():
    with pytest.raises(ValueError, match='x,z in mm'):
        parse_points(['10'])


def test_plan_trials_rejects_unreachable_points_before_motion(dims):
    trials = plan_trials('left', list(DEFAULT_POINTS_MM), dims=dims)
    assert len(trials) == 3

    with pytest.raises(ValueError, match='unreachable'):
        plan_trials('right', [(500.0, 30.0)], dims=dims)


def test_run_trials_records_errors_and_returns_home(dims):
    trials = plan_trials('left', [(-30.0, 30.0), (0.0, 30.0)], dims=dims)
    published = []
    answers = iter(['', '-28.0 31.5', '', '0.5 29.0'])
    logs = []

    results = run_trials(
        trials, 'left', published.append, lambda _: None,
        lambda _: next(answers), logs.append, dwell=0.1)

    assert len(results) == 2
    assert results[0].dx_mm == pytest.approx(2.0)
    assert results[0].dz_mm == pytest.approx(1.5)
    assert results[0].error_mm == pytest.approx(2.5)
    assert published[-1] == arm_home('left')


def test_run_trials_unwinds_home_on_abort_or_keyboard_interrupt(dims):
    trials = plan_trials('left', [(-30.0, 30.0), (0.0, 30.0)], dims=dims)
    published = []
    answers = iter(['', 'q'])
    results = run_trials(
        trials, 'left', published.append, lambda _: None,
        lambda _: next(answers), lambda _: None, dwell=0.1)
    assert results == []
    assert published[-1] == arm_home('left')

    published2 = []

    def raise_interrupt(_prompt):
        if len(published2) >= 1:
            raise KeyboardInterrupt
        return ''

    with pytest.raises(KeyboardInterrupt):
        run_trials(trials, 'left', published2.append, lambda _: None,
                   raise_interrupt, lambda _: None, dwell=0.1)
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
