"""scripts/pwm_clock_check.py: which channels it may drive, and the arithmetic.

Pure tests: smbus2 and gpiomon are only used inside main().
"""
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import pwm_clock_check as pc  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402


@pytest.mark.parametrize('address', [0x40, 0x43])
def test_it_never_drives_a_channel_a_joint_owns_nor_channel_0(address):
    owned = {s.channel for s in SERVO_MAP.values() if s.address == address}
    free = pc.free_channels(address)
    assert free and not owned & set(free)
    assert 0 not in free


def test_the_free_channels_of_today():
    assert pc.free_channels(0x40) == [1, 2, 4, 5, 6, 7, 8, 9]       # 3 is the L16
    assert pc.free_channels(0x43) == [1, 2, 3, 10, 11, 12, 13, 14, 15]


def test_counts_at_the_nominal_clock():
    # 25 MHz / 4096 / 122 = 50.03 Hz, 19988 us a period
    assert pc.counts_for_us(1500) == 307


def edges(period_us, width_us, n=50, start_ns=1_000_000_000):
    out, t = [], start_ns
    for _ in range(n):
        out += [(1, t), (0, t + round(width_us * 1000))]
        t += round(period_us * 1000)
    return out


def test_a_nominal_board_reads_exact():
    r = pc.analyze(edges(19988.5, 1500.0), 1500.0)
    assert r['frequency_hz'] == pytest.approx(50.03, abs=0.01)
    assert r['width_ratio'] == pytest.approx(1.0)
    assert r['oscillator_error_pct'] == pytest.approx(0.0, abs=0.01)


def test_a_fast_oscillator_shortens_the_pulse_and_raises_the_frequency():
    k = 1.2
    r = pc.analyze(edges(19988.5 / k, 1500.0 / k), 1500.0)
    assert r['frequency_hz'] == pytest.approx(50.03 * k, rel=1e-3)
    assert r['width_ratio'] == pytest.approx(1 / k, rel=1e-3)
    assert r['oscillator_mhz'] == pytest.approx(25 * k, rel=1e-3)


def test_a_lost_edge_does_not_move_the_median():
    ev = edges(19988.5, 1500.0)
    del ev[21]                                   # one falling edge lost
    assert pc.analyze(ev, 1500.0)['period_us'] == pytest.approx(19988.5, abs=0.1)


def test_gpiomon_lines_and_too_few_edges():
    assert pc.parse_gpiomon('1 12 500\n0 12 1500500\nnoise\n') == [
        (1, 12_000_000_500), (0, 12_001_500_500)]
    assert pc.analyze(edges(20000, 1500, n=3), 1500) is None
