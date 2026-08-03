"""The minimum-jerk profile: organic motion that can never break the ramp.

"Soft" is not an aesthetic claim here, it is four measurable properties:
the joint starts gently, stops gently, never exceeds the per joint rate
ceiling, and retargeting mid-motion never causes a velocity jump.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soma_driver.servo_map import SERVO_MAP  # noqa: E402
from soma_driver.trajectory import (  # noqa: E402
    MIN_DURATION_S, PEAK_FACTOR, JointMotion)

DT = 0.02  # the driver tick


def run_to_rest(motion, max_ticks=5000):
    """Step until the motion settles. Returns the list of positions."""
    positions = []
    for _ in range(max_ticks):
        positions.append(motion.step(DT))
        if motion.done:
            return positions
    pytest.fail('motion never settled')


class TestReachesTheTarget:
    def test_settles_exactly_on_target(self):
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.0)
        run_to_rest(m)
        assert m.position == pytest.approx(1.0)
        assert m.velocity == 0.0

    def test_zero_delta_is_a_no_op(self):
        m = JointMotion(0.5, max_rate=2.5)
        m.set_target(0.5)
        assert m.done
        assert m.step(DT) == pytest.approx(0.5)

    def test_works_in_reverse(self):
        m = JointMotion(1.0, max_rate=2.5)
        m.set_target(-1.0)
        run_to_rest(m)
        assert m.position == pytest.approx(-1.0)


class TestTheCeilingHolds:
    """The rate limit is a hard ceiling, exactly as with the linear ramp."""

    def test_no_step_ever_exceeds_the_rate_limit(self):
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        prev = 0.0
        for pos in run_to_rest(m):
            assert abs(pos - prev) <= 2.5 * DT + 1e-12
            prev = pos

    def test_ceiling_holds_even_with_a_hostile_replan_storm(self):
        """Retargeting every tick to alternating extremes cannot produce a
        step faster than the ceiling."""
        m = JointMotion(0.0, max_rate=2.5)
        prev = 0.0
        for i in range(200):
            m.set_target(1.5 if i % 2 == 0 else -1.5)
            pos = m.step(DT)
            assert abs(pos - prev) <= 2.5 * DT + 1e-12
            prev = pos

    def test_peak_velocity_is_the_ceiling_not_below_it(self):
        """The duration formula aims the bell's peak AT max_rate: softness
        must not come at the price of crawling."""
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        positions = run_to_rest(m)
        steps = [abs(b - a) for a, b in zip([0.0] + positions, positions)]
        assert max(steps) == pytest.approx(2.5 * DT, rel=0.03)


class TestSoftness:
    def test_starts_gently(self):
        """First tick of a long move is far below the ceiling: velocity
        ramps up instead of jumping (the linear ramp jumped)."""
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        first = m.step(DT)
        assert abs(first) < 0.2 * 2.5 * DT

    def test_stops_gently(self):
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        positions = run_to_rest(m)
        last_step = abs(positions[-1] - positions[-2])
        assert last_step < 0.2 * 2.5 * DT

    def test_velocity_is_a_bell_one_rise_one_fall(self):
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        positions = run_to_rest(m)
        steps = [b - a for a, b in zip([0.0] + positions, positions)]
        peak = steps.index(max(steps))
        rising = steps[1:peak]
        falling = steps[peak:-1]
        assert all(b >= a - 1e-9 for a, b in zip(rising, rising[1:]))
        assert all(b <= a + 1e-9 for a, b in zip(falling, falling[1:]))

    def test_tiny_moves_get_the_minimum_duration(self):
        """A 0.001 rad correction takes MIN_DURATION_S, not one violent
        tick."""
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(0.001)
        ticks = len(run_to_rest(m))
        assert ticks >= int(MIN_DURATION_S / DT)


class TestRetargeting:
    def test_replan_midflight_has_no_velocity_jump(self):
        """A new command while moving decelerates and reverses smoothly:
        the velocity right after the replan matches the velocity right
        before it."""
        m = JointMotion(0.0, max_rate=2.5)
        m.set_target(1.5)
        for _ in range(20):
            m.step(DT)
        v_before = m.velocity
        m.set_target(-1.5)  # full reversal, worst case
        m.step(DT)
        assert abs(m.velocity - v_before) < 2.5 * 0.15
        run_to_rest(m)
        assert m.position == pytest.approx(-1.5)


class TestAgainstTheRealMap:
    def test_l16_full_stroke_stays_within_its_physical_rate(self):
        """The lift's bell peaks at 0.020 m/s, the actuator's real speed,
        so the rod can actually track the command."""
        spec = SERVO_MAP['torso_lift_joint']
        m = JointMotion(spec.lower, max_rate=spec.max_rate)
        m.set_target(spec.upper)
        prev = spec.lower
        n = 0
        for pos in run_to_rest(m):
            assert abs(pos - prev) <= spec.max_rate * DT + 1e-12
            prev = pos
            n += 1
        # A bell profile at the same peak takes ~PEAK_FACTOR times the
        # constant-rate time: about 12 s for the 130 mm band, not 6.5 s.
        expected = PEAK_FACTOR * (spec.upper - spec.lower) / spec.max_rate
        assert n * DT == pytest.approx(expected, rel=0.05)
