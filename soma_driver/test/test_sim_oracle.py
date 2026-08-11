"""The L1 pendulum oracle: MuJoCo must reproduce pencil-and-paper physics.

Pattern borrowed from kimi-k3-in-c's 13-layer oracle: before trusting the
engine with the real thing, prove it on a miniature with the same
structure. Here the miniature is one hinged box with SOMA's measured
forearm numbers, and the reference is the analytic small-angle pendulum.

If these tests fail, the simulation pipeline (units, inertia
conventions, gravity, integrator) is lying, and every later system-ID or
RL result built on it would inherit the lie. Skipped automatically where
the mujoco wheel is not installed.
"""
import math
import sys
from pathlib import Path

import pytest

# Runtime skip, NOT pytest.importorskip at module level: the ROS
# container ships pytest 6.2.5 with pluggy 0.13, and a Skipped raised
# during collection import aborts collection of every sibling file
# (observed 2026-08-10: 125 tests collapsed to 0). pytestmark skips at
# run time and leaves the rest of the suite alone.
try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None

pytestmark = pytest.mark.skipif(mujoco is None, reason='mujoco not installed')

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

REPO = Path(__file__).resolve().parents[2]
ORACLE = REPO / 'sim' / 'pendulum_oracle.xml'

# The same numbers the MJCF carries, restated independently here so the
# test derives its expectation from physics, not from the simulator.
MASS = 0.16          # kg, forearm estimate pending the real weigh-in
LX, LY, LZ = 0.0319, 0.0528, 0.075   # m, measured 2026-08-05 cross + length
GRAVITY = 9.81
PIVOT_TO_COM = LZ / 2.0

AMPLITUDE_RAD = math.radians(5.0)


def analytic_period_s() -> float:
    """Small-angle period of a box pendulum hinged at its top face.

    T = 2*pi*sqrt(I_pivot / (m*g*d)), with the box inertia about the
    hinge axis (the y axis of the MJCF: swing spans the box length LZ
    and thickness LX) moved to the pivot by the parallel axis theorem.
    """
    i_com = MASS * (LZ ** 2 + LX ** 2) / 12.0
    i_pivot = i_com + MASS * PIVOT_TO_COM ** 2
    return 2.0 * math.pi * math.sqrt(
        i_pivot / (MASS * GRAVITY * PIVOT_TO_COM))


def simulate(seconds: float):
    model = mujoco.MjModel.from_xml_path(str(ORACLE))
    data = mujoco.MjData(model)
    data.qpos[0] = AMPLITUDE_RAD
    mujoco.mj_forward(model, data)
    trace = []
    steps = int(seconds / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)
        trace.append((data.time, data.qpos[0], data.qvel[0]))
    return model, data, trace


class TestPendulumOracle:
    def test_period_matches_the_analytic_pendulum(self):
        _, _, trace = simulate(4.0)
        crossings = [
            t0 + (t1 - t0) * (0.0 - q0) / (q1 - q0)
            for (t0, q0, _), (t1, q1, _) in zip(trace, trace[1:])
            if q0 < 0.0 <= q1
        ]
        assert len(crossings) >= 4, 'pendulum barely swings: something is off'
        periods = [b - a for a, b in zip(crossings, crossings[1:])]
        measured = sum(periods) / len(periods)
        expected = analytic_period_s()
        assert measured == pytest.approx(expected, rel=0.01), (
            f'simulated {measured:.4f}s vs analytic {expected:.4f}s')

    def test_energy_is_conserved(self):
        """Undamped RK4 at 0.5 ms must not leak energy over 4 seconds."""
        model, _, trace = simulate(4.0)
        i_com = MASS * (LZ ** 2 + LX ** 2) / 12.0
        i_pivot = i_com + MASS * PIVOT_TO_COM ** 2

        def energy(q, w):
            height = -PIVOT_TO_COM * math.cos(q)
            return 0.5 * i_pivot * w * w + MASS * GRAVITY * height

        first = energy(trace[0][1], trace[0][2])
        last = energy(trace[-1][1], trace[-1][2])
        reference = MASS * GRAVITY * PIVOT_TO_COM  # energy scale of the system
        assert abs(last - first) / reference < 0.005

    def test_oracle_numbers_match_the_measured_forearm(self):
        """The MJCF geometry must carry the caliper numbers of 2026-08-05."""
        model = mujoco.MjModel.from_xml_path(str(ORACLE))
        half = model.geom_size[0]
        assert 2.0 * half[0] == pytest.approx(LX)
        assert 2.0 * half[1] == pytest.approx(LY)
        assert 2.0 * half[2] == pytest.approx(LZ)
        assert model.body_mass[1] == pytest.approx(MASS)


class TestBenchUrdfLoadsInMujoco:
    """The full model must at least parse: geometry travels, layers pend."""

    @pytest.fixture(scope='class')
    def bench_model(self, tmp_path_factory):
        xacro = pytest.importorskip('xacro')
        src = REPO / 'soma_description' / 'urdf'
        work = tmp_path_factory.mktemp('xacro')
        for f in src.glob('*.xacro'):
            text = f.read_text().replace(
                '$(find soma_description)/urdf', str(work))
            (work / f.name).write_text(text)
        urdf = xacro.process_file(str(work / 'soma_bench.urdf.xacro')).toxml()
        path = work / 'soma_bench.urdf'
        path.write_text(urdf)
        return mujoco.MjModel.from_xml_path(str(path))

    def test_joint_count_survives_the_translation(self, bench_model):
        # 12 commanded arm joints + 2 finger_r (mimic is IGNORED by the
        # URDF parser: the gear becomes an MJCF equality later, by hand).
        assert bench_model.njnt == 14

    def test_no_actuators_arrive_for_free(self, bench_model):
        # Position servos are hand-authored in the L1 pass, per the plan.
        assert bench_model.nu == 0
