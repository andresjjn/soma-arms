"""Tests for soma_driver.er2_supervisor (v0.5 ER 2 supervisor, pure/offline)."""
import csv
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'soma_driver'))

import soma_driver.er2_supervisor as sup  # noqa: E402
from soma_driver.homography import fit_homography  # noqa: E402
from soma_driver.kinematics import load_dimensions  # noqa: E402


def _synthetic_camera_project(deck_pts_mm):
    H = np.array([
        [0.05, -1.65, 320.0],
        [-0.38, 0.02, 260.0],
        [-0.0004, 0.0, 1.0],
    ], dtype=np.float64)
    out = []
    for x, y in deck_pts_mm:
        v = H @ np.array([x, y, 1.0])
        out.append((float(v[0] / v[2]), float(v[1] / v[2])))
    return out


@pytest.fixture(scope='module')
def dims():
    return load_dimensions()


@pytest.fixture(scope='module')
def calibration():
    deck_pts = [(-70.0, 81.0), (70.0, 81.0), (70.0, -81.0), (-70.0, -81.0)]
    img_pts = _synthetic_camera_project(deck_pts)
    return fit_homography(img_pts, deck_pts, image_size=(640, 360), deck_z_mm=40.0)


class FakeEr2Backend:
    def __init__(self, responses):
        self._responses = list(responses)
        self.calls = []

    def generate(self, image_bytes, mime_type, prompt, thinking='low'):
        self.calls.append((mime_type, prompt, thinking))
        item = self._responses.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


def test_importing_supervisor_never_imports_google_sdk_or_ros():
    assert 'google.genai' not in sys.modules
    assert 'rclpy' not in sys.modules


def test_tool_schema_and_source_never_touch_arming_or_limits():
    source = Path(sup.__file__).read_text()
    for forbidden in ('/soma/arm', 'allow_real', 'RealPca9685', 'disable_all', 'rclpy'):
        assert forbidden not in source
    assert {t['name'] for t in sup.TOOLS} == {'pick_and_place', 'run_tending_cycle', 'go_home'}


def test_execute_tool_call_whitelist_and_safety_enforcement(dims):
    res, steps = sup.execute_tool_call(
        'pick_and_place', {'side': 'left', 'src_x_mm': -70.0, 'dst_x_mm': 0.0}, dims=dims)
    assert res == {'ok': True, 'steps': 8}
    assert len(steps) == 8

    res_cycle, cycle_steps = sup.execute_tool_call(
        'run_tending_cycle', {'side': 'right'}, dims=dims)
    assert res_cycle['ok'] is True and len(cycle_steps) == 18

    res_home, home_steps = sup.execute_tool_call('go_home', {'side': 'left'}, dims=dims)
    assert res_home == {'ok': True, 'steps': 1} and home_steps[0].label == 'home'

    # Unknown tools, extra keys, non-finite values, and unreachable targets are refused.
    for name, args in (
        ('arm_robot', {}),
        ('pick_and_place', {'side': 'left', 'src_x_mm': 0, 'dst_x_mm': 10, 'force': True}),
        ('pick_and_place', {'side': 'center', 'src_x_mm': 0, 'dst_x_mm': 10}),
        ('pick_and_place', {'side': 'left', 'src_x_mm': float('nan'), 'dst_x_mm': 10}),
        ('pick_and_place', {'side': 'right', 'src_x_mm': 0.0, 'dst_x_mm': 70.0}),
        ('go_home', {'side': 'both'}),
    ):
        r, s = sup.execute_tool_call(name, args, dims=dims)
        assert r['ok'] is False and 'error' in r and s == []


def test_parsers_handle_fences_and_reject_malformed_json():
    pts = sup.parse_points('```json\n[{"point": [500, 250], "label": "block"}]\n```')
    assert pts == [{'point': [500.0, 250.0], 'label': 'block'}]

    v = sup.parse_verification('{"success": true, "part_id": "B-04", "reason": "seated"}')
    assert v == sup.VerificationResult(True, 'B-04', 'seated')

    for bad in ('not json', '{}', '[{"point": [1001, 0], "label": "x"}]'):
        with pytest.raises(ValueError):
            sup.parse_points(bad)
    for bad_v in ('[]', '{"success": "yes"}', '{"success": true, "part_id": 12}'):
        with pytest.raises(ValueError):
            sup.parse_verification(bad_v)


def test_verified_cycle_increments_counter_and_logs_cost_and_part_id(
    calibration, dims, tmp_path
):
    backend = FakeEr2Backend([
        sup.Er2Response(
            text='{"success": true, "part_id": "BLK-01", "reason": "cleanly seated"}',
            latency_s=1.8,
            prompt_tokens=1000,
            answer_tokens=40,
            thinking_tokens=60,
            total_tokens=1100,
        )
    ])
    session = sup.SupervisorSession(backend, calibration, dims=dims)
    executed = []
    outcome = session.run_cycle('left', executed.append, lambda: b'fake-jpg')

    assert outcome.status == 'verified'
    assert outcome.part_id == 'BLK-01'
    assert session.counters.verified == 1
    assert session.counters.unverified == 0
    assert session.counters.failed == 0
    assert session.seen_part_ids == ['BLK-01']
    assert len(executed) == 1

    csv_path = tmp_path / 'er2_usage.csv'
    session.write_usage_csv(csv_path)
    rows = list(csv.DictReader(csv_path.open()))
    assert len(rows) == 1
    assert rows[0]['operation'] == 'verify'
    # 1000 * $1/M + (40+60) * $5/M = $0.001500
    assert float(rows[0]['estimated_cost_usd']) == pytest.approx(0.0015, abs=1e-7)


def test_dropped_part_inside_lane_is_relocalized_and_recovered(calibration, dims):
    # Dropped part sits at (-25, 81) mm on the left lane.
    u, v = _synthetic_camera_project([(-25.0, 81.0)])[0]
    er2_yx = [v / 360.0 * 1000.0, u / 640.0 * 1000.0]

    backend = FakeEr2Backend([
        sup.Er2Response('{"success": false, "part_id": null, "reason": "output empty"}', 1.5),
        sup.Er2Response(f'[{{"point": [{er2_yx[0]:.2f}, {er2_yx[1]:.2f}], "label": "wooden block"}}]', 1.4),
        sup.Er2Response('{"success": true, "part_id": "BLK-03", "reason": "recovered"}', 1.6),
    ])
    session = sup.SupervisorSession(backend, calibration, dims=dims)
    executed = []
    outcome = session.run_cycle('left', executed.append, lambda: b'fake-jpg')

    assert outcome.status == 'recovered'
    assert outcome.part_id == 'BLK-03'
    assert session.counters.verified == 1
    assert session.counters.recovered == 1
    assert session.counters.failed == 0
    assert len(executed) == 2  # initial cycle + recovery pick_and_place


def test_dropped_part_outside_lane_fails_cleanly_without_crashing(calibration, dims):
    # Part landed in the central dead zone (0, 0) mm -> cannot be reached laterally.
    u, v = _synthetic_camera_project([(0.0, 0.0)])[0]
    er2_yx = [v / 360.0 * 1000.0, u / 640.0 * 1000.0]

    backend = FakeEr2Backend([
        sup.Er2Response('{"success": false, "part_id": null, "reason": "dropped"}', 1.2),
        sup.Er2Response(f'[{{"point": [{er2_yx[0]:.2f}, {er2_yx[1]:.2f}], "label": "wooden block"}}]', 1.3),
    ])
    session = sup.SupervisorSession(backend, calibration, dims=dims)
    executed = []
    outcome = session.run_cycle('left', executed.append, lambda: b'fake-jpg')

    assert outcome.status == 'failed'
    assert 'outside reachable lane' in outcome.reason
    assert session.counters.failed == 1
    assert len(executed) == 1  # never attempts an unreachable move


def test_api_outage_falls_back_to_geometry_and_counts_unverified(calibration, dims):
    backend = FakeEr2Backend([sup.Er2ApiError('429 rate limited')])
    session = sup.SupervisorSession(backend, calibration, dims=dims)
    executed = []
    outcome = session.run_cycle('right', executed.append, lambda: b'fake-jpg')

    assert outcome.status == 'unverified_fallback'
    assert session.counters.unverified == 1
    assert session.counters.verified == 0
    assert session.counters.total_completed == 1
    assert len(executed) == 1
