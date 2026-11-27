"""Tests for soma_driver.homography (pure, no ROS, no OpenCV)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'soma_driver'))

from soma_driver.homography import (  # noqa: E402
    HomographyCalibration,
    assign_lane,
    er2_point_to_pixel,
    er2_point_to_pose,
    fit_homography,
    point_to_pose,
)
from soma_driver.kinematics import JOINTS, fk, load_dimensions  # noqa: E402


@pytest.fixture(scope='module')
def dims():
    return load_dimensions()


def _synthetic_camera_project(deck_pts_mm):
    """Project deck (x, y) mm through a realistic oblique camera homography."""
    H_deck_to_px = np.array([
        [0.05, -1.65, 320.0],
        [-0.38, 0.02, 260.0],
        [-0.0004, 0.0, 1.0],
    ], dtype=np.float64)
    out = []
    for x, y in deck_pts_mm:
        v = H_deck_to_px @ np.array([x, y, 1.0])
        out.append((float(v[0] / v[2]), float(v[1] / v[2])))
    return out


def test_fit_homography_recovers_perspective_and_roundtrips_json(tmp_path):
    deck_pts = [
        (-70.0, 81.0),
        (70.0, 81.0),
        (70.0, -81.0),
        (-70.0, -81.0),
    ]
    img_pts = _synthetic_camera_project(deck_pts)
    cal = fit_homography(
        img_pts, deck_pts, image_size=(640, 360), deck_z_mm=30.0,
        date='2026-11-27', source='synthetic oblique test')

    assert cal.rms_error_mm < 1e-6
    assert cal.max_error_mm < 1e-6

    # Check interior point (0, 81) mm.
    u, v = _synthetic_camera_project([(0.0, 81.0)])[0]
    x_mm, y_mm = cal.pixel_to_deck(u, v)
    assert x_mm == pytest.approx(0.0, abs=1e-5)
    assert y_mm == pytest.approx(81.0, abs=1e-5)

    out_path = tmp_path / 'homography.json'
    cal.save(out_path)
    loaded = HomographyCalibration.load(out_path)
    assert loaded == cal


def test_fit_homography_on_real_october_bench_marks():
    marks_file = REPO / 'calibration' / 'camera_marks_2026-10-26.json'
    data = json.loads(marks_file.read_text())
    by_name = {m['name']: m['px'] for m in data['marks']}
    # Four taped marks: near at x=0, far at x=250 mm, along right (-81) and left (+81).
    img_pts = [
        tuple(by_name['near left']),
        tuple(by_name['far left']),
        tuple(by_name['far right']),
        tuple(by_name['near right']),
    ]
    deck_pts = [
        (0.0, 81.0),
        (250.0, 81.0),
        (250.0, -81.0),
        (0.0, -81.0),
    ]
    cal = fit_homography(img_pts, deck_pts, image_size=(360, 640), deck_z_mm=30.0)
    assert cal.max_error_mm < 1e-6
    # Midpoint of left lane pixels should land near x=125 mm, y=+81 mm.
    mid_u = 0.5 * (by_name['near left'][0] + by_name['far left'][0])
    mid_v = 0.5 * (by_name['near left'][1] + by_name['far left'][1])
    x_mm, y_mm = cal.pixel_to_deck(mid_u, mid_v)
    assert 115.0 < x_mm < 150.0
    assert y_mm == pytest.approx(81.0, abs=3.0)


def test_fit_homography_rejects_degenerate_and_non_finite_inputs():
    with pytest.raises(ValueError, match='at least 4'):
        fit_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)], (640, 360))

    with pytest.raises(ValueError, match='finite'):
        fit_homography([(float('nan'), 0), (1, 0), (1, 1), (0, 1)],
                       [(0, 0), (1, 0), (1, 1), (0, 1)], (640, 360))

    # Collinear points cannot define a 2D homography.
    with pytest.raises(ValueError, match='collinear'):
        fit_homography([(0, 0), (10, 0), (20, 0), (0, 10)],
                       [(0, 0), (10, 0), (20, 0), (0, 10)], (640, 360))


def test_horizon_singularity_and_er2_bounds_are_refused():
    deck_pts = [(-70.0, 81.0), (70.0, 81.0), (70.0, -81.0), (-70.0, -81.0)]
    img_pts = _synthetic_camera_project(deck_pts)
    cal = fit_homography(img_pts, deck_pts, image_size=(640, 360), deck_z_mm=30.0)

    # Very far negative v crosses the projective horizon in this synthetic camera.
    with pytest.raises(ValueError, match='horizon|finite'):
        cal.pixel_to_deck(320.0, 10000.0)

    with pytest.raises(ValueError, match='0..1000'):
        er2_point_to_pixel([-1.0, 500.0], 640, 360)
    with pytest.raises(ValueError, match='0..1000'):
        er2_point_to_pixel([500.0, 1001.0], 640, 360)


def test_assign_lane_refuses_dead_zone_and_outboard_clicks(dims):
    side_l, off_l = assign_lane(85.0, dims=dims)
    assert side_l == 'left' and off_l == pytest.approx(4.0)

    side_r, off_r = assign_lane(-78.0, dims=dims)
    assert side_r == 'right' and off_r == pytest.approx(3.0)

    # Centerline (y=0) is 81 mm from both planar lanes -> physically unreachable.
    with pytest.raises(ValueError, match='lateral limit'):
        assign_lane(0.0, dims=dims)

    # Too far outboard (> 25 mm past lane) is also refused.
    with pytest.raises(ValueError, match='lateral limit'):
        assign_lane(120.0, dims=dims)


def test_pixel_and_er2_point_to_pose_round_trip_with_fk(dims):
    deck_pts = [(-70.0, 81.0), (70.0, 81.0), (70.0, -81.0), (-70.0, -81.0)]
    img_pts = _synthetic_camera_project(deck_pts)
    cal = fit_homography(img_pts, deck_pts, image_size=(640, 360), deck_z_mm=30.0)

    target_deck = (-30.0, 81.0)
    u, v = _synthetic_camera_project([target_deck])[0]
    side, pose = point_to_pose(cal, u, v, dims=dims)
    assert side == 'left'
    q = tuple(pose[f'left_arm_{j}_joint'] for j in JOINTS)
    pts, _ = fk(q, dims)
    assert pts[-1][0] == pytest.approx(-0.030, abs=1e-5)
    assert pts[-1][1] == pytest.approx(0.030, abs=1e-5)

    # ER 2 normalized [y, x] in 0..1000 maps through the same pipeline.
    er2_yx = [v / 360.0 * 1000.0, u / 640.0 * 1000.0]
    side2, pose2 = er2_point_to_pose(cal, er2_yx, dims=dims)
    assert side2 == 'left'
    assert pose2 == pose

    # Clicking in the dead zone between lanes is refused before any IK/motion.
    u_dead, v_dead = _synthetic_camera_project([(0.0, 0.0)])[0]
    with pytest.raises(ValueError, match='lateral limit'):
        point_to_pose(cal, u_dead, v_dead, dims=dims)

    # Clicking far outside arm reach on a valid lane is refused.
    u_far, v_far = _synthetic_camera_project([(350.0, 81.0)])[0]
    with pytest.raises(ValueError, match='unreachable or unsafe'):
        point_to_pose(cal, u_far, v_far, dims=dims)
