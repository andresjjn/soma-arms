"""scripts/arm_side_view.py: down from the IMU, link tilts, joint angles.

Pure tests: the camera, numpy and OpenCV are imported inside the
functions that need them.
"""
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import arm_side_view as sv  # noqa: E402

ROT = sv.IMU_TO_CAM


def test_the_upright_reading_of_2026_10_27_sees_down_as_image_down():
    # The camera stood level beside the robot, which stood upright in the frame.
    up_cam = sv.imu_to_camera((-0.079, 9.694, -0.464), ROT)
    dx, dy = sv.image_down(up_cam)
    assert dx == pytest.approx(0.0, abs=0.01) and dy == pytest.approx(1.0, abs=1e-3)
    assert abs(sv.camera_pitch_deg(up_cam)) < 5


def test_a_camera_rolled_a_quarter_turn_sees_down_sideways():
    dx, dy = sv.image_down((-9.81, 0.0, 0.0))     # up along camera -x
    assert (dx, dy) == pytest.approx((1.0, 0.0))


def test_looking_straight_down_has_no_down_in_the_picture():
    with pytest.raises(ValueError):
        sv.image_down((0.0, 0.0, -9.81))


def test_the_tripod_reading_of_2026_10_27_is_a_steep_look_down_rolled_a_quarter():
    up = sv.imu_to_camera((4.015, 0.81, 8.866), ROT)
    assert sv.camera_pitch_deg(up) < -55
    dx, dy = sv.image_down(up)
    assert dx < -0.95          # down on the raw frame's left: the turn oak_view undid


def test_a_straight_hanging_arm_reads_zero_everywhere():
    pts = [(500, 100), (500, 300), (500, 490), (500, 865)]
    tilts = sv.link_tilts(pts, (0.0, 1.0), +1)
    assert tilts == pytest.approx([0.0, 0.0, 0.0])
    assert sv.joint_angles(tilts) == pytest.approx([0.0, 0.0])


def test_a_forearm_bent_toward_the_front_reads_positive_at_the_elbow():
    l = 190
    a = math.radians(30)
    elbow = (500, 300)
    wrist = (500 + l * math.sin(a), 300 + l * math.cos(a))     # front is image right
    tip = (wrist[0] + 375 * math.sin(a), wrist[1] + 375 * math.cos(a))
    tilts = sv.link_tilts([(500, 100), elbow, wrist, tip], (0.0, 1.0), +1)
    assert tilts == pytest.approx([0.0, 30.0, 30.0], abs=1e-6)
    assert sv.joint_angles(tilts) == pytest.approx([30.0, 0.0], abs=1e-6)
    assert sv.link_tilts([(500, 100), elbow, wrist, tip], (0.0, 1.0), -1)[1] == pytest.approx(-30.0)


def test_tilts_are_measured_against_imu_down_not_the_frame():
    # The whole picture rolled 10 deg: a vertical arm is still vertical.
    r = math.radians(10)
    down = (-math.sin(r), math.cos(r))
    pts = [(500 + t * down[0], 100 + t * down[1]) for t in (0, 200, 390, 765)]
    assert sv.link_tilts(pts, down, +1) == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)


def test_lengths_scale_by_the_upper_arm():
    pts = [(0, 0), (0, 200), (0, 390), (0, 765)]
    assert sv.lengths_mm(pts) == pytest.approx([80.0, 76.0, 150.0])


def test_markers_are_ordered_down_the_arm():
    pts = [(10, 500), (12, 100), (11, 300)]
    assert sv.order_down(pts, (0.0, 1.0)) == [(12, 100), (11, 300), (10, 500)]


def test_it_never_reaches_the_driver():
    source = (REPO / 'scripts' / 'arm_side_view.py').read_text()
    for word in ('docker', 'ros2', '/soma/arm', 'allow_real', 'rclpy'):
        assert word not in source


def test_roi_parses_fractions_and_refuses_nonsense():
    assert sv.parse_roi('0.3,0.1,0.8,1') == (0.3, 0.1, 0.8, 1.0)
    assert sv.parse_roi(None) is None
    for bad in ('0.8,0,0.3,1', '0,0,1', '0,0,1.5,1'):
        with pytest.raises(ValueError):
            sv.parse_roi(bad)
