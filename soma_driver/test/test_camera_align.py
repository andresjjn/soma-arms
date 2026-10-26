"""scripts/camera_align.py: the offsets, their grades and the reference record.

Pure tests: the camera, numpy and OpenCV are imported inside the
functions that need them, so this runs anywhere.
"""
import json
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import camera_align as ca  # noqa: E402

W, H = ca.WIDTH, ca.HEIGHT
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]


def similarity(dx, dy, deg=0.0, scale=1.0, about=(W / 2, H / 2)):
    """A similarity that turns and scales about a point, then shifts."""
    c, s = scale * math.cos(math.radians(deg)), scale * math.sin(math.radians(deg))
    px, py = about
    return [[c, -s, px - c * px + s * py + dx], [s, c, py - s * px - c * py + dy]]


class TestCameraOffsets:
    def test_nothing_moved_reads_zero(self):
        assert ca.camera_offsets(IDENTITY, W, H) == {
            'dx_pct': 0.0, 'dy_pct': 0.0, 'turn_deg': 0.0, 'scale_pct': 100.0}

    def test_a_shift_reads_in_percent_of_each_side(self):
        cam = ca.camera_offsets(similarity(W * 0.05, -H * 0.02), W, H)
        assert cam['dx_pct'] == pytest.approx(5.0)
        assert cam['dy_pct'] == pytest.approx(-2.0)

    def test_turn_and_scale_about_the_center_do_not_read_as_shift(self):
        cam = ca.camera_offsets(similarity(0, 0, deg=1.5, scale=1.02), W, H)
        assert cam['turn_deg'] == pytest.approx(1.5)
        assert cam['scale_pct'] == pytest.approx(102.0)
        assert cam['dx_pct'] == pytest.approx(0.0, abs=0.01)
        assert cam['dy_pct'] == pytest.approx(0.0, abs=0.01)


class TestRobotOffsets:
    BOX = ca.to_px(ca.DEFAULT_ROBOT_BOX, W, H)

    def test_robot_that_moved_with_the_camera_did_not_move(self):
        m = similarity(30, -12)
        cx, cy = ca.box_center(self.BOX)
        rob = ca.robot_offsets(m, self.BOX, (cx + 30, cy - 12), W, H)
        assert rob['dx_pct'] == pytest.approx(0.0) and rob['dy_mm'] == 0

    def test_robot_move_reads_against_the_scene_in_mm(self):
        cx, cy = ca.box_center(self.BOX)
        rob = ca.robot_offsets(IDENTITY, self.BOX, (cx + 20, cy), W, H)
        assert rob['dx_pct'] == pytest.approx(100 * 20 / W, abs=0.01)
        # 2.7 mm/px at 360 px wide is 1.8 mm/px at 540
        assert rob['dx_mm'] == 36

    def test_center_line(self):
        assert ca.center_offset_pct(W / 2, W) == 0.0
        assert ca.center_offset_pct(W * 0.6, W) == pytest.approx(10.0)

    def test_default_box_is_centered_and_inside_the_frame(self):
        x, y, w, h = ca.DEFAULT_ROBOT_BOX
        assert x + w / 2 == pytest.approx(0.5)
        assert 0 <= y and y + h <= 1


class TestGrades:
    @pytest.mark.parametrize('value, expected', [
        (0.0, 'ok'), (1.0, 'ok'), (-1.01, 'near'), (3.0, 'near'), (3.5, 'off'), (-9, 'off')])
    def test_shift(self, value, expected):
        assert ca.grade(value, ca.SHIFT_OK, ca.SHIFT_WARN) == expected

    def test_the_worst_reading_sets_the_camera_grade(self):
        cam = {'dx_pct': 0.2, 'dy_pct': 0.1, 'turn_deg': 2.5, 'scale_pct': 100.4}
        assert ca.camera_grade(cam) == 'off'
        cam['turn_deg'] = 0.3
        assert ca.camera_grade(cam) == 'ok'
        cam['scale_pct'] = 97.5
        assert ca.camera_grade(cam) == 'near'

    def test_robot_grade(self):
        assert ca.robot_grade({'dx_pct': 0.5, 'dy_pct': -2.0}) == 'near'


class TestReference:
    def test_a_saved_record_loads_back(self, tmp_path):
        image = tmp_path / 'ref.jpg'
        record = ca.reference_record('2026-10-26', (0.2, 0.5, 0.6, 0.3))
        (tmp_path / 'ref.json').write_text(json.dumps(record))
        assert ca.load_reference(str(image))['robot_box'] == (0.2, 0.5, 0.6, 0.3)

    def test_an_image_alone_gets_the_default_box(self, tmp_path):
        assert ca.load_reference(str(tmp_path / 'x.jpg'))['robot_box'] == ca.DEFAULT_ROBOT_BOX

    @pytest.mark.parametrize('box', [[0.5, 0.5, 0.6, 0.2], [0.1, 0.1, 0.2], [-0.1, 0, 0.2, 0.2]])
    def test_a_box_outside_the_frame_is_refused(self, tmp_path, box):
        (tmp_path / 'ref.json').write_text(json.dumps({'robot_box': box}))
        with pytest.raises(ValueError):
            ca.load_reference(str(tmp_path / 'ref.jpg'))

    def test_the_record_says_the_arms_hang(self):
        assert 'hanging' in ca.reference_record('2026-10-26')['pose']


@pytest.mark.parametrize('line, key', [
    ('s\n', 'save'), (' G ', 'ghost'), ('q', 'quit'), ('', None), ('arm', None)])
def test_keys(line, key):
    assert ca.parse_key(line) == key


def test_it_never_reaches_the_driver():
    source = (REPO / 'scripts' / 'camera_align.py').read_text()
    for word in ('docker', 'ros2', '/soma/arm', 'allow_real', 'rclpy'):
        assert word not in source
