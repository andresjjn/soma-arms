"""The reach map must agree with the model the rest of the repo trusts.

Pure tests, no ROS. Coarse grids (5 mm, 2 deg) keep them fast; the
numbers pinned here were produced by the script itself at full
resolution on 2026-10-03 and rounded to the coarse grid.
"""
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))
sys.path.insert(0, str(REPO / 'soma_driver'))

import workspace_map as wm  # noqa: E402
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

COARSE = dict(x_step_mm=5, yaw_step_deg=2.0)


@pytest.fixture(scope='module')
def dims():
    return wm.load_dimensions()


def runs(z_mm, side, dims, **kw):
    return wm.reach_runs(z_mm / 1000.0, side, dims, **COARSE, **kw)


def test_home_fingertip_height_is_predicted_within_3_mm_of_the_bench(dims):
    # 2026-10-27, measured with the arm hanging: the closed tip 38 mm over
    # the plate. The FK has the clavicle rising 15.6 mm to the shoulder
    # (clavicle_angle) and the rest hanging straight: 327.2 + 15.6 - 306.
    pts, phi = wm.fk((0.0, 0.0, 0.0, 0.0), dims)
    assert pts[1][1] - dims['mount_h'] == pytest.approx(0.0156, abs=5e-4)   # shoulder above the disc
    assert pts[-1][1] == pytest.approx(0.038, abs=0.003)
    assert pts[-1][0] == pytest.approx(pts[1][0], abs=1e-9)                # hangs below the shoulder
    assert phi == 0.0


def test_the_left_elbow_bend_of_the_smoke_test_lifts_the_tool_296_mm(dims):
    # The smoke test clamps the left elbow at its real limit, -1.8850 rad;
    # forearm 76 + direct tool 150 = 226 mm swing up by 1 - cos. (The URDF
    # builds the tool from its parts, 149.1 mm, and TF there reads 1 mm less.)
    rest, _ = wm.fk((0.0, 0.0, 0.0, 0.0), dims)
    bent, _ = wm.fk((0.0, 0.0, -1.8850, 0.0), dims)
    assert bent[-1][1] - rest[-1][1] == pytest.approx(0.296, abs=1e-3)


def test_ik_round_trip_recovers_every_configuration(dims):
    rng = random.Random(2026)
    for _ in range(200):
        q = tuple(rng.uniform(-2.0, 2.0) for _ in range(4))
        pts, phi = wm.fk(q, dims)
        branches = wm.ik(pts[-1], phi, q[0], dims)
        assert branches, q
        for sol in branches:
            again, _ = wm.fk(sol, dims)
            assert math.dist(again[-1], pts[-1]) < 1e-9
        assert any(all(abs(wm.wrap(a - b)) < 1e-6 for a, b in zip(sol, q))
                   for sol in branches)


def test_unreachable_targets_give_no_branch(dims):
    assert wm.ik((1.0, 0.0), 0.0, 0.0, dims) == []


def test_lanes_are_symmetric_and_outboard_of_the_box(dims):
    # 162 mm between the closed tips, both arms hanging (2026-10-27).
    assert wm.lane_y('left', dims) == pytest.approx(0.081, abs=1e-6)
    assert wm.lane_y('right', dims) == pytest.approx(-0.081, abs=1e-6)
    # The model's own geometry lands on the same lane: box, disc, offset.
    assert dims['mount_ly'] / 2 + dims['disc_h'] + dims['plane_offset'] == pytest.approx(0.081)
    assert wm.lane_y('left', dims) > dims['mount_ly'] / 2


def test_plate_level_reach_is_a_narrow_band_around_the_axis(dims):
    r = runs(10, 'right', dims)
    assert len(r) == 1
    a, b = r[0]
    assert a <= 0 <= b
    # Since 2026-10-27 (the clavicle rises 15.6 mm and the disc sits at
    # 327.2) the hanging tip is 37 mm up: reaching 10 mm folds the yaw and
    # the band grows to about -85 .. +60 mm.
    assert max(abs(a), abs(b)) <= 100


def test_reach_widens_from_the_plate_up_to_fifty_mm(dims):
    # The left arm: the right's short yaw and wrist bands make its lane
    # lopsided since the 2026-10-27 geometry (front reach gone by 50 mm).
    widths = []
    for z in (10, 20, 30, 50):
        r = runs(z, 'left', dims)
        assert len(r) == 1, z
        widths.append(r[0][1] - r[0][0])
    assert widths == sorted(widths) and len(set(widths)) == 4


def test_right_front_reach_collapses_at_110_mm_because_of_its_wrist(dims):
    # The right wrist pitch may lean only 22 deg forward (+0.4869 rad).
    assert all(b < 0 for _, b in runs(110, 'right', dims))
    assert any(b > 0 for _, b in runs(110, 'left', dims))


def test_a_tool_tilt_lengthens_the_lane(dims):
    strict = runs(60, 'right', dims)
    tilted = runs(60, 'right', dims, tilt_deg=15.0)
    assert tilted[0][0] < strict[0][0] and tilted[-1][1] > strict[-1][1]


def test_solve_returns_command_frame_values_inside_the_map(dims):
    # (0.05, 0.06) was reachable on nominal radians; real radians
    # (2026-10-26) and the rising clavicle (2026-10-27) shrank the right
    # arm's level reach: (0.03, 0.03) is a point it still holds level.
    q = wm.solve((0.03, 0.03), 0.0, 'right', dims, yaw_step_deg=2.0)
    assert q is not None
    for joint, value in zip(wm.JOINTS, q):
        spec = SERVO_MAP[f'right_arm_{joint}_joint']
        assert spec.lower + 0.1 <= value <= spec.upper - 0.1, joint
    pts, phi = wm.fk(q, dims)     # under the +1 hypothesis command == model
    assert math.dist(pts[-1], (0.03, 0.03)) < 1e-9
    assert abs(phi) < 1e-9
    assert wm.solve((0.4, 0.06), 0.0, 'right', dims, yaw_step_deg=2.0) is None


def test_flipping_a_sign_mirrors_its_limits():
    signs = dict(wm.SIGNS, elbow=-1)
    assert wm.limits('right', signs)['elbow'] == pytest.approx((-0.4771 + 0.1, 1.9404 - 0.1))
    assert wm.limits('right')['elbow'] == pytest.approx((-1.9404 + 0.1, 0.4771 - 0.1))


def test_pockets_are_centered_on_the_run_or_refused():
    assert wm.pockets((-120, 120), 70, 3, 46) == [-70.0, 0.0, 70.0]
    assert wm.pockets((-40, 40), 70, 3, 46) is None
    assert wm.longest([(-10, 5), (20, 90)]) == (20, 90)
    assert wm.longest([]) is None


def test_markdown_table_has_one_row_per_deck(dims):
    rows = wm.reach_table([0, 40], 20, dims, wm.SIGNS, 0.1, **COARSE)
    table = wm.markdown_table(rows)
    assert table.count('\n') == 3
    assert '| 40 | 60 |' in table


def test_svg_is_well_formed_with_two_lanes_162_mm_apart(dims):
    svg = wm.render_svg(dims, {'left': [(-120, 130)], 'right': [(-120, 120)]},
                        {'left': [-70.0, 0.0, 70.0], 'right': None}, 46, 40)
    root = ET.fromstring(svg)
    ns = {'s': 'http://www.w3.org/2000/svg'}
    lanes = root.findall(".//s:line[@class='lane']", ns)
    assert len(lanes) == 2
    y = {ln.get('id'): float(ln.get('y1')) for ln in lanes}
    assert y['left_lane_0'] - y['right_lane_0'] == pytest.approx(-162.0, abs=0.2)
    assert len(root.findall(".//s:rect[@class='pocket']", ns)) == 3
    assert root.get('width').endswith('mm')
