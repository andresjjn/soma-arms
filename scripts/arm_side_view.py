#!/usr/bin/env python3
"""The arm's real joint angles, read by the OAK from the side.

Run on the Jetson host with its system Python (depthai 2.32, OpenCV,
numpy), outside the driver's container. It reads the camera and its IMU
and commands nothing.

Why: on 2026-10-27 the hanging right arm measured about 293 mm from the
disc to the closed tip, against 338.7 mm of chain: somewhere it is not
straight. A side view reads each link's tilt directly, and an angle in a
picture does not depend on scale (no distance, no depth error).

Setup: the OAK on a tripod about 45 cm from the arm, looking at it along
the joint axes (the arm's plane parallel to the sensor), at elbow height.
A small square of blue tape on the center of each axis screw down the arm
(shoulder, elbow, wrist pitch) and one on the closed fingertip; a fifth on
the disc center if it can be reached. Nothing else blue in the frame.

    python3 scripts/arm_side_view.py --forward right      # the robot's front is on the image right

"Down" comes from the OAK's own IMU (a BMI270 on this unit): the
accelerometer at rest reads the reaction to gravity; rotated into the
color camera's frame (IMU_TO_CAM), its projection on the image plane is
down in the picture, however the camera is mounted. Its
part along the optical axis is the camera's pitch; past PITCH_WARN_DEG the
perspective starts to bend vertical links and the script says so.

Output: each link's tilt from the vertical (positive = toward the robot's
front), each joint's angle (the difference between consecutive links),
and each link's length in mm scaled by the upper arm (REF_MM), as a check
on the markers. An annotated image and a json go to OUT_DIR.

Only the pure helpers sit at module level, so the tests import this file
without a camera, numpy or OpenCV.
"""
import argparse
import json
import math
import os
import time

OUT_DIR = '/tmp/soma_cam/side'
PITCH_WARN_DEG = 5.0
REF_MM = 80.0                # shoulder to elbow, caliper 2026-10-27
MIN_BLOB_PX = 30
NAMES_4 = ('shoulder', 'elbow', 'wrist', 'tip')
NAMES_5 = ('disc',) + NAMES_4
# IMU axes to color camera axes (x right, y down, z forward), MEASURED on
# this unit on 2026-10-27 in two poses: upright and level, the
# accelerometer read (-0.08, +9.69, -0.46), so camera y = -IMU y; looking
# steeply down from the tripod it read (4.0, 0.8, 8.9), so camera z =
# -IMU z; a right-handed frame leaves camera x = IMU x, and the tripod
# reading then puts up on the right of the raw frame, the quarter turn
# the software used to undo. The factory IMU-to-camera extrinsics read as
# a swap of x and y, in some other axis convention: not used.
IMU_TO_CAM = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def imu_to_camera(accel_imu, rot) -> tuple[float, float, float]:
    """A vector in the IMU frame rotated into the camera frame (3x3 rot)."""
    return tuple(sum(rot[r][c] * accel_imu[c] for c in range(3)) for r in range(3))


def image_down(up_cam) -> tuple[float, float]:
    """Unit 'down' in image pixels (x right, y down) from the camera-frame up."""
    dx, dy = -up_cam[0], -up_cam[1]
    n = math.hypot(dx, dy)
    if n < 1e-6:
        raise ValueError('the camera looks straight up or down: no down in the picture')
    return dx / n, dy / n


def camera_pitch_deg(up_cam) -> float:
    """How far the optical axis is from horizontal, degrees."""
    n = math.sqrt(sum(v * v for v in up_cam))
    return math.degrees(math.asin(max(-1.0, min(1.0, up_cam[2] / n))))


def link_tilts(points, down, forward_sign: int) -> list[float]:
    """Each link's angle from the vertical, degrees, + toward the front.

    `points` run down the chain (shoulder first). `down` is the image's
    unit down; `forward_sign` +1 if the robot's front is on the image right.
    """
    rx, ry = down[1], -down[0]           # down turned a quarter: the picture's right
    tilts = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        vx, vy = x1 - x0, y1 - y0
        along = vx * down[0] + vy * down[1]
        across = vx * rx + vy * ry
        tilts.append(forward_sign * math.degrees(math.atan2(across, along)))
    return tilts


def joint_angles(tilts: list[float]) -> list[float]:
    """Each joint's bend: a link's tilt minus the one before it."""
    return [b - a for a, b in zip(tilts, tilts[1:])]


def lengths_mm(points, ref_index: int = 0, ref_mm: float = REF_MM) -> list[float]:
    """Link lengths in mm, scaled so link `ref_index` reads `ref_mm`."""
    px = [math.dist(a, b) for a, b in zip(points, points[1:])]
    scale = ref_mm / px[ref_index]
    return [p * scale for p in px]


def order_down(points, down) -> list[tuple[float, float]]:
    """Markers sorted along 'down': fine for a hanging or mildly bent arm."""
    return sorted(points, key=lambda p: p[0] * down[0] + p[1] * down[1])


def capture(seconds: float = 1.5):
    """(rgb frame, mean IMU accel in the IMU frame)."""
    import depthai as dai
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    for name, out in (('rgb', cam.isp), ('imu', imu.out)):
        x = p.create(dai.node.XLinkOut)
        x.setStreamName(name)
        out.link(x.input)
    with dai.Device(p) as dev:
        q_rgb, q_imu = dev.getOutputQueue('rgb', 2, False), dev.getOutputQueue('imu', 50, False)
        samples, frame, end = [], None, time.time() + seconds
        while time.time() < end or frame is None or not samples:
            m = q_imu.tryGet()
            if m is not None:
                samples += [(k.acceleroMeter.x, k.acceleroMeter.y, k.acceleroMeter.z)
                            for k in m.packets]
            f = q_rgb.tryGet()
            if f is not None:
                frame = f.getCvFrame()
            time.sleep(0.005)
    accel = tuple(sum(s[i] for s in samples) / len(samples) for i in range(3))
    return frame, accel


def parse_roi(text: str | None) -> tuple[float, float, float, float] | None:
    """'x0,y0,x1,y1' in fractions of the frame, or None for all of it."""
    if not text:
        return None
    vals = tuple(float(v) for v in text.split(','))
    if len(vals) != 4 or not (0 <= vals[0] < vals[2] <= 1 and 0 <= vals[1] < vals[3] <= 1):
        raise ValueError('roi is x0,y0,x1,y1 with 0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1')
    return vals


def blue_markers(frame, expected: int, roi=None):
    """Centers of the `expected` largest blue blobs, or fewer if not found."""
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Wide on purpose: under the bench's warm light blue tape washes out
    # (2026-10-27: a blue note read hue 100 to 105 at low saturation).
    mask = cv2.inRange(hsv, (90, 70, 50), (130, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if roi is not None:                  # keep only the arm's region
        h, w = mask.shape
        keep = np.zeros_like(mask)
        x0, y0, x1, y1 = round(roi[0] * w), round(roi[1] * h), round(roi[2] * w), round(roi[3] * h)
        keep[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        mask = keep
    count, _, stats, cents = cv2.connectedComponentsWithStats(mask)
    blobs = sorted(((stats[i, cv2.CC_STAT_AREA], tuple(cents[i])) for i in range(1, count)
                    if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_PX), reverse=True)
    return [c for _, c in blobs[:expected]], mask


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--forward', choices=('right', 'left'), required=True,
                    help="which side of the image the robot's front is on")
    ap.add_argument('--markers', type=int, choices=(4, 5), default=4,
                    help='4: shoulder, elbow, wrist, tip; 5: the disc center too')
    ap.add_argument('--roi', help='x0,y0,x1,y1 in fractions: look for markers only there')
    ap.add_argument('--out', default=OUT_DIR)
    args = ap.parse_args(argv)
    roi = parse_roi(args.roi)
    import cv2
    os.makedirs(args.out, exist_ok=True)
    frame, accel = capture()
    up = imu_to_camera(accel, IMU_TO_CAM)
    down = image_down(up)
    pitch = camera_pitch_deg(up)
    print(f'camera pitch {pitch:+.1f} deg (level within {PITCH_WARN_DEG:g} for clean angles), '
          f'down in the picture ({down[0]:+.3f}, {down[1]:+.3f})', flush=True)
    names = NAMES_5 if args.markers == 5 else NAMES_4
    found, mask = blue_markers(frame, len(names), roi)
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    cv2.imwrite(os.path.join(args.out, f'{stamp}_raw.jpg'), frame)
    cv2.imwrite(os.path.join(args.out, f'{stamp}_mask.png'), mask)
    if len(found) < len(names):
        print(f'found {len(found)} blue markers, expected {len(names)}: see {args.out}')
        return 1
    pts = order_down(found, down)
    sign = 1 if args.forward == 'right' else -1
    tilts = link_tilts(pts, down, sign)
    joints = joint_angles(tilts)
    ref = names.index('shoulder')
    lens = lengths_mm(pts, ref_index=ref)
    links = [f'{a}-{b}' for a, b in zip(names, names[1:])]
    result = {'camera_pitch_deg': round(pitch, 1), 'image_down': [round(v, 4) for v in down],
              'links': {k: {'tilt_deg': round(t, 1), 'length_mm': round(l, 1)}
                        for k, t, l in zip(links, tilts, lens)},
              'joints_deg': {n: round(j, 1) for n, j in zip(names[1:-1], joints)},
              'markers_px': {n: [round(p[0], 1), round(p[1], 1)] for n, p in zip(names, pts)}}
    if abs(pitch) > PITCH_WARN_DEG:
        result['warning'] = f'camera pitched {pitch:.1f} deg: level it for clean angles'
    view = frame.copy()
    for (a, b) in zip(pts, pts[1:]):
        cv2.line(view, tuple(map(int, a)), tuple(map(int, b)), (0, 255, 255), 3)
    for n, p in zip(names, pts):
        cv2.circle(view, tuple(map(int, p)), 9, (0, 0, 255), 2)
        cv2.putText(view, n, (int(p[0]) + 12, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    c = (60, 60)
    cv2.arrowedLine(view, c, (int(c[0] + 80 * down[0]), int(c[1] + 80 * down[1])), (0, 255, 0), 3)
    cv2.imwrite(os.path.join(args.out, f'{stamp}_side.jpg'), view)
    with open(os.path.join(args.out, f'{stamp}_side.json'), 'w') as f:
        json.dump(result, f, indent=1)
    print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
