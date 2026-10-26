#!/usr/bin/env python3
"""Put the camera and the robot back where the reference has them.

Run on the Jetson host with its system Python (depthai 2.32, OpenCV,
numpy), outside the driver's container. It only reads the camera: no
driver, no gate, no servo. The arms HANG during the whole routine: on
this bench hanging is the stretched pose, and it is the only one that
needs no motion (rule 4).

    python3 scripts/camera_align.py
    # on the Mac: ssh -N -L 8091:localhost:8091 jetson, then open http://localhost:8091

The view shows the reference as a ghost over the live frame, the frame's
center line and the box where the robot should be. Two readings, in
percent of the frame (and about mm on the plane of the lane marks):

  CAMERA  how far the scene moved against the reference: shift, turn,
          scale. ORB features of the table and the room, the robot and
          the bottom strip (the tripod's own legs) left out, fitted with
          a similarity (shift, rotation, one scale) under RANSAC.
  ROBOT   how far the robot moved against the scene: the reference crop
          of the robot found in the live frame, minus where the camera
          shift alone would have put it.

Green is within 1 %, yellow within 3 %, red beyond. A positive turn is the
scene turned clockwise on screen. Checked on 2026-10-26 with live frames
and synthetic moves: a 2 % shift reads 2.0 %, 1 deg reads 0.99 deg, two
live frames read 0.0 %. It is a fine tool: after a big move (the camera
of that evening against the morning's reference) the scene is not
matched, and the ghost is the guide until it is close. The robot crop
does not turn or scale, so at 8 % plus 3 deg the robot reads 1.1 % off
when only the camera moved. Type in the terminal:
`s` + Enter saves the live frame as the new reference (do it once, with
the robot centered on the guide line and the arms hanging), `g` + Enter
toggles the ghost, `q` + Enter quits.

The reference lives in calibration/camera_reference.{jpg,json}. Review
the saved image before committing it: the repo is public.

Only the pure helpers sit at module level, so the tests import this file
without a camera, numpy or OpenCV.
"""
import argparse
import json
import math
import os
import select
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REFERENCE = os.path.join(REPO, 'calibration', 'camera_reference')
PORT = 8091
WIDTH, HEIGHT = 540, 960       # 1080p ISP at half scale, turned upright
# Where the robot should sit when nothing has moved, in fractions of the
# frame (x, y, w, h): centered on the frame's vertical line, at the size
# the tripod of 2026-10-26 sees it.
DEFAULT_ROBOT_BOX = (0.265, 0.58, 0.47, 0.29)
MM_PER_PX_AT_360 = 2.7         # along the lanes, calibration/camera_marks_2026-10-26.json
SHIFT_OK, SHIFT_WARN = 1.0, 3.0        # percent of the frame
TURN_OK, TURN_WARN = 0.5, 2.0          # degrees
SCALE_OK, SCALE_WARN = 1.0, 3.0        # percent away from 100
MIN_INLIERS = 25
MIN_ROBOT_SCORE = 0.5          # normalized correlation of the robot crop
ANALYZE_EVERY_S = 0.3


def apply(m, x: float, y: float) -> tuple[float, float]:
    """A 2x3 affine matrix applied to one point."""
    return (m[0][0] * x + m[0][1] * y + m[0][2],
            m[1][0] * x + m[1][1] * y + m[1][2])


def camera_offsets(m, width: int, height: int) -> dict:
    """Where the reference's frame center landed: shift %, turn deg, scale %."""
    cx, cy = width / 2, height / 2
    x, y = apply(m, cx, cy)
    return {'dx_pct': round(100 * (x - cx) / width, 2),
            'dy_pct': round(100 * (y - cy) / height, 2),
            'turn_deg': round(math.degrees(math.atan2(m[1][0], m[0][0])), 2),
            'scale_pct': round(100 * math.hypot(m[0][0], m[1][0]), 2)}


def box_center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, w, h = box
    return x + w / 2, y + h / 2


def robot_offsets(m, ref_box_px, found_center, width: int, height: int) -> dict:
    """The robot's move against the scene: found minus where the camera put it."""
    ex, ey = apply(m, *box_center(ref_box_px))
    dx, dy = found_center[0] - ex, found_center[1] - ey
    mm = MM_PER_PX_AT_360 * 360 / width
    return {'dx_pct': round(100 * dx / width, 2), 'dy_pct': round(100 * dy / height, 2),
            'dx_mm': round(dx * mm), 'dy_mm': round(dy * mm)}


def center_offset_pct(x: float, width: int) -> float:
    """A point's distance from the frame's vertical center line, % of width."""
    return round(100 * (x - width / 2) / width, 2)


def grade(value: float, ok: float, warn: float) -> str:
    v = abs(value)
    return 'ok' if v <= ok else 'near' if v <= warn else 'off'


def camera_grade(cam: dict) -> str:
    grades = [grade(cam['dx_pct'], SHIFT_OK, SHIFT_WARN),
              grade(cam['dy_pct'], SHIFT_OK, SHIFT_WARN),
              grade(cam['turn_deg'], TURN_OK, TURN_WARN),
              grade(cam['scale_pct'] - 100, SCALE_OK, SCALE_WARN)]
    return worst(grades)


def robot_grade(rob: dict) -> str:
    return worst([grade(rob['dx_pct'], SHIFT_OK, SHIFT_WARN),
                  grade(rob['dy_pct'], SHIFT_OK, SHIFT_WARN)])


def worst(grades: list[str]) -> str:
    for g in ('off', 'near'):
        if g in grades:
            return g
    return 'ok'


def to_px(box, width: int, height: int) -> tuple[int, int, int, int]:
    x, y, w, h = box
    return round(x * width), round(y * height), round(w * width), round(h * height)


def reference_record(date: str, robot_box=DEFAULT_ROBOT_BOX) -> dict:
    return {'date': date, 'frame': f'{WIDTH}x{HEIGHT}, RGB ISP 1080p at half scale, '
            'turned 90 deg counterclockwise', 'robot_box': list(robot_box),
            'pose': 'both arms hanging, driver disarmed'}


def load_reference(path: str) -> dict:
    """The reference record, or a default one when only an image is given."""
    meta = os.path.splitext(path)[0] + '.json'
    if os.path.exists(meta):
        with open(meta) as f:
            record = json.load(f)
    else:
        record = reference_record('unknown')
    box = record.get('robot_box')
    if not (isinstance(box, list) and len(box) == 4
            and all(0 <= v <= 1 for v in box) and box[0] + box[2] <= 1 and box[1] + box[3] <= 1):
        raise ValueError(f'robot_box in {meta} must be 4 fractions inside the frame')
    record['robot_box'] = tuple(box)
    return record


def parse_key(line: str) -> str | None:
    t = line.strip().lower()
    return {'s': 'save', 'g': 'ghost', 'q': 'quit'}.get(t)


class Aligner:
    """Reference features, computed once; measure(frame) reads both offsets."""

    def __init__(self, image, robot_box) -> None:
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.image = cv2.resize(image, (WIDTH, HEIGHT))
        self.box_px = to_px(robot_box, WIDTH, HEIGHT)
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.orb = cv2.ORB_create(2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.kp, self.des = self.orb.detectAndCompute(self.gray, self.scene_mask())
        x, y, w, h = self.box_px
        self.template = self.gray[y:y + h, x:x + w]

    def scene_mask(self):
        """The table and the room: not the robot, not the strip under it."""
        np = self.np
        mask = np.full((HEIGHT, WIDTH), 255, np.uint8)
        x, y, w, h = self.box_px
        pad = 12
        mask[max(0, y - pad):y + h + pad, max(0, x - pad):x + w + pad] = 0
        mask[y + h + pad:, :] = 0
        return mask

    def measure(self, frame) -> dict:
        cv2, np = self.cv2, self.np
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        out = {'camera': None, 'robot': None, 'matrix': None}
        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or self.des is None:
            return out
        matches = self.matcher.match(self.des, des)
        if len(matches) < MIN_INLIERS:
            return out
        src = np.float32([self.kp[mt.queryIdx].pt for mt in matches])
        dst = np.float32([kp[mt.trainIdx].pt for mt in matches])
        m, inliers = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
                                                 ransacReprojThreshold=3.0)
        if m is None or int(inliers.sum()) < MIN_INLIERS:
            return out
        out['matrix'] = m.tolist()
        out['camera'] = dict(camera_offsets(out['matrix'], WIDTH, HEIGHT),
                             inliers=int(inliers.sum()),
                             match_pct=round(100 * float(inliers.mean()), 1))
        result = cv2.matchTemplate(gray, self.template, cv2.TM_CCOEFF_NORMED)
        _, score, _, (fx, fy) = cv2.minMaxLoc(result)
        _, _, w, h = self.box_px
        found = (fx + w / 2, fy + h / 2)
        if score >= MIN_ROBOT_SCORE:
            out['robot'] = dict(robot_offsets(out['matrix'], self.box_px, found, WIDTH, HEIGHT),
                                score_pct=round(100 * score, 1),
                                box=(fx, fy, w, h),
                                center_pct=center_offset_pct(found[0], WIDTH))
        return out


COLORS = {'ok': (0, 200, 0), 'near': (0, 200, 255), 'off': (0, 0, 255)}


def draw(frame, reading, aligner, ghost: bool, default_box):
    import cv2
    import numpy as np
    view = frame.copy()
    if aligner is not None and ghost:
        view = cv2.addWeighted(view, 0.65, aligner.image, 0.35, 0)
    for y in range(0, HEIGHT, 16):                     # dashed center line
        cv2.line(view, (WIDTH // 2, y), (WIDTH // 2, y + 8), (255, 255, 0), 1)
    lines = []
    if aligner is None:
        x, y, w, h = to_px(default_box, WIDTH, HEIGHT)
        cv2.rectangle(view, (x, y), (x + w, y + h), (255, 255, 0), 2)
        lines.append(('no reference: arms hanging, robot in the box, then s + Enter',
                      (255, 255, 255)))
    elif reading is None or reading['camera'] is None:
        lines.append(('CAMERA  scene not matched (moved a lot, or dark?)', COLORS['off']))
    else:
        m = np.float32(reading['matrix'])
        x, y, w, h = aligner.box_px
        corners = np.float32([[x, y], [x + w, y], [x + w, y + h], [x, y + h]]).reshape(-1, 1, 2)
        target = cv2.transform(corners, m).astype(np.int32)
        cv2.polylines(view, [target], True, (255, 255, 0), 2)
        c = reading['camera']
        g = camera_grade(c)
        lines.append((f"CAMERA {g.upper():4}  dx {c['dx_pct']:+.1f}%  dy {c['dy_pct']:+.1f}%  "
                      f"turn {c['turn_deg']:+.1f} deg  scale {c['scale_pct']:.1f}%", COLORS[g]))
        lines.append((f"        match {c['match_pct']:.0f}% ({c['inliers']} features)",
                      (255, 255, 255)))
        r = reading['robot']
        if r is None:
            lines.append(('ROBOT   not found (moved a lot, or covered?)', COLORS['off']))
        else:
            g = robot_grade(r)
            fx, fy, w, h = r['box']
            cv2.rectangle(view, (fx, fy), (fx + w, fy + h), COLORS[g], 2)
            lines.append((f"ROBOT  {g.upper():4}  dx {r['dx_pct']:+.1f}% ({r['dx_mm']:+d} mm)  "
                          f"dy {r['dy_pct']:+.1f}% ({r['dy_mm']:+d} mm)", COLORS[g]))
            lines.append((f"        off the center line {r['center_pct']:+.1f}%  "
                          f"match {r['score_pct']:.0f}%", (255, 255, 255)))
    for i, (text, color) in enumerate(lines):
        org = (8, 22 + 20 * i)
        cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return view


def open_camera():
    import cv2
    import depthai as dai
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setIspScale(1, 2)
    cam.setFps(10)
    out = p.create(dai.node.XLinkOut)
    out.setStreamName('rgb')
    cam.isp.link(out.input)
    device = dai.Device(p)
    queue = device.getOutputQueue('rgb', 2, False)

    def read():
        m = queue.tryGet()
        return None if m is None else cv2.rotate(m.getCvFrame(), cv2.ROTATE_90_COUNTERCLOCKWISE)
    return device, read


def save_reference(frame, path: str, box) -> None:
    import cv2
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path + '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    with open(path + '.json', 'w') as f:
        json.dump(reference_record(time.strftime('%Y-%m-%d'), box), f, indent=1)
        f.write('\n')
    print(f'reference saved: {path}.jpg and .json (review the image before committing)',
          flush=True)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--reference', default=REFERENCE + '.jpg',
                    help='reference image; its .json beside it holds the robot box')
    ap.add_argument('--save-to', default=REFERENCE, help='where s writes, no extension')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--once', action='store_true', help='no view: print one reading and exit')
    args = ap.parse_args(argv)

    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    aligner, box = None, DEFAULT_ROBOT_BOX
    if os.path.exists(args.reference):
        box = load_reference(args.reference)['robot_box']
        aligner = Aligner(cv2.imread(args.reference), box)
        print(f'reference {args.reference}, robot box {box}', flush=True)
    else:
        print(f'no reference at {args.reference}: guides only until s', flush=True)

    publish = None
    if not args.once:
        from hand_gripper import _web_view
        publish = _web_view(args.port)
        print(f'view on localhost:{args.port}; from the Mac: '
              f'ssh -N -L {args.port}:localhost:{args.port} jetson. '
              's / g / q + Enter.', flush=True)
    device, read = open_camera()
    frame, reading, ghost, last, started = None, None, True, 0.0, time.time()
    try:
        while True:
            f = read()
            if f is not None:
                frame = f
            if frame is None or time.time() - started < 1.5:      # exposure settling
                time.sleep(0.02)
                continue
            if aligner is not None and time.time() - last > ANALYZE_EVERY_S:
                reading, last = aligner.measure(frame), time.time()
                if args.once:
                    print(json.dumps({k: v for k, v in reading.items() if k != 'matrix'}))
                    return 0
            elif aligner is None and args.once:
                print('no reference to compare against', file=sys.stderr)
                return 2
            if publish is not None:
                publish(draw(frame, reading, aligner, ghost, box))
            if select.select([sys.stdin], [], [], 0)[0]:
                key = parse_key(sys.stdin.readline())
                if key == 'quit':
                    return 0
                if key == 'ghost':
                    ghost = not ghost
                if key == 'save':
                    save_reference(frame, args.save_to, DEFAULT_ROBOT_BOX)
                    box = DEFAULT_ROBOT_BOX
                    aligner, reading = Aligner(frame, box), None
            time.sleep(0.03)
    except KeyboardInterrupt:
        return 0
    finally:
        device.close()


if __name__ == '__main__':
    raise SystemExit(main())
