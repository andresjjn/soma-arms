#!/usr/bin/env python3
"""Does the camera see the arm? A depth difference against the empty bench.

Run on the Jetson host with its system Python (depthai 2.32, OpenCV,
numpy), outside the driver's container. It reads the camera and
/joint_states and commands nothing:

    python3 scripts/camera_coverage.py --baseline     # arms hanging, nobody in view
    python3 scripts/camera_coverage.py --watch        # then play camera_check

With --watch it polls /joint_states once a second; when the bases settle
on a new pose it averages two seconds of depth, finds what came closer to
the camera than the baseline (the arm), and reports whether that blob is
inside the frame or cut by an edge, how big it is and how far. Each pose
leaves an annotated image and a line in summary.json under OUT_DIR. In
another terminal, after arming (Andres's call, rule 1):

    soma ros2 run soma_driver soma_primitives camera_check --step

Nobody in view during the whole run: a person moving is also "closer than
the baseline". Frames are turned like scripts/oak_view.py (the mast mount
of 2026-10-13, a quarter turn).

Only the pure helpers sit at module level, so the tests import this file
without a camera, numpy or OpenCV.
"""
import argparse
import json
import os
import subprocess
import sys
import time

OUT_DIR = '/tmp/soma_cam/coverage'
CLOSER_MM = 40          # a pixel this much closer than the baseline changed
COLOR_DELTA = 35        # gray levels: a pixel this much brighter or darker changed
                        # (catches metal inside the stereo blind zone, no depth)
MIN_BLOB = 0.002        # of the frame: smaller blobs are noise
EDGE_PX = 3             # a blob this close to a border is cut by it
SETTLE_READS = 2        # identical /joint_states reads before capturing
YAWS = ('right_arm_yaw_joint', 'left_arm_yaw_joint')


def parse_joint_states(text: str) -> dict[str, float]:
    """`ros2 topic echo --once /joint_states` output to {joint: position}."""
    names, positions, current = [], [], None
    for raw in text.splitlines():
        line = raw.strip()
        if line in ('name:', 'position:', 'velocity:', 'effort:', 'header:'):
            current = line[:-1]
            continue
        if line.startswith('- ') and current in ('name', 'position'):
            value = line[2:].strip()
            if current == 'name':
                names.append(value)
            else:
                try:
                    positions.append(float(value))
                except ValueError:
                    return {}
        elif line and not line.startswith('- '):
            current = None
    return dict(zip(names, positions)) if len(names) == len(positions) else {}


def pose_name(state: dict[str, float]) -> str:
    """'home', or the raised bases in degrees forward, e.g. 'right_30'."""
    raised = []
    for joint in YAWS:
        deg = round(-state.get(joint, 0.0) * 180.0 / 3.141592653589793)
        if abs(deg) >= 3:
            raised.append(f"{joint.split('_')[0]}_{deg}")
    return '+'.join(raised) or 'home'


def verdict(bbox: tuple[int, int, int, int] | None, width: int, height: int,
            edge_px: int = EDGE_PX) -> tuple[str, list[str]]:
    """('in frame' | 'cut at ...' | 'not seen', edges touched) for a blob box."""
    if bbox is None:
        return 'not seen', []
    x, y, w, h = bbox
    edges = [name for name, touch in (
        ('left', x <= edge_px), ('top', y <= edge_px),
        ('right', x + w >= width - edge_px), ('bottom', y + h >= height - edge_px))
        if touch]
    return ('cut at ' + ' and '.join(edges) if edges else 'in frame'), edges


def read_joint_states(timeout_s: float = 8.0) -> dict[str, float]:
    """One /joint_states message through the driver's container, or {}."""
    inside = '. /opt/ros/humble/setup.bash && exec "$@"'
    cmd = ['docker', 'exec', 'soma_driver', 'bash', '-c', inside, 'soma',
           'timeout', '5', 'ros2', 'topic', 'echo', '--once', '/joint_states']
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (subprocess.TimeoutExpired, OSError):
        return {}
    return parse_joint_states(done.stdout)


class Camera:
    """RGB plus depth in millimetres aligned to it, both turned upright."""

    def __init__(self) -> None:
        import cv2
        import depthai as dai
        self.cv2 = cv2
        p = dai.Pipeline()
        left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
        for mono, socket in ((left, dai.CameraBoardSocket.CAM_B),
                             (right, dai.CameraBoardSocket.CAM_C)):
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono.setBoardSocket(socket)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)          # parts of the arm come within 34 cm
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        rgb = p.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setIspScale(1, 3)
        for name, out in (('depth', stereo.depth), ('rgb', rgb.isp)):
            x = p.create(dai.node.XLinkOut)
            x.setStreamName(name)
            out.link(x.input)
        self.device = dai.Device(p)
        self.q_depth = self.device.getOutputQueue('depth', 4, False)
        self.q_rgb = self.device.getOutputQueue('rgb', 4, False)

    def capture(self, seconds: float = 2.0):
        """(rgb, median depth in mm) over `seconds` of fresh frames."""
        import numpy as np
        cv2 = self.cv2
        turn = cv2.ROTATE_90_COUNTERCLOCKWISE      # = oak_view.ROTATION
        while self.q_depth.tryGet() is not None:   # drop stale frames
            pass
        frames, rgb, end = [], None, time.time() + seconds
        while time.time() < end or not frames or rgb is None:
            d = self.q_depth.tryGet()
            if d is not None:
                frames.append(d.getFrame())
            r = self.q_rgb.tryGet()
            if r is not None:
                rgb = r.getCvFrame()
            time.sleep(0.01)
        depth = np.median(np.stack(frames), axis=0).astype(np.float32)
        depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        return cv2.rotate(rgb, turn), cv2.rotate(depth, turn)

    def close(self) -> None:
        self.device.close()


def find_arm(base, depth):
    """The largest blob that came CLOSER_MM nearer than the baseline."""
    return _largest_blob((base > 0) & (depth > 0) & (base - depth > CLOSER_MM))


def find_by_color(base_rgb, rgb):
    """The largest blob whose gray level moved by COLOR_DELTA (depth-blind)."""
    import cv2
    import numpy as np
    gray = [cv2.GaussianBlur(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (7, 7), 0)
            for im in (base_rgb, rgb)]
    return _largest_blob(cv2.absdiff(gray[0], gray[1]) > COLOR_DELTA)


def _largest_blob(changed):
    import cv2
    import numpy as np
    changed = cv2.morphologyEx(changed.astype(np.uint8), cv2.MORPH_OPEN,
                               np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(changed)
    if count <= 1:
        return None, None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = (int(v) for v in stats[best])
    if area < MIN_BLOB * changed.size:
        return None, None
    return (x, y, w, h), labels == best


def report(name: str, rgb, depth, base, out_dir: str, base_rgb=None) -> dict:
    import cv2
    import numpy as np
    bbox, mask = find_arm(base, depth)
    h, w = depth.shape
    text, edges = verdict(bbox, w, h)
    row = {'pose': name, 'verdict': text, 'edges': edges}
    if base_rgb is not None:
        cbox, cmask = find_by_color(base_rgb, rgb)
        ctext, cedges = verdict(cbox, w, h)
        row['color'] = {'verdict': ctext, 'edges': cedges}
        if cbox is not None:
            cx, cy, cw, ch = cbox
            row['color'].update(area=round(float(cmask.mean()), 4),
                                bbox=[round(cx / w, 3), round(cy / h, 3),
                                      round(cw / w, 3), round(ch / h, 3)])
    view = rgb.copy()
    if bbox is not None:
        near = depth[mask & (depth > 0)]
        x, y, bw, bh = bbox
        row.update(area=round(float(mask.mean()), 4),
                   bbox=[round(x / w, 3), round(y / h, 3), round(bw / w, 3), round(bh / h, 3)],
                   nearest_mm=int(np.percentile(near, 5)), median_mm=int(np.median(near)))
        view[mask] = (0.5 * view[mask] + (0, 127, 0)).astype(view.dtype)
        cv2.rectangle(view, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
    if base_rgb is not None and row['color'].get('bbox'):
        cx, cy, cw, ch = cbox
        view[cmask] = (0.5 * view[cmask] + (127, 0, 127)).astype(view.dtype)
        cv2.rectangle(view, (cx, cy), (cx + cw, cy + ch), (255, 0, 255), 2)
    cv2.putText(view, f'{name}: {text}', (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)
    cv2.imwrite(os.path.join(out_dir, f'{name}.jpg'), view)
    return row


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--baseline', action='store_true', help='capture the empty bench')
    mode.add_argument('--watch', action='store_true', help='capture every new pose')
    mode.add_argument('--pose', metavar='NAME', help='capture once, under this name')
    ap.add_argument('--out', default=OUT_DIR)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    base_path = os.path.join(args.out, 'baseline.npz')
    if not args.baseline and not os.path.exists(base_path):
        print(f'no baseline yet: run --baseline first ({base_path})', file=sys.stderr)
        return 2

    import numpy as np
    cam = Camera()
    try:
        if args.baseline:
            rgb, depth = cam.capture(3.0)
            np.savez_compressed(base_path, depth=depth, rgb=rgb)
            cam.cv2.imwrite(os.path.join(args.out, 'baseline.jpg'), rgb)
            valid = depth[depth > 0]
            print(f'baseline saved: {valid.size / depth.size:.0%} valid depth, '
                  f'median {np.median(valid):.0f} mm')
            return 0
        saved = np.load(base_path)
        base = saved['depth']
        base_rgb = saved['rgb'] if 'rgb' in saved.files else None
        if args.pose:
            rgb, depth = cam.capture()
            print(json.dumps(report(args.pose, rgb, depth, base, args.out, base_rgb)))
            return 0
        rows, last, seen, streak = [], None, None, 0
        print('watching /joint_states; play camera_check now. Ctrl-C ends.', flush=True)
        while True:
            state = read_joint_states()
            if not state:
                print('no /joint_states (is the driver up?)', flush=True)
                time.sleep(1.0)
                continue
            name = pose_name(state)
            streak = streak + 1 if name == seen else 1
            seen = name
            if streak >= SETTLE_READS and name != last:
                rgb, depth = cam.capture()
                row = report(name, rgb, depth, base, args.out, base_rgb)
                rows.append(row)
                last = name
                print(json.dumps(row), flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        if args.watch:
            with open(os.path.join(args.out, 'summary.json'), 'w') as f:
                json.dump(rows, f, indent=1)
            print(f'\n{len(rows)} poses, summary in {args.out}/summary.json')
        return 0
    finally:
        cam.close()


if __name__ == '__main__':
    raise SystemExit(main())
