#!/usr/bin/env python3
"""One gripper follows one hand: open the hand and the gripper opens with it.

A bench experiment decided by Andres on 2026-10-12, not part of the
library. The OAK-D Lite runs the MediaPipe palm detector and hand
landmarker on the camera itself (RVC2, about 8 fps), and how open the
hand is sets how open ONE gripper is, in proportion. It is two processes
joined by a pipe, because the camera lives on the Jetson host (the
depthai 3 venv) and ROS lives in the driver's container:

    # measure only, no robot: prints how open the hand reads, live, and
    # with --show draws it in a browser at http://localhost:8090 on the
    # Jetson (from the Mac: ssh -N -L 8090:localhost:8090 jetson)
    ~/venvs/depthai3/bin/python scripts/hand_gripper.py [--show]

    # drive the right gripper (arming stays a separate, explicit step)
    ~/venvs/depthai3/bin/python scripts/hand_gripper.py --emit \\
      | docker exec -i soma_driver bash -c '. /opt/ros/humble/setup.bash \\
          && . /ros2_ws/install/setup.bash \\
          && python3 /ros2_ws/src/soma-arms/scripts/hand_gripper.py --bridge right'

How open the hand is: the distance from the thumb tip to the mean of the
four fingertips, divided by the palm's size (its length, wrist to the
middle knuckle, or its width scaled to a length if that is longer, so a
hand tilted toward the lens does not inflate it). The ratio does not
change with the hand's distance to the camera, and it reads a pinch, a
claw and a fist alike as closed. Best seen with the palm to the camera
and the fingers up, as if waving at the robot. It is
mapped linearly from --closed (gripper 0.0) to --open (gripper 1.0) and
smoothed, and a new target goes out only when it moved by a deadband.

What keeps this safe:
  - The bridge publishes one joint only, finger_l of the arm it is given,
    clamped to 0..1. The driver clamps again to the soft limits and ramps
    every target (minimum jerk under the joint's max rate).
  - Nothing here can arm the driver: /soma/arm stays a manual call.
  - No hand in view, no command: the gripper holds where it is.
  - Ending either side (q, Ctrl-C, the pipe closing) sends nothing more;
    the gripper holds its last target until the driver is disarmed.
  - No force sensing: closing the hand on an object stalls the servo
    against it. Hold briefly, and open a little at a sustained hum.

Only the pure helpers sit at module level: the container has no depthai
and the host has no rclpy, so each side imports its own library inside
its own function, and the tests import neither.
"""
import argparse
import math
import sys
import time

# MediaPipe hand landmark indices.
WRIST, THUMB_TIP, INDEX_MCP, MIDDLE_MCP, PINKY_MCP = 0, 4, 5, 9, 17
FINGERTIPS = (8, 12, 16, 20)
# Palm length (wrist to middle knuckle) over palm width (index knuckle to
# little finger knuckle) on a flat adult hand, about 9 cm over 7 cm.
WIDTH_TO_LENGTH = 1.3

# Calibrated on Andres's hand, 2026-10-12, 270 s at 8 fps: a pinch read
# 0.4 to 0.5, an open hand 1.5 to 1.7; readings above 2.5 were not a hand.
CLOSED_RATIO = 0.5      # thumb on the fingertips: gripper 0.0
OPEN_RATIO = 1.5        # hand open, thumb out: gripper 1.0
SMOOTHING = 0.5         # weight of the newest reading
DEADBAND = 0.05         # smallest change worth a new target
MAX_RATIO = 2.5         # above this no real hand: a misread, skipped
ARMS = ('right', 'left')


def palm_size(kpts) -> float:
    """The palm's length, or its width scaled to a length if that is longer.

    The camera sees the hand flat, in 2D: a hand tilted toward the lens
    shows a short palm and every ratio over it inflates (readings of 2 to
    5 for an open hand on 2026-10-12). Tilting about one axis shortens
    the length or the width, rarely both, so the longer of the two is the
    better estimate of the hand's real size.
    """
    length = math.dist(kpts[WRIST], kpts[MIDDLE_MCP])
    width = math.dist(kpts[INDEX_MCP], kpts[PINKY_MCP]) * WIDTH_TO_LENGTH
    return max(length, width)


def hand_ratio(kpts) -> float | None:
    """Thumb tip to the fingertips' mean, in palm sizes; None if degenerate.

    `kpts` is the 21 MediaPipe landmarks as (x, y) in an isotropic frame
    (pixels, or normalized coordinates of a square image).
    """
    if len(kpts) < 21:
        return None
    palm = palm_size(kpts)
    if palm < 1e-6:
        return None
    tips = [kpts[i] for i in FINGERTIPS]
    mean = (sum(p[0] for p in tips) / len(tips), sum(p[1] for p in tips) / len(tips))
    return math.dist(kpts[THUMB_TIP], mean) / palm


def openness(ratio: float, closed: float = CLOSED_RATIO, open_: float = OPEN_RATIO) -> float:
    """The ratio mapped linearly onto 0.0 (closed) .. 1.0 (open), saturated."""
    return min(max((ratio - closed) / (open_ - closed), 0.0), 1.0)


class Follower:
    """From one reading per frame to the targets worth sending.

    update() takes the hand's ratio, or None when no hand is in view, and
    returns a gripper target or None. A lost hand never moves the gripper,
    and neither does a ratio above max_ratio: an open hand reads about
    1.7 at most, so 3 is the landmarker guessing, not a hand. The fully
    closed and fully open ends always go out, even inside the deadband,
    so a closed hand really reaches 0.0.
    """

    def __init__(self, closed: float = CLOSED_RATIO, open_: float = OPEN_RATIO,
                 smoothing: float = SMOOTHING, deadband: float = DEADBAND,
                 max_ratio: float = MAX_RATIO) -> None:
        if not open_ > closed:
            raise ValueError(f'--open ({open_}) must be above --closed ({closed})')
        self.closed, self.open_ = closed, open_
        self.smoothing, self.deadband = smoothing, deadband
        self.max_ratio = max_ratio
        self.level: float | None = None    # smoothed openness
        self.sent: float | None = None     # last target that went out

    def update(self, ratio: float | None) -> float | None:
        if ratio is None or ratio > self.max_ratio:
            return None
        raw = openness(ratio, self.closed, self.open_)
        self.level = raw if self.level is None else (
            self.smoothing * raw + (1.0 - self.smoothing) * self.level)
        target = round(self.level, 2)
        if self.sent is not None:
            moved = abs(target - self.sent)
            at_end = target in (0.0, 1.0) and target != self.sent
            if moved < self.deadband and not at_end:
                return None
        self.sent = target
        return target


def _fmt(value: float | None) -> str:
    return '-' if value is None else f'{value:.2f}'


def _bar(level: float | None, width: int = 20) -> str:
    filled = 0 if level is None else round(level * width)
    return '[' + '#' * filled + '.' * (width - filled) + ']'


def status_text(ratio: float | None, level: float | None,
                max_ratio: float = MAX_RATIO) -> str:
    """One status line: what the hand read and where the gripper would go.

    `level` is None until a first usable reading, which a misread is not.
    """
    if ratio is None:
        return 'no hand'
    if ratio > max_ratio:
        return f'ratio {ratio:4.2f}  misread, skipped  gripper {_fmt(level)}'
    return f'ratio {ratio:4.2f}  gripper {_fmt(level)}  {_bar(level)}'


def parse_target(line: str) -> float | None:
    """One line from the pipe to a gripper target in 0..1, or None to skip it."""
    try:
        value = float(line.strip())
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return min(max(value, 0.0), 1.0)


def joint_for(arm: str) -> str:
    """The one joint the bridge may command: that arm's driven finger."""
    if arm not in ARMS:
        raise ValueError(f'arm must be one of {ARMS}, not {arm!r}')
    return f'{arm}_arm_finger_l_joint'


def bridge(arm: str, wait_s: float = 5.0) -> int:
    """Inside the driver's container: targets from stdin to /soma/command."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    joint = joint_for(arm)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('soma_hand_gripper')
    pub = node.create_publisher(JointState, 'soma/command', 10)
    try:
        deadline = time.monotonic() + wait_s
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print('REFUSED: nobody listens on /soma/command. Is the driver up?',
                  file=sys.stderr)
            return 2
        print(f'bridge up: stdin -> {joint}', file=sys.stderr, flush=True)
        for line in sys.stdin:
            value = parse_target(line)
            if value is None:
                continue
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name, msg.position = [joint], [value]
            pub.publish(msg)
            print(f'{joint} -> {value:.2f}', file=sys.stderr, flush=True)
        print('bridge: end of input, the gripper holds its last target',
              file=sys.stderr)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def track(args) -> int:
    """On the Jetson host: the hand on the camera to targets on stdout."""
    import depthai as dai
    from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork

    side, fps, pad = 768, args.fps, 0.1

    class CropConfigs(dai.node.HostNode):
        """Palm detections to one crop per hand for the landmarker, as in
        the oak-examples hand-pose example."""

        def __init__(self):
            super().__init__()
            self.configs = self.createOutput()

        def build(self, detections, size):
            self.size = size
            self.link_args(detections)
            return self

        def process(self, dets: dai.Buffer) -> None:
            group = dai.MessageGroup()
            for i, det in enumerate(dets.detections):
                box = det.getBoundingBox()
                rect = dai.RotatedRect()
                rect.center.x, rect.center.y = box.center.x, box.center.y
                rect.size.width = box.size.width + 2 * pad
                rect.size.height = box.size.height + 2 * pad
                rect.angle = 0
                cfg = dai.ImageManipConfig()
                cfg.addCropRotatedRect(rect, normalizedCoords=True)
                cfg.setOutputSize(*self.size, dai.ImageManipConfig.ResizeMode.STRETCH)
                cfg.setReusePreviousImage(False)
                cfg.setTimestamp(dets.getTimestamp())
                cfg.setSequenceNum(dets.getSequenceNum())
                group[f'cfg_{i}'] = cfg
            group.setTimestamp(dets.getTimestamp())
            group.setSequenceNum(dets.getSequenceNum())
            self.configs.send(group)

    def archive(model):
        desc = dai.NNModelDescription(model=model, platform='RVC2')
        return dai.NNArchive(dai.getModelFromZoo(desc))

    follower = Follower(args.closed, args.open)
    out = sys.stderr if args.emit else sys.stdout
    with dai.Pipeline() as pipeline:
        palm = archive('luxonis/mediapipe-palm-detection:192x192')
        marks = archive('luxonis/mediapipe-hand-landmarker:224x224')
        cam = pipeline.create(dai.node.Camera).build()
        if not args.no_rotate:   # the camera hangs upside down (2026-10-12)
            cam.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
        video = cam.requestOutput((side, side), dai.ImgFrame.Type.BGR888p, fps=fps)
        resize = pipeline.create(dai.node.ImageManip)
        resize.setMaxOutputFrameSize(palm.getInputWidth() * palm.getInputHeight() * 3)
        resize.initialConfig.setOutputSize(palm.getInputWidth(), palm.getInputHeight(),
                                           mode=dai.ImageManipConfig.ResizeMode.STRETCH)
        resize.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        video.link(resize.inputImage)
        det_nn = pipeline.create(ParsingNeuralNetwork).build(resize.out, palm)
        crops = pipeline.create(CropConfigs).build(
            det_nn.out, (marks.getInputWidth(), marks.getInputHeight()))
        cropper = pipeline.create(FrameCropper).fromManipConfigs(
            inputManipConfigs=crops.configs,
            maxOutputFrameSize=marks.getInputWidth() * marks.getInputHeight() * 3,
            waitForConfig=True).build(det_nn.passthrough)
        pose_nn = pipeline.create(ParsingNeuralNetwork).build(cropper.out, marks)
        gather = pipeline.create(GatherData).build(
            cameraFps=fps, inputData=pose_nn.outputs, inputReference=det_nn.out)
        q_hands = gather.out.createOutputQueue(maxSize=4, blocking=False)
        want_video = args.show or args.save
        q_video = video.createOutputQueue(maxSize=2, blocking=False) if want_video else None
        publish = _web_view(args.port) if args.show else None
        pipeline.start()

        say = _Reporter(out)
        log = open(args.log, 'w', buffering=1) if args.log else None
        say(f'tracking at {fps} fps; closed {args.closed:.2f}, open {args.open:.2f} '
            'palm sizes' + (f'; view on http://localhost:{args.port}' if args.show else '')
            + '; Ctrl-C quits')
        lock = HandLock()
        frames, started = 0, time.monotonic()
        last_print = last_save = 0.0
        hands, kpts, img = [], None, None
        try:
            while pipeline.isRunning():
                if q_video is not None:
                    msg = q_video.tryGet()
                    if msg is not None:
                        img = msg.getCvFrame()
                data = q_hands.tryGet()
                if data is None:
                    time.sleep(0.005)
                    continue
                frames += 1
                now = time.monotonic()
                hands = _hands(data, pad)
                kpts, ratio, target = frame_step(hands, lock, follower, now,
                                                 args.min_confidence)
                if args.emit and target is not None:
                    print(f'{target:.2f}', flush=True)
                palms = ' '.join(
                    f'{c:.2f}/{_fmt(hand_ratio(k))}@{k[WRIST][0]:.2f},{k[WRIST][1]:.2f}'
                    f'/L{math.dist(k[WRIST], k[MIDDLE_MCP]):.3f}'
                    f'W{math.dist(k[INDEX_MCP], k[PINKY_MCP]):.3f}' for c, k in hands)
                line = (f'{status_text(ratio, follower.level, follower.max_ratio)}'
                        f'  | palms {palms or "-"}')
                if log is not None:
                    log.write(f'{now - started:7.2f} {line}\n')
                if args.verbose or now - last_print >= 0.5:
                    rate = frames / max(now - started, 1e-6)
                    say(f'{rate:4.1f} fps  {line}')
                    last_print = now
                if args.save and img is not None and now - last_save >= 2.0:
                    import cv2
                    path = f'{args.save}/hand_{frames:05d}.jpg'
                    cv2.imwrite(path, _draw(img, hands, kpts, follower.level))
                    say(f'saved {path}')
                    last_save = now
                if publish is not None and img is not None:
                    publish(_draw(img, hands, kpts, follower.level))
        except BrokenPipeError:   # the bridge ended: stop quietly
            return 0
        except KeyboardInterrupt:
            return 130
        finally:
            if log is not None:
                log.close()
    return 0


class _Reporter:
    """Status lines through a thread that drops them rather than wait.

    A terminal that stops reading (a hidden panel, an SSH channel that
    stalls) fills the pipe, and a blocking print would then freeze the
    camera loop and the view with it, as it did on 2026-10-12. The loop
    only queues; when the queue is full the line is dropped. Targets for
    the bridge never go through here: they must not be dropped.
    """

    def __init__(self, stream, depth: int = 64) -> None:
        import queue
        import threading
        self._queue = queue.Queue(maxsize=depth)
        self._stream = stream
        threading.Thread(target=self._drain, daemon=True).start()

    def __call__(self, line: str) -> None:
        import queue
        try:
            self._queue.put_nowait(line)
        except queue.Full:
            pass

    def _drain(self) -> None:
        while True:
            line = self._queue.get()
            try:
                print(line, file=self._stream, flush=True)
            except (BrokenPipeError, ValueError):
                return


LOCK_RADIUS = 0.2     # how far the wrist may jump between frames, frame widths
LOCK_HOLD_S = 1.0     # how long a lost hand keeps the lock


class HandLock:
    """Follow the same hand while more than one is in view.

    choose() takes (confidence, landmarks) per hand, already filtered by
    confidence, and returns the landmarks to follow or None. Locked on a
    hand, it takes the hand whose wrist is nearest to where the locked
    wrist was, within LOCK_RADIUS. If the locked hand is missing it
    returns None (the gripper holds) and keeps the lock for LOCK_HOLD_S,
    so the other hand cannot take over by a blink of the detector; after
    that it locks onto the most confident hand in view.
    """

    def __init__(self, radius: float = LOCK_RADIUS, hold_s: float = LOCK_HOLD_S) -> None:
        self.radius, self.hold_s = radius, hold_s
        self.wrist = None
        self.lost_at: float | None = None

    def choose(self, hands, now: float):
        if self.wrist is not None:
            near = [(math.dist(k[WRIST], self.wrist), k) for _, k in hands]
            near = [nk for nk in near if nk[0] <= self.radius]
            if near:
                kpts = min(near, key=lambda nk: nk[0])[1]
                self.wrist, self.lost_at = kpts[WRIST], None
                return kpts
            if self.lost_at is None:
                self.lost_at = now
            if now - self.lost_at < self.hold_s:
                return None
            self.wrist = self.lost_at = None
        if not hands:
            return None
        kpts = max(hands, key=lambda h: h[0])[1]
        self.wrist = kpts[WRIST]
        return kpts


def frame_step(hands, lock: HandLock, follower: Follower, now: float,
               min_confidence: float):
    """One frame: (landmarks followed, their ratio, target to send or None)."""
    kpts = lock.choose([h for h in hands if h[0] >= min_confidence], now)
    ratio = hand_ratio(kpts) if kpts else None
    return kpts, ratio, follower.update(ratio)


def _hands(data, pad: float):
    """(confidence, landmarks) for every palm, landmarks in frame coords."""
    out = []
    for i, det in enumerate(data.reference_data.detections):
        conf = data.items[i]['1'].getFirstPrediction()
        box = det.getBoundingBox()
        x0 = box.center.x - box.size.width / 2 - pad
        y0 = box.center.y - box.size.height / 2 - pad
        w, h = box.size.width + 2 * pad, box.size.height + 2 * pad
        out.append((conf, [(x0 + w * kp.imageCoordinates.x, y0 + h * kp.imageCoordinates.y)
                           for kp in data.items[i]['0'].getKeypoints()]))
    return out




def _draw(img, hands, chosen, level):
    """The frame with every palm on it: the followed hand green, others red."""
    import cv2
    img = img.copy()
    h, w = img.shape[:2]
    for conf, kpts in hands:
        color = (0, 255, 0) if kpts is chosen else (0, 0, 255)
        for x, y in kpts:
            cv2.circle(img, (round(x * w), round(y * h)), 4, color, -1)
        x, y = kpts[WRIST]
        cv2.putText(img, f'{conf:.2f}', (round(x * w), round(y * h) + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(img, 'no hand' if chosen is None else f'gripper {_fmt(level)}',
                (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return img


def _web_view(port: int):
    """Serve the annotated frames as an MJPEG stream; returns publish(img).

    The depthai 3 venv carries headless OpenCV (no windows), so the view
    goes to a browser, as the Luxonis examples do on 8082. It listens on
    localhost only: the camera looks at the operator, so the stream is not
    put on the LAN. From the Mac, tunnel it:
    `ssh -N -L 8090:localhost:8090 jetson`, then open http://localhost:8090.
    """
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    import cv2
    latest = {'jpg': None}
    fresh = threading.Condition()

    page = (b'<!doctype html><meta charset="utf-8"><title>SOMA hand</title>'
            b'<body style="margin:0;background:#111">'
            b'<img src="/stream" style="display:block;margin:auto;max-width:100%;'
            b'max-height:100vh">')

    class Stream(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return
            if self.path != '/stream':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with fresh:
                        fresh.wait(timeout=1.0)
                        jpg = latest['jpg']
                    if jpg is not None:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' + str(len(jpg)).encode()
                                         + b'\r\n\r\n' + jpg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Stream)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def publish(img) -> None:
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with fresh:
                latest['jpg'] = buf.tobytes()
                fresh.notify_all()
    return publish


def parse_cli(argv: list[str]):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bridge', choices=ARMS, metavar='ARM',
                    help='inside the container: forward stdin targets to that arm')
    ap.add_argument('--emit', action='store_true',
                    help='print gripper targets on stdout, for the bridge')
    ap.add_argument('--show', action='store_true',
                    help='serve the frames with the hands drawn to a browser')
    ap.add_argument('--port', type=int, default=8090, help='port of --show (default 8090)')
    ap.add_argument('--closed', type=float, default=CLOSED_RATIO,
                    help='ratio read as a closed hand (default %(default)s)')
    ap.add_argument('--open', type=float, default=OPEN_RATIO,
                    help='ratio read as an open hand (default %(default)s)')
    ap.add_argument('--fps', type=int, default=8, help='camera fps (default 8, RVC2)')
    ap.add_argument('--min-confidence', type=float, default=0.5)
    ap.add_argument('--no-rotate', action='store_true', help='frames as they come')
    ap.add_argument('--verbose', action='store_true', help='one status line per frame')
    ap.add_argument('--save', metavar='DIR', help='every 2 s, save a frame with the hands drawn')
    ap.add_argument('--log', metavar='PATH', help='every frame, one line to this file')
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_cli(sys.argv[1:] if argv is None else argv)
    if args.bridge:
        return bridge(args.bridge)
    if not args.open > args.closed:
        print(f'--open ({args.open}) must be above --closed ({args.closed})',
              file=sys.stderr)
        return 2
    return track(args)


if __name__ == '__main__':
    raise SystemExit(main())
