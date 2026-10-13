#!/usr/bin/env python3
"""Live view of the OAK-D Lite, for aiming the camera and for filming.

    python3 scripts/oak_view.py              # RGB, turned right side up
    python3 scripts/oak_view.py --depth      # RGB and depth side by side
    python3 scripts/oak_view.py --snapshot /tmp/soma_cam/frame.jpg   # no window

In the window: `s` saves the current RGB frame (full resolution, right side
up) to /tmp/soma_cam/frame.jpg, the file scripts/er2_probe.py reads; `q`
or Esc quits. Only one process can hold the camera, so while this window
is open, frames for the probe come from `s`.

Runs on the Jetson with its system Python (depthai 2.32 and OpenCV, outside
the driver's container), in a desktop session for the window. It only
reads the camera: it cannot reach the driver or a servo.

The camera is mounted upside down (found 2026-10-12), so every frame is
turned 180 deg in software; --no-rotate shows it as it comes. Depth uses
extended disparity, because the work area can sit near the stereo pair's
minimum range (about 34 cm, 17 cm with extended disparity).
"""
import argparse
import os
import time

import cv2
import depthai as dai

SAVE_PATH = '/tmp/soma_cam/frame.jpg'
VIEW_HEIGHT = 540


def build(depth: bool) -> dai.Pipeline:
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setFps(15)
    out = p.create(dai.node.XLinkOut)
    out.setStreamName('rgb')
    cam.isp.link(out.input)
    if depth:
        left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
        for mono, socket in ((left, dai.CameraBoardSocket.CAM_B),
                             (right, dai.CameraBoardSocket.CAM_C)):
            mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
            mono.setBoardSocket(socket)
            mono.setFps(15)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        dout = p.create(dai.node.XLinkOut)
        dout.setStreamName('disparity')
        stereo.disparity.link(dout.input)
    return p


def colorize(disparity, max_disp: float):
    scaled = (disparity * (255.0 / max_disp)).astype('uint8')
    return cv2.applyColorMap(scaled, cv2.COLORMAP_JET)


def fit(img, height: int):
    h, w = img.shape[:2]
    return cv2.resize(img, (round(w * height / h), height))


def save(frame, path: str) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f'saved {path} {frame.shape[1]}x{frame.shape[0]}', flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--depth', action='store_true', help='show depth beside RGB')
    ap.add_argument('--no-rotate', action='store_true', help='frames as they come')
    ap.add_argument('--snapshot', metavar='PATH',
                    help='no window: save one settled RGB frame (and depth, with --depth) and exit')
    args = ap.parse_args()
    turn = (lambda img: img) if args.no_rotate else (lambda img: cv2.rotate(img, cv2.ROTATE_180))

    with dai.Device(build(args.depth)) as dev:
        print(f'OAK {dev.getMxId()} on USB {dev.getUsbSpeed().name}', flush=True)
        q_rgb = dev.getOutputQueue('rgb', maxSize=2, blocking=False)
        q_disp = dev.getOutputQueue('disparity', maxSize=2, blocking=False) if args.depth else None
        rgb = disp = None
        started, frames, fps = time.time(), 0, 0.0
        while True:
            m = q_rgb.tryGet()
            if m is not None:
                rgb, frames = turn(m.getCvFrame()), frames + 1
            if q_disp is not None:
                d = q_disp.tryGet()
                if d is not None:
                    disp = turn(colorize(d.getFrame(), 190.0))
            if args.snapshot:
                if rgb is not None and time.time() - started > 2.5:   # exposure settled
                    save(rgb, args.snapshot)
                    if disp is not None:
                        save(disp, os.path.splitext(args.snapshot)[0] + '_depth.jpg')
                    return 0
                time.sleep(0.02)
                continue
            if rgb is None:
                time.sleep(0.01)
                continue
            elapsed = time.time() - started
            if elapsed > 1.0:
                fps, frames, started = frames / elapsed, 0, time.time()
            view = fit(rgb, VIEW_HEIGHT)
            if disp is not None:
                view = cv2.hconcat([view, fit(disp, VIEW_HEIGHT)])
            cv2.putText(view, f'OAK-D Lite  {fps:4.1f} fps   s: save frame   q: quit',
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow('SOMA eyes (OAK-D Lite)', view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save(rgb, SAVE_PATH)
            elif key in (ord('q'), 27):
                return 0


if __name__ == '__main__':
    raise SystemExit(main())
