# Next session: where 2026-10-12 left off

Handoff written at the end of 2026-10-12, before a context compaction.
The record of the day is `docs/log.md` (2026-10-12, both entries); this
file is the short list to pick up from. Delete it when its items are done
and logged.

## State

- `main` at fe67031, CI green (unit job 431+ passed; the ROS job's colcon
  step now really runs the suite, 448 tests ran).
- Sign check: 12 of 12 `ok` on the metal (reports in `calibration/`).
  The model was fixed against the metal (wrist roll and finger axes,
  pinned by `test_model_directions.py`). For the v0.2 tag only the wave
  on video is missing.
- Jetson: `jetson.local` (DHCP, 192.168.1.11 today), repository at
  `~/Projects/soma/soma-arms`, on fe67031. The driver container was
  recreated with **gate 1 OPEN** (`scripts/jetson_driver.sh open`) and is
  DISARMED. Close it at the end of any session:
  `bash scripts/jetson_driver.sh closed`.
- Camera: the OAK-D Lite is mounted on the column but looks forward at the
  operator, a few centimeters from where the raised gripper reaches,
  upside down. It must move up and out of the arms' reach, looking down,
  before the wave or any arm motion near it. `scripts/oak_view.py`
  (depthai 2.32, the system Python) turns it right side up.
- OAK examples (not part of SOMA): `~/Documents/Camera/oak-examples` on the
  Jetson, depthai 3.11 in the venv `~/venvs/depthai3`; the system Python
  keeps depthai 2.32 for SOMA's own scripts. Local edits there:
  `generic-example/utils/input.py` (camera ROTATE_180_DEG) and
  `spatial-detections/main.py` (numShaves 7 to 4, the OAK-D Lite fix the
  example's own TODO asks for). The v3 examples show results at
  http://192.168.1.11:8082. Detection models are COCO only (no boxes, no
  wooden blocks); YOLO-World runs only on RVC4, not on the OAK-D Lite.

## Pending, in order

1. **ER 2 first call (G0).** API key from https://aistudio.google.com/apikey
   (restricted to the Gemini API), in the environment only. The Mac's
   `.venv` already has google-genai 2.30.0 and Pillow 12.3.0. Then
   `scp jetson:/tmp/soma_cam/frame.jpg /tmp/frame.jpg && .venv/bin/python scripts/er2_probe.py /tmp/frame.jpg --thinking medium --annotate /tmp/seen.jpg`
   (frame saved with `s` in `oak_view.py`). No people in frames sent to
   ER 2. Read the account's real rate limits in AI Studio.
2. **Gripper by hand gesture (proposed, not decided).** One joint, the
   right gripper only: the hand-pose example (MediaPipe palm and hand
   landmarker, RVC2 but slow) gives a thumb-index distance; two states with
   hysteresis; a command only on a change of state, through `docker exec`
   and `ros2 topic pub`; hand not seen = no change. Pure logic with tests
   in this repository. First measure: run `pose-estimation/hand-pose` and
   note fps and tracking. Holding an object stalls the servo (no force
   sensing): soft object, short holds, open on a sustained hum; place the
   object with the gripper open and that hand withdrawn. About 2 to 3
   hours. It also characterizes the gripper's contact, which v0.4 needs.
3. **Arm mimic:** CLAUDE.md keeps it post-1.0. Recommended at most a
   timeboxed "mimic v0" after the v0.2 wave (one arm, yaw and elbow,
   reduced range, a deadman key), and only as an explicit, logged
   decision by Andres. Not decided.
4. **Safety fixes before running `demo`:** the `grippers_open` pose opens
   both grippers at once, and the grippers open sideways: by the FK of the
   model the inner fingertips overlap by about 12 mm at full open (more on
   the metal, which opens 180 deg). Change the catalog and add an FK test
   for the clearance between the two grippers. Also open: `/joint_states`
   can report past a soft limit right after a redirected ramp (a chip from
   the Ctrl-C session); the driver needs a status topic before any agent;
   the IK must keep a fixed 0.1 rad from the captured stops.
5. **Bench:** shoulder threshold (runbook step 8), V+ at a servo connector
   under load (the servos look weak: torque, backlash or sag), then the
   wave in step mode once the camera is out of the forearm's path, filmed:
   tag `v0.2`.
6. **Plan:** apply the changes listed in the 2026-10-12 log entry to
   `docs/plan.md` (ER 2 probe now, ER 2 pointing in the v0.3 acceptance,
   homography as the main mapping, shallow pockets, status topic, rate
   limits and billing before v0.5).
7. **Cleanup, after asking:** the three integrated worktrees in
   `.claude/worktrees/`, the remote branch
   `claude/compassionate-proskuriakova-b5edda`, the stopped container
   `soma_driver_old_8d38831` on the Jetson, and two untracked
   `servo_calibration*.json` at the root of the Jetson's repository
   (compare with `calibration/` first).
