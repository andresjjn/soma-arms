# Branch `v0.3-kinematics`: step-by-step engineering log and technical decisions

Branch derived from `main` on `2026-11-27` to prepare all software,
safety invariants, and simulation checks needed for **v0.3** (pixel-to-deck
homography and planar IK) and **v0.4** (scripted tending cycle) before
running hardware sessions on the Jetson bench.

---

## Step 1: Promote planar 4R kinematics into `soma_driver`

**Files:** `soma_driver/soma_driver/kinematics.py`, `scripts/workspace_map.py`,
`soma_driver/test/test_kinematics.py`

### What changed
- Extracted `load_dimensions`, `link_lengths`, `lane_y`, `limits`, `wrap`,
  `fk`, `ik`, and `solve` from `scripts/workspace_map.py` into a pure
  Python module (`soma_driver.kinematics`) with no ROS dependencies.
- Added `solve_pose()`, which returns a complete 6-joint target dictionary
  for either `'left'` or `'right'` arm (`yaw`, `shoulder`, `elbow`,
  `wrist_pitch`, `wrist_roll`, `finger_l`) ready for `/soma/command`.
- Updated `VERIFIED` in `kinematics.py` so all four planar joints reflect
  the `2026-10-12` sign-check pass (resolving the stale `UNVERIFIED`
  output noted in `docs/next_session.md`).
- Refactored `scripts/workspace_map.py` to import directly from
  `soma_driver.kinematics` so there is one source of truth for FK and IK.

### Technical decision and trade-offs
- **Why promote into `soma_driver` instead of importing from `scripts/`:**
  `scripts/` is not installed as a ROS 2 Python package by `colcon`. Moving
  kinematics into `soma_driver` lets CLIs (`soma_touch_test`), future
  nodes, and `pytest` import it cleanly, while `default_dimensions_path()`
  resolves `dimensions.yaml` from both the source tree and an installed
  `ament_index_python` share directory.
- **Alternative cost:** Keeping kinematics only in `scripts/` would have
  forced duplicate math or fragile `sys.path` hacks in ROS nodes.

---

## Step 2: Safety engineering guards (torque, clearance, and 50 Hz ramp sweep)

**Files:** `soma_driver/soma_driver/kinematics.py`, `soma_driver/soma_driver/touch_test.py`,
`soma_driver/test/test_kinematics.py`, `soma_driver/test/test_touch_test.py`

### What changed
- Added `estimate_base_torque_kgcm()` using the `0.7 kg` arm mass estimate
  distributed across `S->E`, `E->W`, and `W->T` (calibrated so a straight
  horizontal arm yields `~11.4 kg.cm`, matching `primitives.py`).
- Added `pose_safety_fault()` enforcing:
  1. Finite joint angles (`NaN`/`Inf` rejected).
  2. `SERVO_MAP` soft limits.
  3. Elbow curl floor (`ELBOW_CURL_FLOOR_RAD = -1.30 rad`, from the
     `2026-10-26` `elbow_reach` body-clearance measurement).
  4. Plate clearance (`MIN_CLEARANCE_Z_M = 0.005 m`) across all link
     endpoints (`shoulder`, `elbow`, `wrist`, `fingertip`), not just the
     fingertip.
  5. Base static torque limit (`MAX_BASE_TORQUE_KGCM = 9.5 kg.cm`, leaving
     margin below the MG996R `~10 kg.cm` stall).
- Added `check_trajectory_safety()` to step `JointMotion` tick-by-tick at
  `50 Hz` (`ArmController.RATE_HZ`) between start and end poses and run
  `pose_safety_fault()` at every `20 ms` tick.
- Enforced these guards inside `solve_pose()` and `touch_test.plan_trials()`.

### Technical decision and trade-offs
- **Why simulate every 50 Hz tick instead of checking endpoints only:**
  Each joint runs an independent minimum-jerk ramp in `JointMotion`. Two
  individually safe endpoints can cross an unsafe intermediate pose when
  joints travel different angular distances (for example, `(-2.0, 0.0, -0.5, 0.0)`
  to `home` unfolds the elbow before the base clears the horizontal, spiking
  base torque to `9.53 kg.cm` at tick 13).
- **Why keep `solve()` unconstrained while `solve_pose()` enforces safety:**
  `workspace_map.py` uses `solve()` to map raw geometric reach across the
  plane; `solve_pose()` is the command-generating surface and must never
  return an unsafe pose.

---

## Step 3: Bench touch-test CLI (`soma_touch_test`) and ROS adapter tests

**Files:** `soma_driver/soma_driver/touch_test.py`, `soma_driver/soma_driver/touch_test_cli.py`,
`soma_driver/setup.py`, `soma_driver/test/test_touch_test.py`,
`soma_driver/test/test_touch_test_cli.py`

### What changed
- Built `soma_touch_test` (`touch_test.py` pure core + `touch_test_cli.py`
  thin `rclpy` adapter) to step through `(x_mm, z_mm)` lane targets one
  `ENTER` at a time, log `(requested, reached, dx, dz, error_mm)` to a
  timestamped CSV, and report mean error, max error, and repeatability spread.
- Added safety checks: refuses `--dwell <= 0`, `NaN`/`Inf`, `z_mm < 10.0`
  (plate safety floor), and any sequence whose forward transitions or
  emergency unwind paths fail `check_trajectory_safety()`.
- Recorded each target pose in `walked` before calling `publish()` so a
  `KeyboardInterrupt` inside `publish()` still unwinds to `home`.
- Added `test_touch_test_cli.py` using `FakeROS` (same pattern as
  `test_sign_check_cli.py`) plus real-`rclpy` CI checks.

### Technical decision and trade-offs
- **Why pre-filter `--points` before `argparse`:** Standard `argparse`
  rejects negative coordinates like `-30,30` as unknown flags when `nargs='+'`
  is used. Pre-collecting tokens after `--points` up to the next known flag
  lets operators type natural coordinates (`--points -30,30 0,30 30,30`)
  at the bench without `--points=-30,30` workarounds.

---

## Step 4: Pixel-to-deck homography and lane-to-IK bridge (v0.3)

**Files:** `soma_driver/soma_driver/homography.py`, `soma_driver/test/test_homography.py`

### What changed
- Implemented normalized 2D DLT (`fit_homography` with Hartley isotropic
  normalization) in pure Python + `numpy` (no OpenCV dependency) mapping
  `(u, v)` pixels to deck `(x_mm, y_mm)` coordinates.
- Added strict geometric validation: rejects `< 4` correspondences,
  non-finite coordinates, collinear triples, ill-conditioned SVD matrices
  (checking rank 8 for minimal 4-point sets and singular-value ratio for
  overdetermined sets), and projective horizon singularities (`w <= 0`).
- Added `assign_lane()` to map `y_mm` to `'left'` (`+81 mm`) or `'right'`
  (`-81 mm`) and reject points in the dead zone between lanes
  (`|y_mm - lane_y_mm| > 25 mm`).
- Added `point_to_pose()` and `er2_point_to_pose()` connecting pixel clicks
  and Gemini ER 2 `[y, x]` (`0..1000`) coordinates directly to `solve_pose()`.
- Validated against the real October bench pixel marks in
  `calibration/camera_marks_2026-10-26.json`.

### Technical decision and trade-offs
- **Why normalized DLT in `numpy` instead of `cv2.findHomography`:** Keeps
  `soma_driver` free of OpenCV system dependencies in headless CI and on
  laptops while remaining exact and fast for 4+ deck markers.

---

## Step 5: Pick-and-place primitives and 50 Hz tending verification (v0.4)

**Files:** `soma_driver/soma_driver/tending.py`, `soma_driver/test/test_tending.py`

### What changed
- Implemented `solve_with_tilt()` (searching `0, +-5, +-10, +-15 deg`),
  `plan_pick_and_place()` (8-step sequence: `approach`, `descend_pick`,
  `grasp`, `lift`, `move`, `descend_place`, `release`, `retreat`),
  `plan_tending_cycle()` (`home -> load -> unload -> home`), and
  `interleave_cycles()` (sequential left-then-right execution).
- Verified every transition tick-by-tick at `50 Hz` via
  `check_trajectory_safety()`.

### Technical decision and trade-offs (critical simulation finding)
- **Why `DEFAULT_PICK_Z_MM = 40.0` and `DEFAULT_LIFT_Z_MM = 60.0` instead of
  `50/70 mm` or `60/90 mm`:** 50 Hz simulation revealed two hardware limits:
  1. Above `75 mm` a central hole opens under the disc axis.
  2. On the left lane at `-70 mm`, lifting from `50 mm` to `70 mm` caused
     an elbow branch flip (`+0.711` to `-1.237 rad`) whose mid-ramp
     straightening spiked base torque to `9.73 kg.cm`.
  At `40 mm` pick (`20 mm` deck + `20 mm` grasp) and `60 mm` lift, both
  `left` (`-70, 0, +70 mm`) and `right` (`-130, -60, +10 mm`, using tool
  tilt up to `-15 deg` at `+10 mm`) pass all 50 Hz ticks with zero branch
  flips and torque `<= 9.5 kg.cm`.

---

## Step 6: Gate silence while awake in `voice_relay.py`

**Files:** `firmware/eyes/tools/voice_relay.py`, `soma_driver/test/test_voice_relay.py`

### What changed
- Reused `VoiceGate` inside `Relay.uplink()` so quiet room frames are not
  streamed to Gemini Live while awake, sending `audio_stream_end=True`
  when `gate.ended` fires (after `1.0 s` of silence).
- Added `VoiceGate.reset()` so when SOMA starts speaking (half-duplex),
  any open gate closes cleanly and emits `audio_stream_end=True` once
  without leaking buffered frames.

### Technical decision and trade-offs
- **Why gate silence while awake:** As logged on `2026-11-22`, the Live API
  re-bills the full session context every turn (~25 tokens/s of continuous
  mic audio, ~59,000 tokens in 4 minutes). Reusing `VoiceGate` (which keeps
  `0.5 s` preroll and triggers `audio_stream_end=True` at turn end) cuts
  silence token burn while preserving first-syllable capture and turn
  responsiveness.
- **Alternative cost:** Leaving continuous mic streaming wastes tokens in
  quiet intervals; hardware VAD on the ESP32 would require reflashing and
  would not share the host half-duplex `SpeakerQueue` clock.
