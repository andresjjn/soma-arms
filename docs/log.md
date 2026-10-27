# SOMA bench and decision log

One entry per bench session or per decision, newest at the bottom, dates in
ISO form. Measured facts first, interpretation second, and every number
says where it came from. This file is the live log since 2026-09-01; the
earlier history (first power-up 2026-07-22, caliper session 2026-08-05,
two-board bus 2026-08-11, power plan 2026-08-12) is archived in the Waver
repository (`cad/MEDIDAS.md`, `HANDOFF_SOMA.md`) and is read only.

Incidents also get a row in the table of `docs/safety.md`, paired with the
change they forced.

## 2026-09-01: back on the bench after three weeks

- The 10 A fuse is bought and installed on the ATX branch. The ATX
  (MaxiTech 300U, 10 A on +12 V) becomes the bench supply: 11.6 V stable
  with nothing connected, inside the ATX +-5 % band, below the 11.8 to
  12.3 V we would have liked. Watch it under load; below 11.4 V sustained,
  add a load fan on 5 V.
- Cleaning rule: no WD-40 on electronics (oily residue traps dust and ruins
  rework). Isopropyl alcohol at 90 % or more, soft brush, air, everything
  unpowered and dry before power.
- Assembly finished: molex to barrel for the Jetson (5.5 x 2.5 mm, center
  positive, checked against the NVIDIA spec), P4 to fuse to XT60 to a Y
  splitter to both UBECs, star distribution with separate returns. The
  Jetson ran from the ATX for the first time.
- Incident: hot plugging the servo branch (input capacitors of both UBECs
  plus twelve servos) sank the ATX 12 V rail and hard-shut the Jetson. The
  supply is the common root of the star; a star does not protect against
  the root collapsing. Decision A taken: the Jetson runs on its own wall
  adapter, the ATX feeds servos only.
- Driver live on the Jetson: the `soma_driver` container on `main`, logs
  "MOCK backend, DISARMED". 6.00 V confirmed on both terminal blocks.
- The arming ritual, three steps, all in Andres's hands: (A) relaunch with
  `allow_real:=true`; (B) call `/soma/arm`; (C) `soma_sign_check`,
  interactive, one joint at a time.
- Before anything moves: boards, UBECs and cables were loose on the bench
  (a real risk of a short or a tug while moving); mechanical fixing first,
  safety rule 5 in action.

## 2026-10-01: Jetson network cured, OAK-D Lite verified, harness lesson, shoulders diagnosed

- SSH and xrdp were slow because WiFi power save was on (ping 37 ms mean,
  99 ms max, intermittent loss), not because of power (VDD_IN 4.7 W, 49 C,
  CPU idle). Fix: a scoped sudoers file (`/etc/sudoers.d/soma-claude`,
  NOPASSWD only for nmcli, iw, nvpmodel, jetson_clocks, shutdown), power
  save disabled on the profile and with `iw`: 1.5 ms mean, 3.4 max, 0 %
  loss. The Jetson is at 192.168.1.2 on WiFi (`wlP1p1s0`); a DHCP
  reservation for MAC f0:68:e3:32:b6:8f is still pending. MAXN_SUPER on,
  `jetson_clocks` not applied on purpose.
- OAK-D Lite (MxId 19443010A1A8DE5900) on the Jetson's USB-C port with a
  40 Gbps cable: USB SUPER (5 Gbps), RGB 640x400 at 29.7 fps, chip 33 C,
  depthai 2.32 on the system Python. Depth: baseline 7.5 cm (factory
  calibration), 640x480 at 30.0 fps with LR-check and subpixel, 196 cm
  measured at the center with 0.93 cm temporal noise, 37 % valid pixels on
  a plain wall. Mounting rule: at least 35 cm between the camera and the
  grippers' work area, above and looking down at an angle.
- Lesson of the day: the harness was reported as "channels 1 to 12" and the
  map followed it (commit 6610d13). Result: a dead right arm and a left
  arm receiving the pulses of neighbouring joints. Everything else was
  measured and fine (registers, MODE2, OE at 0 V, +6 V with the right
  polarity, PWM on the header, another supply). The boards have no channel
  silkscreen and the header had been counted from the wrong end. A spare
  servo on every channel (`servo_workbench.py --raw`) proved the August
  layout: 15 down to 10 on `0x40`, 9 down to 4 on `0x43`. Map restored
  (3bccad9). Rule: a channel number is proven by moving a servo, never by
  counting pins.
- Also that day: a 3.3 V logic glitch put both boards back to power-on
  defaults (asleep, 200 Hz) while software believed it was driving pulses;
  the driver and the workbench now detect a reset and disarm (91d295f,
  3ca9ef4, `test_board_reset.py`). Every arm joint re-captured on the
  two-board bench (2bc3883, `calibration/servo_calibration_2026-10-01.json`);
  the right yaw zero was not re-captured and keeps 720 us.
- Workbench renewed: joint list from `SERVO_MAP` on both boards, sliders
  start at the calibrated zero, ALL OFF tolerates a missing board, `--raw`
  mode, sequence player with step mode.
- INA3221 taken off the bus by Andres: nothing worth measuring yet.
- Shoulder oscillation, diagnosed: with the arm hanging vertical (zero) the
  shoulders oscillate fore and aft without end; it stops when another pose
  is commanded or with the lightest touch of a finger. A backlash limit
  cycle in the dead zone of the analog MG996R at about zero gravity
  torque: no preload, the gear play is free inside a lightly damped P loop
  carrying the arm's inertia. Confirmed by Andres: a preloading force
  stabilizes it, so the servo is healthy and it is not a power problem.
  Plan: (1) a soft mechanical preload (band or spring, one direction over
  the whole shoulder travel, only what beats the backlash, every gram costs
  payload); (2) a rest pose with the shoulders off the vertical, sized by
  the "microseconds from zero where it stops" measurement, still pending.
  A digital servo for the shoulders only if (1) is not enough.
- Direction verified on the metal by Andres: on the base (yaw, J1), FORWARD
  is MIN and BACKWARD is MAX. Since every MIN capture sits on the negative
  (hug) side, yaw negative = forward, away from the column. If the same
  rule holds for shoulder, elbow and wrist pitch (not yet checked), the
  shoulder's large +134 deg travel is backward: it cannot raise the arm to
  the front by more than about 25 deg.
- Priority set by Andres: ROS first. The greeting runs through the driver
  (`soma_primitives wave`); the workbench stays a bench tool.

## 2026-10-03: plan restructured around a tending cell, learning track removed

- State at the start of the session: `v0.1` is still the only tag. The 12
  MG996R answer from the workbench one by one, but nothing has moved
  through the ROS driver yet: shoulder, elbow and wrist pitch signs are
  unverified (only the yaw is), no wave, no shoulder preload. The camera
  is connected and verified, not yet fixed in place.
- Decisions by Andres: the arms stay hanging as they are; no parts yet
  (bought or made; a laser cutter for wood is available in town); the
  gripper is a scissor type and opens 180 deg; API billing and the Google
  Developer Program credit will be enabled; planning by objectives, never
  by dates; the goal is one continuous 6 to 7 hour shift doing an
  operator's job; the learning track (RL, MuJoCo, MJX) is removed; the
  greeting comes first; the plan lives in this repository
  (`docs/plan.md`, README); and from today the log lives here too, the
  Waver repository is history only.
- Geometry that rules everything (`scripts/workspace_map.py`, from the
  measured dimensions and limits): each arm is a planar chain and the
  fingertip center never leaves its plane; the planes are at +-62.3 mm,
  124.6 mm apart. Each arm works a lane, no handoff is possible, no
  sorting across lanes. With a vertical gripper and the URDF sign
  hypothesis, the right lane is x in [-69, +71] mm at 20 mm above the
  plate and [-122, +120] at 60 mm; above 70 mm a hole opens under the axis
  and the right arm's front run collapses by 110 mm (its wrist pitch leans
  only 22 deg forward, +0.4869 rad measured; the left one 74 deg). A 15 deg
  tool tilt adds about 40 mm each way. Decision: deck 40 mm above the
  plate, pockets at about -70, 0 and +70 mm per lane (46 mm pockets for
  40 mm parts), tool tilt up to 15 deg; cardboard mock before the laser;
  the deck overhangs the plate's front edge (55 mm, an estimate) and
  stands on the table around it; the rear pocket runs beside the column
  with about 10 mm of clearance.
- The task for v1.0: a two-station tending cell. Each arm serves its lane
  with INPUT, MACHINE (a fixture with a lamp on a free channel of board
  `0x40`) and OUTPUT; the supervisor (Gemini Robotics ER 2) points at the
  part, the arm loads it, the lamp runs, the arm unloads, the supervisor
  verifies and reads the engraved id; the counter increments only on
  verified cycles; roles swap when INPUT is empty so the cell runs for
  hours with nobody at the bench; the two arms work interleaved. Parts:
  wooden blocks of about 40 mm, under 50 g. No conveyor, no rotary table
  (deferred past v1.0), no bimanual handoff (physics, not priority).
  Biggest risk: MG996R thermal and mechanical endurance over seven hours;
  the one hour soak with measured temperatures is a gate before any longer
  run. The full ladder is in `docs/plan.md`.
- Gemini, verified against sources on 2026-10-03: "Gemini Robotics 2" (the
  action model) and "On-Device 2" are for partners and trusted testers
  only. The only callable model is `gemini-robotics-er-2-preview` (2D
  points `[y, x]` in 0 to 1000, boxes, trajectories, planning with function
  calling, success and progress verification on video; 2D output, the XYZ
  comes from the OAK). Price $1 input and $5 output per million tokens
  until 2026-12-31, double from 2027-01-01; ER 1.5 and 1.6 were already
  retired, so the model id is configuration. The AI Pro subscription gives
  no API access; it includes $10 per month of Cloud credit (Developer
  Program premium) to activate on a project with billing. Free tier
  reported at about 20 requests per day (community figure). Latency about
  2.6 s per call (community figure). The key comes from the environment
  only, never from a file in this repository. Terms: not for safety
  critical use, at your own risk, which matches rule 6: the cloud proposes,
  the armed driver disposes.
- Code shipped to `main` (b401c9f, CI green on both jobs, 253 tests
  passing plus 7 skipped without ROS or mujoco): the `wave` sequence
  (poses `wave_probe`, `wave_raise_1`, `wave_raise_2`, `wave_open`,
  `wave_close`; base-raised, elbow sign unverified, probe first);
  `soma_primitives wave --step` (one pose per ENTER, `q` unwinds along the
  walked path, never one jump to home); `scripts/workspace_map.py` with
  `docs/workspace_map.svg`; `scripts/er2_probe.py`; `docs/plan.md`, README
  and CLAUDE.md rewritten to the task-gated ladder.
- Next on the bench, in order: arming ritual and `soma_sign_check` on the
  right arm; `soma_primitives wave --step`, then without `--step`, filmed;
  measure the shoulder oscillation threshold and fit the preload; redo the
  reach map with the verified signs (`--flip`), cardboard deck, then the
  laser; buy six wooden blocks and a lamp; AI Studio auth key with billing
  and `er2_probe.py` on one frame from the Jetson.
- Clarified 2026-10-09: "with the verified signs (`--flip`)" above is
  wrong for the case it was written for. Metal reversed is fixed by
  mirroring the joint's `SERVO_MAP` row, and the map is then rerun
  without `--flip`; RViz reversed with the metal right is an axis flip in
  the model, which the map does not read. See `docs/plan.md` section 4.

## 2026-10-09: the v0.2 bench tools, written at the desk (nothing moved)

- No hardware was touched and nothing moved: a desk session on the Mac,
  executing the plan of `docs/next_session.md` (deleted with this entry,
  its items are done). The Jetson was not contacted.
- `soma_sign_check` rewritten (`sign_check.py`, pure, plus a thin rclpy
  adapter): `--arm right|left|both`, joints in confidence order (yaw
  first, shoulder last), every excursion on the hug side and at least
  0.1 rad from a stop (the right yaw goes to -0.25 now, not to +0.25, 1.4
  deg from its +0.2749 stop), verdicts `ok` / `reversed` / `model` /
  `unclear`. A `reversed` joint gets its mirrored `SERVO_MAP` row, its
  xacro limits and the list of catalog poses that leave the new band (a
  mirrored right elbow breaks `wave` and `demo`); it never gets an axis
  flip. It refuses to start unless the driver listens and every joint it
  will move reads 0.0: a walk that starts off zero can show the right
  motion as the wrong direction.
- Corrections to the handoff, found while doing it:
  - Ctrl-C on Humble: rclpy's own SIGINT handler shuts the context down
    (read in the rclpy humble source, `signal_handler.cpp`), and nothing
    can be published after that. The sign check now starts rclpy with
    `SignalHandlerOptions.NO`, so Ctrl-C mid-hold still sends the joint
    back to 0.0; two tests in the ROS job pin it. `soma_primitives` keeps
    the default: Ctrl-C there leaves the arm at its last pose, held by the
    driver, and `q` stays the abort that unwinds.
  - Arming does cause motion, and not under the ramp: the first armed
    tick sends every channel the pulse of the pose the driver believes it
    holds, at the servo's own speed (about 6 rad/s against the 2.5 rad/s
    cap). `TestWhatArmingSends` pins it (396 passed, 5 skipped in the ROS
    job at 3cb1cd4); `docs/safety.md` rule 2 says it.
  - `--flip`: a reversed metal is a driver fix, after which the reach map
    is rerun WITHOUT `--flip`; a model fix never touches the map, which
    does not read the URDF. "RViz reversed, then `--flip`" would have
    drawn a physically wrong deck.
  - The model check needs no metal, so it left the bench session: section
    10 of `docs/session_v02.md` runs it at a desk against the mock driver.
    RViz beside the bench has no validated recipe yet.
  - Gate 1 outlives a reboot: `restart: unless-stopped` keeps a container
    created with `ALLOW_REAL=true`, so every session closes gate 1 with a
    plain relaunch.
  - After an ATX stop the driver is still ARMED (the boards' logic runs on
    the Jetson's 3.3 V, it cannot see the rail drop): disarm before the
    ATX comes back on.
  - Keys typed while a joint moved answered the next prompt (a double
    ENTER could accept a verdict and start the next joint); both CLIs
    discard pending input before every prompt now.
  - The `soma` helper of the handoff spliced `$*` into `bash -c`, which
    splits `"{data: true}"` in two; it passes positional parameters now,
    with a quoting self-test in the runbook.
- Desk analysis for the model check, a prediction until RViz confirms it
  (pure Python FK transcribed from the xacro, signed fingertip separation):
  the four planar joints are drawn negative = forward on both arms, as the
  hug convention wants; a negative wrist roll is drawn clockwise seen from
  above on the right arm and counterclockwise on the left, to compare with
  the CW or CCW noted at the bench; and the finger axis `0 1 0` turns both
  fingertips toward each other for a positive angle (separation +25.7 mm
  at 0.0, -6.8 at 0.25, -84.9 at 1.0), so RViz should draw "open" as
  crossed fingers: a likely `model` finding on both grippers.
- CI: the "Run tests through colcon" step collects 0 tests; the ROS node
  tests actually run inside `scripts/smoke_test.sh`. Noted, not fixed.
- Tests on the Mac (no ROS, no mujoco): 377 passed, 9 skipped.
- Next: the bench session, with `docs/session_v02.md` printed.

## 2026-10-09: colcon test runs the driver suite (CI only, nothing moved)

- Measured: the ROS job's "Run tests through colcon" step printed
  `Summary: 0 tests` and passed (runs 37964849489 at 3cb1cd4 and
  37965586501 at a2fa035). The suite and its ROS node tests ran only in
  `scripts/smoke_test.sh`, through plain pytest: 397 passed, 5 skipped
  at a2fa035.
- Cause, read in the colcon-core 0.21.3 source and reproduced on the
  Mac: colcon runs pytest only when `setup.py` names it, in
  `tests_require` or `extras_require['test']`. `soma_driver` named it
  only as a `<test_depend>`, which is for rosdep, so colcon ran
  `python3 -m unittest`: no TestCase found, no junit XML, and on the
  image's Python 3.10 an "OK" with exit 0. The image's pytest 6.2.5
  played no part; colcon only needs it importable.
- Fix: `extras_require={'test': ['pytest']}` in `soma_driver/setup.py`,
  the form of the Humble `ament_python` template since 2025-09. CI's
  colcon step and smoke step 2, which now runs the same commands instead
  of plain pytest, end with `scripts/check_test_results.py`: it fails
  unless at least one test reached a verdict (tests minus skipped minus
  errors). `ROS_LOCALHOST_ONLY` covers the whole job.
- A count, not a grep: the first guard looked for `tests="[1-9]` in the
  XML and missed the other silent zero. pytest exits 5 when it collects
  nothing, colcon passes that, and the XML of a collapsed collection
  says `tests="1"`, one skip. The collapse happened here: on 2026-08-11
  a module level `importorskip('mujoco')` cut the suite to "1 skipped"
  in the container (run 31451116966, fixed in fd6bec3). The trigger is
  `launch_testing`'s pytest hook (Humble source): it imports every test
  module itself and catches only SyntaxError, so the skip escapes and
  takes the whole collection with it. Reproduced with pytest 6.2.5 and
  a stand-in for the hook; pytest 6.2.5 alone skips just the one file.
  The comments in `test_sim_oracle.py` and the two CLI tests have the
  effect right but blame the container's old pytest, not the hook. Not
  changed.
- The end to end failed once (run 37976433906, a docs-only commit after
  a green run of the same code): the elbow command was published and TF
  still read z 0.006. Likely cause: `ros2 topic pub --once` exits 0.1 s
  after its only sample, and a driver that has not matched the
  publisher yet drops it (DDS discovery is not symmetric). The smoke
  test now sends the same command three times (`--times 3`), 1 s apart.
- Read off run 37977558879 at df7ac0f: 403 passed, 5 skipped, `Summary:
  408 tests, 0 errors, 0 failures, 5 skipped` and "403 tests ran", in
  the colcon step and again in smoke step 2; the end to end lifted tool0
  311.0 mm. On the Mac: 383 passed, 9 skipped, through colcon as through
  plain pytest.

## 2026-10-09: Ctrl-C walks the arm home in `soma_primitives` too (nothing moved)

- Desk work on the Mac, no hardware touched, the Jetson not contacted.
- `soma_primitives` now starts rclpy with `SignalHandlerOptions.NO`, like
  the sign check, and turns a hang-up or a kill into Ctrl-C. This replaces
  the line of the entry above that kept the rclpy default. `play()` takes
  every way out of a sequence the way `q` takes it: the end of input,
  Ctrl-C at a prompt or in the middle of a move (with or without
  `--step`), and any error, which is raised again once the arm is home. A
  second Ctrl-C, during the walk home, is let through: the arm keeps the
  last pose commanded (manual recovery in `docs/session_v02.md` step 7).
- Found while doing it: the unwind retraced the walked path literally, so
  `q` after the descending `wave_probe`, one move from home, climbed back
  through all three swings (12 moves), and a Ctrl-C during the final
  `home` would have replayed the whole wave backwards. The unwind now
  starts from the first time the arm held its current pose and skips the
  loop that followed: still only moves already made, each at most 1 rad,
  now never the same pose twice (from the descending probe: 1 move).
- A move cut short is undone when it is new ground (the way up, which is
  what step mode probes) and let finish, undisturbed, when it retraces a
  move already made (the swings after the first, the way down). If the
  interrupt lands inside the publish, the command may or may not have gone
  out, so the move is always undone: that asks for the pose the arm is
  already in, or reverses a move that just began.
- Undoing a move in flight is new for the driver's planner, which replans
  from the current position and velocity. Simulated with `trajectory.py`
  at the 50 Hz tick, every step of both sequences cut at every 1% of its
  ramp: the reverse strays at most 0.023 rad outside the span already
  walked on an arm joint (0.024 rad on a finger, past its closed end,
  where the pulse saturates) and settles at least 0.47 s inside the wait
  the unwind sleeps. Simulated, not measured. A test pins the bounds at a
  tenth of a ramp: 0.1 rad and the wait.
- Re-sending the same target mid-ramp is not harmless: the replan
  overshot by up to 0.047 rad in the same simulation, which is why a move
  that is let finish is waited for, not commanded again.
- Tests on the Mac (no ROS, no mujoco): 398 passed, 11 skipped. The two
  new real-rclpy tests (the init call, and a publish that still goes out
  after a real SIGINT) run only in the ROS job.

## 2026-10-12: first joints moved through the ROS driver; the camera; the Jetson as found

- Right-arm sign check on the metal, through the driver: 6 of 6 `ok`
  (yaw, finger, wrist roll, wrist pitch, elbow, shoulder), every
  excursion 0.25 rad toward the hug side and back. The hug convention
  holds on the right arm; shoulder, elbow and wrist pitch move from URDF
  hypothesis to verified. The driver log shows one ARMED and, about six
  minutes later, one DISARMED, with no board fault between them; every
  joint read 0.0 afterwards. Report: `calibration/sign_check_2026-10-12_right.json`.
  Left arm, same session: 6 of 6 `ok`, report
  `calibration/sign_check_2026-10-12_left.json`, again one ARMED and one
  DISARMED with no fault. All 12 joints are sign-checked on the metal;
  for the v0.2 tag only the wave on video is missing. The right arm was
  checked twice: again at 23:55, in the left arm's armed window, while
  Andres watched the roll from above; same six `ok`
  (`calibration/sign_check_2026-10-12_right_rerun.json`). That rerun had
  overwritten the first report on the Jetson (same file name); report
  names now carry the time.
- The model, checked against the metal: seen from above, a negative wrist
  roll turns the gripper inward, counterclockwise on the right arm and
  clockwise on the left (Andres; a first report of "clockwise" on the
  right had no viewpoint). The FK of the URDF drew both the other way,
  so the model was wrong and the metal right: a `model` finding on both
  arms, fixed with the shared `<axis>` line (`${side}` to `${-side}`).
  The same FK showed the fingers drawn closing and crossing for a
  positive (opening) command, while the metal opens: both finger axes go
  from `0 1 0` to `0 -1 0`. `test_model_directions.py` now runs a
  generic FK over the bench URDF and pins every direction: planar joints
  forward for a negative command, rolls inward, fingers opening. On the
  URDF the Jetson's container produced before the fix, 4 of its 12 cases
  failed (the rolls and the fingers); with the three axis lines flipped,
  12 of 12 pass. The driver, the limits and the reach map are unchanged.
- Andres's impression of the servos at the bench: fragile, with play,
  weak. Not measured yet. Three things look alike and are told apart by
  measurement, not by eye: backlash (MG996R gear play, 1 to 2 deg at the
  output is typical, about 5 mm at the fingertip per degree at the base),
  torque (about 10 kg.cm against roughly 1 N.m for a 0.7 kg arm held
  horizontal, an estimate), and V+ sag under load (6.00 V was measured
  with no load only). Decision by Andres: keep building on the MG996R;
  when they become the limit, evaluate the best servos available with
  real position feedback. That is a new driver backend behind the same
  Pca9685Backend boundary; the mock boot, the two gates and the ramp do
  not change.
- The OAK-D Lite is mounted but looks forward toward the operator, not
  down at the plate. Frames captured on the Jetson (1920x1080 RGB, USB
  SUPER, depthai 2.32): the first shows the desk and Andres, the second
  the right gripper held up by hand a few centimeters from the lens. The
  image is upside down (mount at 180 deg, one line in the pipeline). The
  work area is not in view, the gripper reaches the lens, and that range
  is inside the stereo blind zone (MinZ about 34 cm, 17 cm with extended
  disparity, from the 75 mm baseline and the 73 deg mono HFOV). It has to
  go up and out of the arms' reach, looking down; Andres cannot move it
  this week. A yaw motion Andres confirmed for a field-of-view check was
  not executed: the camera could not see the work area and a hand was on
  the arm.
- The Jetson as found: on DHCP at 192.168.1.11 (use `jetson.local`), the
  repository at `/home/jetson/Projects/soma/soma-arms` (not
  `/home/jetson/soma-arms` as the docs said), no `docker compose`, and
  the driver container created by hand on 2026-10-02 with
  `allow_real:=true`, running on code of 2026-10-03 (8d38831). Gate 1 had
  been open across reboots for ten days (a row in `docs/safety.md`).
  Andres pulled `main` (1e17481) and recreated the container; the old one
  is kept stopped as `soma_driver_old_8d38831`. New:
  `scripts/jetson_driver.sh closed|open`, which refuses while ARMED and
  is tested against a fake docker; the docs use it instead of compose.
- Desk work of the day: the workbench watchdog reads the bus under the
  lock (1e17481); the plan was reviewed against the ER 2 docs read
  again. What changes, for a later edit of `docs/plan.md`: the API key
  and `er2_probe.py` on a real frame start now, in parallel, with no
  motors; ER 2 pointing replaces the click in the v0.3 acceptance; the
  rate limits of ER 2 are not published (read them in AI Studio, billing
  before v0.5); the right lane splits above about 70 mm of fingertip
  height, so pockets stay shallow (10 mm or less) or the deck goes; the
  joint limits are the captured mechanical stops, so the IK keeps a fixed
  0.1 rad margin; the driver needs a status topic before the agent can
  report honest results; the tending cell stays the v1.0 task and the
  one hour soak stays a gate.

## 2026-10-12, evening: the branches merged, the rules, the camera at work

- Integrated, after a review of each: the CI colcon fix (squash; colcon
  now runs the suite, 448 tests ran in the ROS job), the Ctrl-C and `q`
  unwind (re-checked on the merged code: `q` on the way down of the wave
  is 1 move home, it was 12), and the hardware docs (Jetson bench, July
  as history). Inviolable rules 1 and 4 reworded with Andres's approval:
  SSH on the Jetson; arms hanging at rest, never folded by hand before
  arming. CI failed once on a DNS error of the runner (packages.ros.org)
  and passed on rerun.
- The camera put to work: `scripts/oak_view.py` (live view, turned right
  side up, depth beside it, `s` saves a frame) tested on the Jetson;
  `er2_probe.py` gained Google's own open pointing prompt and
  `--annotate`. The ER 2 key is not created yet: no call made.
- Luxonis' depthai 3 examples run on the Jetson in a separate venv. The
  spatial detections example crashed the OAK-D Lite with `E_OUT_OF_MEM`
  in the stereo filter; the example's own TODO says to use 4 SHAVEs
  instead of 7 on the Lite. In v3 the camera is turned with
  `Camera.setImageOrientation(ROTATE_180_DEG)`, so the network sees the
  scene upright. The COCO detector does not know boxes or wooden blocks;
  YOLO-World is RVC4 only.
- Found while choosing a motion for a video: `grippers_open` opens both
  grippers at once, sideways, and by the model's FK the inner fingertips
  overlap by about 12 mm at full open. `demo` must not run until the
  catalog changes (in `docs/next_session.md`).
- Asked and answered, not decided: a full arm mimic is post-1.0 by
  CLAUDE.md (at most a timeboxed v0 after the v0.2 wave, by an explicit
  decision); a gripper opened and closed by a hand pinch gesture is about
  2 to 3 hours, one joint, and would also measure the gripper's contact.
- Gate 1 is open on the Jetson at the end of the day, the driver
  disarmed. Handoff: `docs/next_session.md`.

## 2026-10-12, night: one gripper follows one hand (camera only so far)

- Decided by Andres: a bench experiment, one joint. One gripper opens in
  proportion to how open his hand is, read by the OAK-D Lite. Not the
  arm mimic, which stays post-1.0; not part of the library. Code:
  `scripts/hand_gripper.py`, tests in `test_hand_gripper.py`.
- How it reads the hand: Luxonis' MediaPipe palm detector and hand
  landmarker run on the camera (RVC2, 8.0 fps measured, the depthai 3
  venv). The opening is the thumb tip's distance to the mean of the four
  fingertips over the palm's size. Two processes joined by a pipe: the
  tracker on the host prints targets, a bridge inside the driver's
  container publishes them to `/soma/command` for one finger joint. It
  cannot arm; a lost hand sends nothing; the driver clamps and ramps.
- What the first runs taught, each one now fixed and tested:
  1. Two palms in most frames: the detector saw both of Andres's hands
     (the other one held the phone). Taking the most confident one each
     frame jumped between hands; the tracker now locks onto one hand by
     its wrist's position and keeps the lock through a 1 s blink.
  2. Readings of 2 to 5, even 19, for an open hand: a hand tilted toward
     the lens shows a short palm, and the ratio over it inflates. The
     palm's size is now the longer of its length and its scaled width;
     readings above 2.5 are skipped as misreads.
  3. The landmarker loses confidence on an upside-down hand (the crop is
     never rotated): the pose that works is the palm to the camera,
     fingers up, as if waving at the robot.
  4. The depthai 3 venv has headless OpenCV, so the view is an MJPEG
     page on the Jetson's localhost (8090), tunneled to the Mac. It
     listened on the LAN in its first version; the camera looks at the
     operator, so it is localhost only now.
  5. The view froze after a few minutes: status lines went through `tee`
     and SSH to a terminal that stopped reading, the pipe filled and the
     blocking print stopped the camera loop. Status now goes through a
     thread that drops lines; every frame goes to a log file instead.
- Calibrated on Andres's hand, 270 s: a pinch reads 0.4 to 0.5, an open
  hand 1.5 to 1.7. Defaults now 0.5 for gripper 0.0, 1.5 for 1.0.
- On the metal, Andres's go-ahead for the left gripper only (the right
  one shows some play, to look at later). First a dry run against the
  disarmed driver: the bridge works inside the container, and it left
  the mock finger at 0.63 because the last reading was a half-open hand.
  Arming then would have sent 0.63 to the real gripper with no ramp, so
  the finger was commanded back to 0.0 (still disarmed) and checked on
  `/joint_states` before arming. Armed, the left gripper followed his
  left hand "very well" (his words).
- His one complaint, measured on that run: near closed the gripper
  fluttered, 57 of 216 targets in 107 s sat between 0 and 0.2. The
  filter now takes the median of 3 readings, averages with weight 0.3,
  and holds the ends: at or below 0.2 the gripper shuts and stays shut
  until the hand passes 0.3 (the open end mirrors it at 0.85 and 0.75).
  Replayed on the same run: 131 targets, none between 0 and 0.2.
- Not checked yet: Andres says the two grippers hang about 15 cm apart,
  more than the model's spacing suggests; to be measured with a ruler
  before the `grippers_open` fix is sized.
- Then Andres asked for a sequence for a video: the gripper follows his
  hand; once he opens and closes it, the arm lifts to the front, gripper
  shut, and the wrist rolls a little each way until he ends it. The
  bridge's `--present` does it (designed, tested, not yet run on the
  metal). By the URDF's estimated masses, a level forearm 50 deg forward
  would ask the base for about 9.9 kg.cm, the MG996R's measured limit;
  the chosen pose (base -0.52, elbow -1.65, forearm raised) asks about 7,
  the designed wave's load. The elbow curls first with the arm hanging
  and the base lands first on the way down, so the path never crosses
  the level, loaded forearm (checked through the driver's own planner).
  The show goes home on its own after 30 s; the end of input walks it
  home before the bridge exits; it refuses to start off home.
- Run on the metal by Andres: the left forearm folded in and jammed
  against the central box from the start of the elbow curl (incident row
  in `docs/safety.md`). The model gives that path 45 mm of clearance, so
  the metal and the model disagree at large elbow angles; the elbow's
  real angle per command is to be measured in small steps. The show now
  lifts the straight arm with the base only (`--lift`, 0.52 rad = 30 deg
  by default, at most 0.7): about 5.7 kg.cm on the base by the model,
  45 mm from the body, and the wrist rolls +-0.25 rad. The elbow is
  never commanded.
- The straight lift ran on the metal (Andres: "genial"). Two additions
  on his request. A lost hand releases: once a hand seen before has been
  gone 2 s, the tracker sends `release`; the bridge walks the arm home if
  it was raised, waits 1 s for the ramp to land, and disarms. The script
  can now disarm but still never arm (one request, built with data
  False, pinned by a test). And `--lift` goes up to 45 deg: about
  8.0 kg.cm on the base by the model, 80% of the measured torque, so
  above 40 deg the show lasts at most 15 s.
- Since a lost hand disarms, Andres asked for arming in the same command.
  `hand_gripper.py` still cannot arm; `scripts/hand_show.sh` runs one
  take: it checks that every arm joint of both arms reads 0.0 (the first
  armed tick sends the believed pose with no ramp), prints the rule 4
  checklist, arms only if "arm" is typed, then starts the tracker and
  the bridge. Tested on a fake docker, with a mutation check: a copy
  that arms without asking fails five tests.
- At 45 deg the take worked, but the wrist barely rolled: Andres took his
  hand away right after the trigger, and 2 s later the release walked
  the arm home before the show had begun. The show now does a fixed
  number of full wrist cycles (`--rolls`, 4 of 3 s by default; the time
  caps still apply), then comes home on its own. A release asked during
  the raise or the show waits for it to finish; before the trigger it
  is immediate. The tracker no longer exits on a release: it keeps a
  heartbeat line going, so a Ctrl-C still reaches the bridge as the end
  of input and brings the arm home at once.

## 2026-10-13: servos kept, the camera on its mast, a coverage check

- Servos: a research pass (actuator classes, from programmable PWM to
  Dynamixel and brushless) concluded that inside v1.0 a servo change is
  a thermal decision, not a smoothness one: the shoulder limit cycle is
  backlash inside the servo's P loop where gravity torque crosses zero,
  which a one-sign preload cures and no actuator removes by itself. The
  candidates that fit the brackets with margin cost about USD 460 for J1
  and J2 of both arms. Andres decided to keep the MG996R: preload, rest
  pose off the vertical, a fan and the INA3221 back on the bus first; a
  servo is bought only if the cycle survives the preload or the base
  passes 60 C in the soak.
- Found while planning: every joint's radians come from a NOMINAL
  2000 us = 180 deg (`apply_calibration.py`), never measured. The sign
  check proved directions, not magnitudes; the elbow jam of 2026-10-12
  is the first evidence they differ. Measuring them is v0.2 work.
- The OAK-D Lite is fixed on a mast above the central box, tilted over
  the arms' forward arc, rolled a quarter turn. Depth (extended
  disparity, 60 frames): 76% valid, the robot's metal at 36 to 42 cm, a
  gripper held in front at 52 to 57 cm. Both viewers now turn frames
  90 deg (checked on frames: `addRotateDeg` turns the opposite way to
  OpenCV's naming, -90 there).
- Each arm only moves in its own vertical plane, so pixel to position
  can be one homography per arm plane (four marks each), not only one of
  the deck; the camera only has to see the two planes.
- Coverage check, written and smoke-tested, not run with motion yet:
  `camera_check` (one straight arm at a time, base only, 15, 30, 40 deg,
  down the same way) and `scripts/camera_coverage.py`, which compares
  depth against the empty bench and reports, per pose, whether the arm
  is in frame or cut by an edge. A smoke run showed why nobody may be in
  view: a person moving reads as "closer than the baseline".

## 2026-10-26: the joint probe, written at the desk (nothing moved)

- `soma_joint_probe` measures how far a planar joint really turns per
  radian commanded, since every joint's radians are still the nominal
  2000 us = 180 deg. One joint at a time with the rest at 0.0, so the
  inclination of the link it drives, read from the vertical with a phone
  level, is the joint's angle. It reads 0.0 first (the zero error), then
  walks a staircase forward (0.2 rad stairs by default, up to a ceiling
  per joint: base 0.70, shoulder 0.4, elbow and wrist 1.0, always 0.1
  rad inside the captured stop), and walks back down the same stairs on
  q, x (close to something), Ctrl-C, end of input or any error. A line
  through the readings gives the real scale, the zero error and the
  residual, and from them the real microseconds per radian. Pure module
  with tests; the CLI reuses the sign check's node and home check, and
  never arms. Not run on the metal yet.
- `gemini-bridge` exists outside this repository (its own repo): an MCP
  server that lets Claude delegate to Gemini, including ER 2 pointing for
  SOMA. It is a development tool, read only, with no way to reach the
  robot; SOMA's own agent at v0.5 will call ER 2 through its own adapter.

## 2026-10-26, bench: the camera on a tripod, measured on four lane marks

- The OAK-D Lite came off the mast and onto a tripod behind and above
  the robot, apart from it, looking forward and down over the lanes. A
  mast view had the arms leave the frame and the robot inside the stereo
  blind zone; from the tripod the robot sits at 60 to 68 cm, the desk in
  front at 1.1 to 1.4 m, 73 to 74% valid depth. The tripod may move
  between sessions: the reference view is
  `docs/media/camera/2026-10-26/reference_tripod.jpg`, same height.
- Andres taped four blue marks on the desk: under each closed gripper
  tip, and 250 mm forward of each (center to center). With the camera's
  factory intrinsics and the aligned depth, the marks' 3D positions give
  266.6 mm for both 250 mm runs: depth reads +6.6% at 1.1 to 1.25 m, so
  depth confirms and the homography positions, as the plan says. Along a
  lane, 250 mm is 92 px in the 640x360 frame, about 0.9 mm per pixel at
  full resolution: room for the 5 mm acceptance of v0.3. The view is
  oblique (about 28 deg to the desk); higher and steeper would help.
- Not matching: the two near marks are 197 mm apart in 3D (about 186 mm
  after the depth scale), Andres estimated about 15 cm, and the model
  puts the arm planes 124.6 mm apart. To measure with a ruler before the
  deck or the workspace map are trusted.
- The OAK reported one device crash when a live viewer was cut; the next
  capture worked. Watch it.
- Images and the record: `docs/media/camera/2026-10-26/`,
  `calibration/camera_marks_2026-10-26.json`. From now on useful bench
  images go into `docs/media/`, reviewed for privacy before committing.
- Measured with a ruler, both arms armed and swung straight forward to
  30 deg by the base (Andres's go-ahead, held under 2 minutes, lowered
  in four steps): the closed gripper tips are 180 mm apart. The camera
  had read about 186 mm on the desk marks; the model says 124.6 mm. A
  likely cause, from `dimensions.yaml` itself: the model puts each arm's
  plane on the disc's outer face (54.9 + 7.4 = 62.3 mm from the center),
  but the arm is a 52.8 mm wide sandwich of plates mounted outboard of
  the disc, so its mid-plane sits about 26.4 mm further out: 88.7 mm a
  side, 177.4 mm between arms, close to the 180 measured. To confirm
  with calipers before the URDF, `workspace_map.py` and the gripper
  clearance analysis are corrected.
- From the tripod the raised arms were not in the camera's view (twice,
  the second time with a lamp moved away): either hidden under the box
  or swinging toward the tripod, outside the frame. Which way the arms'
  forward points relative to the camera is to be checked.

- Later the same day Andres fixed a loose elbow, and in doing so moved
  the robot and the tripod. Against the reference image the scene sat
  about 9 % of the frame further right and nearer (median depth 1.0 m
  against 1.1 to 1.25 m), and the robot looked a few centimeters off its
  marks. The lane mark record of the morning no longer applies; the
  marks are still on the desk.
- Before the next arming after the repair: if the elbow horn was seated
  again, its hanging zero may have moved by a spline tooth, and the
  first armed tick sends that joint to its zero with no ramp. Check that
  joint on its own first.
- New `scripts/camera_align.py`, so this does not cost a session again:
  a live view (MJPEG on 8091, localhost) with the reference as a ghost,
  the center line and the robot's box, and two readings in percent of
  the frame. The camera reading is a similarity fit of ORB features of
  the scene under RANSAC, the robot and the tripod's strip left out; the
  robot reading is its reference crop found in the live frame minus the
  camera's own shift. Checked with live and synthetic frames (2 % reads
  2.0 %, 1 deg reads 0.99 deg, two live frames read 0.0 %). Against the
  morning's reference it says "scene not matched": the move was too big
  for features, which is what the ghost is for. A new reference is to be
  saved with the robot centered and the arms hanging.
- The repair, in Andres's words: the right arm's wrist pitch (third servo
  from the gripper up, robot's own right) had play because one horn had
  been mounted reversed; it was turned and seated again. Its hanging
  zero may have moved and is to be read before any motion that needs it.
- New camera reference saved with `camera_align.py --save` at 17:20,
  robot centered on the guide line, arms hanging, nobody in view. A
  reading against it: camera 0.06 % / 0.08 deg, robot 0 mm, 0.0 % off
  the center line, 151 features (37.6 %). A first reference with a
  person beside the robot had matched only 57 features, 14 %: people in
  the scene are noise for the camera reading. The committed copy has the
  laptop screen blurred; screen content changes anyway, so matching
  does not use it.
- Correction: the repaired joint was the right ELBOW (fourth servo from
  the gripper up), not the wrist pitch. Armed with the old map, its
  forearm stood about 44 deg off the hang. Zeros re-captured with
  `servo_workbench.py`, driver container stopped, arms hanging:
  right elbow 935 -> 1425 us (the horn three teeth off), right yaw
  720 -> 750, left yaw 2370 -> 2450, left shoulder 1075 -> 1055; the
  rest hung straight at their old zeros. At 1425 the elbow kept only
  93 deg of forward travel and `wave_close` (-2.09 rad) and
  `wave_raise_2` (-1.83) fell outside it: the tests caught it. So the
  horn was seated again, three teeth back, with the servo held at 945
  us so the direction was not a guess, then fine-tuned to 935 us, its
  October zero. Its forward end stays 2460 us (the 2500 captured is the
  workbench clamp); its backward end takes the 560 us measured that day.
  Record: `calibration/servo_calibration_2026-10-26.json`.
- Andres's reading of the workbench (every joint moved at once): smooth,
  no joint looked forced, the oscillation rare and only at zero, where
  gravity torque crosses zero (the backlash limit cycle). The loose
  elbow likely made it worse. Encouraging; the 10 kg.cm budget still
  holds for extended poses.
- Sign check after the re-seat, gate 1 opened by relaunch, armed by
  Andres: right wrist pitch ok at -0.25 rad (18:27; it was run first by a
  miscount of skips, and it was also touched by the repair), right elbow
  ok at -0.25 rad (18:30): the forearm curled forward, the hug way. The
  re-seated horn kept its sign. Reports in `calibration/`.
- First `soma_joint_probe` runs, elbows, shoulder at 0, phone level on
  the forearm, stairs of 0.2 rad to -0.6:
  - right, 18:33 and 18:37: 0, 14, 23, then 57 deg at -0.6 both times.
    Not the elbow: gravity can only leave a joint short of its command,
    never 23 deg past it, and 90 - 34.4 = 55.6. The level app switched
    its reference past about 45 deg of phone tilt. Those two fits are
    void; the files stay as the record of the trap.
  - right, 18:40 and 18:44: 5, 13/14, 23/22, 32 deg. Scale 0.794 and
    0.777, worst residual 0.7 deg: about 810 us/rad, not the nominal
    636.6. The forearm reads +5 deg at command 0 (Andres: the zero by
    eye is not quite vertical).
  - left, 18:46: 0, 10, 21, 32 deg. Scale 0.934, residual 0.4 deg, about
    682 us/rad.
  - The two elbows differ by 16 % in travel for the same command (27 deg
    against 32 deg from 0 to -0.6): a zero offset shifts readings, it
    cannot shrink travel. The scale belongs to each servo, not to the
    model: SERVO_MAP needs one scale per joint, not one US_PER_RAD.
- The other six planar joints, same procedure (19:00 to 19:07). Some
  runs read negative: the phone was turned, not the joint (every one of
  these joints passed its sign check); magnitudes only below. Fitted
  twice: through every point, and through the loaded points only
  (without the hanging 0.0). The loaded fits are tighter (residuals 0.0
  to 1.0 deg) and run 2 to 8 deg below the origin: with the link hanging,
  gravity torque is zero and the joint rests anywhere in its gear play;
  once loaded, gravity holds it against one side. The hanging point is
  inside the play, so it biases a fit through it, and the loaded lag
  measures the play: largest at the bases (3 to 8 deg, the whole arm on
  them), smallest at the wrists (about 1 deg). This is the play behind
  the oscillation at zero.

  | joint | scale, loaded points | runs |
  |---|---|---|
  | right yaw | 0.83 (0.83, 0.83, 0.92) | 3 |
  | right shoulder | 0.87 | 1, 11.5 deg of span |
  | right elbow | 0.81 (0.83, 0.79) | 2 |
  | right wrist pitch | 0.83 | 1 |
  | left yaw | 0.87 | 1 |
  | left shoulder | 1.05 | 1, 11.5 deg of span |
  | left elbow | 0.96 | 1 |
  | left wrist pitch | 0.92 | 1 |

  A 1 deg phone over 23 deg of span is about +-5 % per scale, +-9 % for
  the shoulders. The right arm reads 0.81 to 0.87 on every joint, the
  left 0.87 to 1.05: the pattern follows the board (right on 0x40, left
  on 0x43) more than the servo. A PCA9685 whose oscillator runs fast
  shortens every pulse on that board by the same factor; prescale 121
  assumes 25 MHz and was never checked against a meter. To measure the
  PWM frequency of each board before any scale goes into SERVO_MAP.
- The right forearm and the right gripper both read +5 deg at command
  0 (the left ones 0): the right elbow's zero holds the forearm about
  5 deg forward, and the gripper reading carries it. A zero correction
  of about 65 us toward the hang, to be applied with the scales.
- Evening, desk only (Andres chose not to touch power or logic wiring
  late at night; `pwm_clock_check.py` is written and waits for a day
  session). The probe's scales went into SERVO_MAP and the xacro: each
  planar joint keeps its measured pulses and zero, and its radian limits
  are the nominal ones times its scale, so every command in radians is a
  real angle. Consequences the tests caught:
  - The wave's in-swing (right elbow -2.09) no longer fitted: the right
    elbow reaches -1.94 rad real. Redrawn with the same base angle and
    the same 28 deg swing, moved down: up -1.60, open -1.35, close -1.84.
    Physically the new close curls further than the old one would have
    (-2.09 nominal was 1.69 rad real). To verify with `wave --step`.
  - The right arm holds a level gripper over less of the plane: the
    workspace test's point moved from (0.05, 0.06) to (0.08, 0.06). The
    reach map and the deck drawing are stale until regenerated (v0.3).
  - Every other angle in the repo is now real too: on the right arm a
    command reaches about 1/0.83 more metal than before, e.g. the hand
    show's LIFT_MAX 0.79 is now 45 deg of base, not about 38 (the model
    puts a straight arm at 45 deg near 8.0 kg.cm).
  - Not applied yet: the right elbow zero (+5 deg forward at command 0),
    to be confirmed on the metal first.
- 19:28, first run of the redrawn wave in step mode, gate 1 opened and
  armed by Andres: `wave_raise_2` (elbow -1.60 real) folded the right
  forearm into the robot's body. Stopped, unwound home, every joint read
  0.0 at 19:30. Incident row in `docs/safety.md`. New diagnostic
  sequence `elbow_reach` to find where the curl really touches.
- `elbow_reach` in step mode (19:38): the last clean stair is -1.30 rad
  (74 deg of curl, base -0.87); Andres quit there. The wave was redrawn
  under it: half -0.50, up -0.95, open -0.70, close -1.20, the base still
  -0.87. The forearm now swings between about horizontal and 30 deg above
  it, a lower wave than designed: the body sets the ceiling. Pinned by
  `test_the_wave_keeps_a_tenth_of_a_radian_short_of_the_measured_clean_curl`.
- The redrawn wave ran clean on the metal (Andres, about 19:45). Then a
  new sequence on his idea, `hello`: the right arm rises folding the
  elbow, waves the hand at the top (wrist roll +-0.45 rad five times, the
  gripper opening on every out) and comes down unfolding, 12.8 s. Every
  dwell is the travel time plus 0.1 s so it flows. Designed against the
  bench: base -2.0 (0.1 inside the right base's real reach), not truly
  vertical so gravity keeps one side of the gear play loaded; the elbow
  folded to -1.0 before the base passes the horizontal, about 9.4 kg.cm
  on the base by a point-mass model fitted to the 11.4 of a straight arm
  (the wave, about 9.7 by the same model, has held); the fold 0.3 rad
  short of the measured clean curl. Not run on the metal yet.
- `hello` ran clean in its first form. Andres asked for a higher arm, a
  more open elbow and the palm waving with the wrist pitch instead of the
  roll. Second form: base -2.08 and shoulder -0.28 (0.1 inside their real
  reach: the upper arm at about 135 deg, as high as these two joints go;
  truly vertical is out of reach on this bench), elbow -0.35, the palm
  swinging -0.025 .. -0.675 on the pitch, five times with the gripper.
  Base 5.7 to 7.0 kg.cm at the top by the model.
- `hello`, second form ran clean too. Third form on Andres's call: at
  the top the elbow nearly straight (-0.15) and waving the forearm
  +0.15 .. -0.45 five times with the gripper, the wrist in line. The
  model puts the hand lower than the second form (about 240 mm above the
  shoulder against 290) and the base at about 8.8 kg.cm at the out-swing
  (+0.30 would have been 9.3). Run without step mode at Andres's request:
  the rise and the fall are the paths already proven.
- `hello`, third form ran clean and looked better. Fourth form, Andres's
  call: the forearm pointing at the ceiling while it waves. Elbow
  -0.25 .. -0.75 around -0.50 (forearm about 150 .. 178 deg), stopping
  short of vertical so the elbow's gravity torque never flips mid-swing.
  Base about 6 kg.cm at the top.
- `hello`, fourth form: Andres pointed at the shoulder, folded in at
  -0.28 at the top. Fifth form: the shoulder opened outward to +0.16
  (upper arm about 110 deg), the elbow waving -0.70 .. -1.15 (forearm
  about 150 .. 176 deg). A horizontal upper arm was asked for in spirit
  but would need the elbow at -1.54, past the curl measured clean (-1.30)
  and near the one that touched (-1.60). Rereading that contact: at
  `wave_raise_2` the forearm was moving up and forward, away from the
  box; the likelier contact is the forearm on its own arm, which depends
  on the fold, not on the posture. The hello's fold margin went from 0.3
  to 0.15 rad. First run of the new top in step mode is advised.
- Andres: the workbench seemed to raise the right arm higher than the
  hello does. The base's map end is the very pulse captured there (2425
  us); the hello stops about 6 deg short of it. What may differ: the
  workbench's all-joints master moved the shoulder too, or the 0.83 base
  scale (measured over 0 to 34 deg) does not hold at the ends. New
  diagnostic `shoulder_reach`: the hello's top, then the shoulder up in
  0.1 rad stairs to 0.1 inside its map end, the upper arm read above the
  horizontal at each stair.
- Andres: the idle left arm hunts at its hanging zero while the right
  arm works. New driver parameter `limp_arm` ('' | 'left' | 'right'):
  that arm's channels get FULL_OFF every tick and its commands are
  dropped. Refused unless every joint of the arm is at home, now and in
  its target: rule 3 still holds (a released arm servo anywhere else
  falls), and no arm joint joined RELEASE_WHEN_SETTLED. Pure logic in
  `limp.py`, node tests in `test_node_safety.py::TestLimpArm`.
- The right base could not go higher (the hello's top is about 80 us
  short of the 2425 us end captured in October); left as technical debt.
  Andres moved to the left arm, whose base reaches about 142 deg real,
  and posed it by hand in the workbench: four poses read as pulses (a
  top with the shoulder open, a bent-in pose, out with the hand open, in
  with it closed) and a fifth that is home within the gear play. They
  became `hello_left`, kept as pulses in the code and converted through
  the calibrated map: all inside the soft band with the 0.1 margin, the
  base at most 7.8 kg.cm by the model. Two waypoints fold the elbow
  while the base passes the horizontal. 10.8 s.
- `hello_left` ran (Andres: "Genial"). The way down felt forced against
  the body: it paused at the rise waypoint with the forearm folded in,
  about 9.9 kg.cm on the base. New way down: it still crosses the
  horizontal folded (straight, the base would see about 10.9, past the
  servo), the shoulder opening 0.1, 0.3, 0.6, then the elbow opens below
  the horizontal and the arm comes home away from the body (about 5.8).
- Andres preferred the plain answer: the way down is the way up played
  backwards (4, 1, lift, rise, home), the road already proven on the
  metal. The opened descent was dropped before it ran.
- `hello_left` with the reversed descent ran fine. Andres asked for no
  pauses between steps. The pauses were not the dwells (travel + 0.1 s)
  but the segments themselves: a rest-to-rest minimum-jerk segment ends
  at zero speed on every pose. The driver already replans from the
  current position and velocity (trajectory.py), so `_flowing` gained a
  `blend`: each pose is sent on at 70 % of its travel and the arm flows
  through it; only home is reached and held. The dwell contract test now
  admits this only for sequences declared in BLENDED, and a new test runs
  the driver's own planner tick by tick over the sequence: arm joints stay
  in their soft band and within 0.1 rad of their poses' span (worst 0.019
  rad, the wrist pitch); the gripper may overshoot its closed stop by 0.02
  in the plan, where the pulse saturates. 10.8 s became 7.0 s, the wave
  keeps its full amplitude.
- Flowing `hello_left` ran well; on the way down the forearm came close
  to the body, the shoulder too closed. The low waypoint of the descent
  only (`hello_left_fall`) is the rise with the shoulder 0.25 rad more
  open: upper arm 57 -> 43 deg, base 9.6 kg.cm held instead of 9.9.

## 2026-10-27, dimensions re-measured for the inverse kinematics

Andres re-measured the right arm with a new digital caliper (the left is
its mirror) before trusting the IK (`scripts/workspace_map.py`). Chain:
clavicle 32.7 (33.0), upper 80.0 (79.3), fore 76.0 (75.0), wrist 66.3
(66.4), gripper base 17.8 (17.9), finger 65.0 (65.7) mm. The tool read in
one go, wrist pitch axis to fingertip: 150.0 closed (the parts add to
149.1; the direct reading is the IK's tool), about 100 fully open (the
pinions stand out past the fingers; the TCP moves about 50 mm with the
opening). Disc outer faces 107.5 apart, each disc 9.7 proud of the box
(7.4 in August), so the box is about 88.1 under the discs (109.8 then).
Closed tips 162 apart, arms hanging: the lane is 81 mm, not the 62.3 the
model had (disc face) nor the 180 of 2026-10-26 (discarded).

Applied: dimensions.yaml and the xacro (measured, caliper, 2026-10-27);
a `plane_offset` of 27.25 mm along the parallel axes (a DH d: it moves
the lane, never x or z) at the shoulder origin, so the model's planar
chain works in the measured lane; `workspace_map.py` uses the measured
lane and the direct tool. `mount_h` (343) is NOT changed: Andres read
about 335 for the disc and 38.7 mm from the closed tip to the plate,
while 338.7 mm of chain under a 335 disc would put the tip under the
plate. The arm cannot be folded 30 to 45 deg unseen, and the right elbow
read +5 deg at zero the day before (under 1 mm of tip height), so the
likelier change is the disc height: the system was moved on 2026-10-26
and the column belongs to a monitor stand. One reading decides it: disc
center to fingertip with the arm hanging limp (about 339 if straight).
- v0.2 closed. The criterion read "`soma_primitives wave` played end to
  end under the ramp, filmed". The wave ran clean on 2026-10-26 but the
  film is of `hello_left` (Andres's phone, low angle in front, 11 s).
  Andres chose to close v0.2 with it, and the README says so: it asks
  more of the driver than the wave (real radians, six joints at once,
  the base near 142 deg, blending, the idle arm limp). Committed at 720
  px H.264 (1.5 MB): no people in frame, and the terminal on the monitor,
  readable at 4K, is not at 720. Tag `v0.2`.
- Disc to closed tip, right arm hanging: about 293 mm; disc to plate 335
  on both (hard to reach the center); tip to plate 38. They agree with
  each other (335 - 38 = 297): the disc height is not the problem, the
  arm is not straight, or a link is not what the model says; about 45 mm
  are missing against the 338.7 mm chain. New `scripts/arm_side_view.py`
  to find where: the OAK from the side, blue tape on each axis screw,
  each link's tilt and each joint's angle; "down" from the OAK's own
  BMI270 IMU through the factory IMU-to-camera rotation (camera x = IMU
  y, y = IMU x, z = -IMU z), so no plumb line and any mounting works.
- The 45 mm, found. Tape: plate to the central box's top edge 370, so
  the disc axis is at 327.2 (half the 85.6 box below), the closed tip 38
  over the plate, disc to tip 295. The OAK from the side, with Andres's
  tape marks on the disc center, the shoulder center, under the box, the
  wrist and the tip (scale from the wrist-to-tip 152 = tool 150, the box
  top-to-tip 332 = 370 - 38 checked the depth scale): the shoulder axis
  sits 15.6 mm ABOVE the disc axis and behind it; from the shoulder the
  arm hangs straight, 160 mm to the wrist (80 + 76 = 156) and 150 more
  to the tip; disc to tip 294. The clavicle does not hang: it leaves the
  disc at 118.5 deg from straight down, backward (asin(15.6/32.7) = 28.5
  deg over the horizontal), and the shoulder's zero takes that back out.
  Not a wrong zero and not a wrong link: a mounting angle the model
  lacked, a DH theta offset. Applied: `clavicle_angle` 2.068 rad in
  dimensions.yaml, the xacro (shoulder origin turned, frame turned back)
  and `workspace_map.py` (fk, ik); `mount_h` 343 -> 327.2. The FK now puts
  the hanging tip 36.8 mm over the plate against 38 measured (pinned by
  a test, 3 mm). The reach map changed with it: the right lane loses its
  front reach above about 40 mm (its short yaw and wrist bands).
- Vertical rise of the shoulder axis over the disc axis, read by Andres:
  about 6.8 mm ("no way to measure it exactly"); horizontal 31.5. With
  the 32.7 clavicle that is a rise of 7 to 9 mm, not the 15.6 the tape
  strips gave the camera. The FK keeps the camera's angle for now (it
  predicts the hanging tip at 36.8 against 38); with a 7 to 9 mm rise it
  would say 29 to 33. A 5 to 9 mm residual that rough measurements
  cannot split: it goes to a least-squares fit on the tape-cross trials
  (mount_h, clavicle_angle, the base and shoulder scales).
- No CAD for the kit exists that we could find. It looks like the
  ARM-21N2 family (CHUNYE): its listings give "head 66 mm" and "forearm
  156 mm", matching our caliper's wrist 66.3 and upper arm + forearm
  80 + 76: an independent check of those links, nothing on the clavicle.
- Physics of the first reach test, left lane, gripper vertical, by the
  model: reaching under the hanging tip (37 mm) needs the base to swing
  the clavicle down, so every pose there pairs a big base angle (-0.85
  to -1.65 rad) with a big opposite shoulder angle (+0.7 to +1.2). Loads
  are small (base at most 4.7 kg.cm, shoulder 4.6). The error budget is
  not: gravity sag through the measured gear play drops the tip 2 to 10
  mm (systematic, predictable), and the scale uncertainty of the joint
  probe (5 % a joint, 9 % the shoulder) moves it 6 to 40 mm at 1 sigma,
  growing with the joint angles. Best ground: tip 30 mm over the plate,
  x from -30 to 0 (about 2 mm of sag, 6 mm of scale). Far from home the
  5 mm target needs the scales fitted from the trials, not the phone.
