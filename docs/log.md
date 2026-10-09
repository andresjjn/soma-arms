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
