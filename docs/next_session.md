# Next session: bench runbook v0.2 and a corrected sign check

Handoff written 2026-10-04 by the cloud session that closed that day. From
here on, sessions run locally on Andres's Mac (they can reach the Jetson at
192.168.1.2; the cloud could not). This file is the approved plan for the
next working session; delete it when its items are done and logged in
`docs/log.md`.

State: `main` is at the plan restructure plus `docs/log.md`, CI green. Nothing
has moved through the ROS driver yet. Only the base direction is verified
(yaw negative = forward).

## Why the bench tools are not ready yet (audit of 2026-10-04)

- `soma_sign_check` walks all 12 joints of both arms with no per-arm option;
  it commands the right yaw +0.25 rad (backward) within 1.4 deg of its
  measured stop (+0.2749); a Ctrl-C during the dwell leaves that joint at
  +-0.25; and when a mismatch is marked it prescribes "an axis flip in
  soma_arm.xacro", which is the wrong fix: metal moving opposite to the hug
  convention is a DRIVER fix (mirror the `SERVO_MAP` row), and every
  `<axis>` line uses `${side}`, so a flip there changes both arms. No tests.
- `docs/bench.md` has no Jetson recipes (only "On the Pi").
  `docker-compose.jetson.yml` cannot pass `allow_real:=true`, so the relaunch
  that `docs/safety.md` rule 2 asks for has no recipe; only `ros2 param set`
  works, and the compose header discourages it.
- `docs/safety.md` says arming never causes motion. In code, arming retargets
  all 12 servos to their calibrated zero: a servo not physically at zero
  moves there under the ramp.
- `docs/wiring.md` still describes the Pi and LiPo bench (bus 1, only
  `0x40`, no ATX, second UBEC "planned").
- Emergency stop today is "unplug the XT60". Pulling an XT60 under load is
  not a stop. The Jetson runs on its own adapter, so the ATX rear switch
  cuts 12 V, the UBECs and the 6 V rail without touching the Jetson: that is
  the stop the runbook uses.
- Jetson: user `jetson`, repo at `/home/jetson/soma-arms`, bus `i2c-7`,
  container `soma_driver`, compose run from the repo root.
- If the sign check forces a mirrored right elbow, the wave values
  (-0.90 to -2.09) fall outside the new band: the wave is redesigned after
  the sign check, never before.

## 1. Rewrite `soma_sign_check`: pure logic plus an rclpy adapter

New `soma_driver/soma_driver/sign_check.py` (no ROS, same pattern as
`player.py`):

- `ARMS = ('right', 'left')`; `joints_for(arm)` in confidence order: yaw
  (already verified, so it proves the pipeline), finger_l, wrist_roll,
  wrist_pitch, elbow, shoulder. `both` means right then left.
- `excursion(name, delta=0.25, margin=0.1)`: the NEGATIVE (hug) side when
  `lower <= -(delta + margin)`, else positive when `upper >= delta + margin`,
  else the larger available travel minus the margin. Never within 0.1 rad of
  a stop (right yaw becomes -0.25, fingers +0.25).
- `expectation(name, target)`: text per joint and per COMMANDED SIGN, in
  bench terms (forward = away from the column): yaw negative, the base swings
  FORWARD; shoulder negative, the arm rises to the front; elbow negative, the
  forearm folds forward (the hug bend); wrist_pitch negative, the gripper
  pitches forward; wrist_roll negative, it rolls inward; finger positive,
  opens a quarter. Positive = the opposite. "(verified on the metal
  2026-10-01)" only on the yaw.
- Verdicts: `ok`; `reversed` (metal went the other way: driver fix);
  `model` (metal fine, RViz drew it the other way: axis flip, model only);
  `unclear` (repeat the joint). `Finding(joint, target, verdict)`.
- `mirror(spec) -> ServoSpec`: swap `min_us` and `max_us`, `lower' = -upper`,
  `upper' = -lower`. It preserves the zero pulse (`command_to_us(0.0)`) and a
  double mirror is the identity. `servo_map_line(name, spec)` prints the row
  in the format of `servo_map.py`.
- `summary(findings)`: per `reversed` joint, the corrected `SERVO_MAP` row,
  the new limits for the xacro (`soma_arm.xacro` defaults for the right arm,
  `soma_bench.urdf.xacro` for the left) and the sentence "this is a driver
  fix, NOT an axis flip"; per `model` joint, the `<axis>` line to flip with
  the warning that it affects both arms unless a per-side parameter is
  added; plus the wave warning when the right elbow changes. No findings:
  "no reversals; the hug convention survived contact with power".
- `run(arm, publish, sleep, ask, log, delta, dwell) -> list[Finding]`: per
  joint print the command and the expectation, ask ENTER/s/q, publish the
  target, sleep `dwell`, publish 0.0, sleep `settle_time_s + 0.5`, ask the
  verdict (ENTER ok, r reversed, m model, u unclear repeats). `q`, Ctrl-C
  and EOF ALWAYS return the current joint to 0.0 before leaving. Never
  touches `/soma/arm`.
- `parse_cli(argv)`: `--arm right|left|both` (default right), `--delta`,
  `--dwell`, `--report PATH` (JSON of the findings), dropping the
  `--ros-args` segment.

`sign_check_cli.py`: new docstring (usage, order, verdicts, which fix each
verdict means, never arms), an adapter that builds the node, `publish(name,
position)` and delegates to `run`; prints `summary`; writes `--report` when
asked.

Tests `soma_driver/test/test_sign_check.py` (pure): order per arm and
`both`; no excursion within 0.1 rad of a stop on any of the 12 joints (right
yaw -0.25, finger +0.25); the expectation names the commanded direction and
only the yaw says "verified"; `mirror` keeps the zero on all 12 rows and is an
involution; `servo_map_line` reproduces the current right elbow row exactly;
`run` with fakes: publishes target then 0.0 per joint, `s` skips, `q` leaves
with the joint at zero, EOF and KeyboardInterrupt return to zero, `r` yields a
`reversed`, `u` repeats; the summary of a reversed right elbow contains
`ServoSpec(12, 535.0, 2460.0, -0.6283, 2.3955, 2.5)`, the word "driver", and
never recommends an axis flip; `parse_cli` cases. Plus
`test_sign_check_cli.py` with `importorskip('rclpy')` inside the test (ROS
job only).

## 2. Compose and operating docs

- `docker-compose.jetson.yml`: `ros2 launch soma_driver driver.launch.py
  allow_real:=${ALLOW_REAL:-false}`; header with the real recipe
  `ALLOW_REAL=true docker compose -f docker-compose.jetson.yml up -d
  --force-recreate` (gate 1 by relaunch), `ros2 param set` as the documented
  alternative, gate 2 through `docker exec`.
- `docs/bench.md`, new `## 6. On the Jetson`: `ssh jetson@192.168.1.2`,
  `cd /home/jetson/soma-arms`, `i2cdetect -y -r 7` (expect `0x40` and `0x43`;
  `0x41` is gone), compose up, logs, down; a shell function `soma()` that
  runs `docker exec -it soma_driver bash -c "source /opt/ros/humble/setup.bash
  && source /ros2_ws/install/setup.bash && $*"`; with it: `soma ros2 topic
  echo /joint_states --once`, `soma ros2 service call /soma/arm ...`,
  `soma ros2 run soma_driver soma_sign_check --arm right`, `soma ros2 run
  soma_driver soma_primitives wave --step`. A "can it move a motor" table.
- `docs/safety.md` rule 2: the Jetson commands (relaunch with
  `ALLOW_REAL=true`); correct "arming never causes motion" to "arming
  retargets every servo to its calibrated zero under the ramp; with the arms
  hanging at zero nothing visible happens, a servo away from its zero moves";
  the emergency stop is the ATX rear switch, never the XT60 under load.
- `docs/wiring.md`: today's power chain (ATX, 10 A fuse, XT60, Y splitter,
  two 6 V UBECs, the Jetson on its own adapter, INA off the bus), bring-up on
  bus 7 with `0x40` and `0x43`, order: Jetson and container first (boots MOCK
  and disarmed), then the ATX with the arms hanging at rest (power-on twitch
  expected), 6.00 V on both blocks, then the gates.
- `docs/plan.md` section 4 (v0.2 step 2) and `docs/log.md` 2026-10-03: where
  they say "flip in the xacro and `--flip`", clarify: metal reversed = mirror
  the `SERVO_MAP` row (the reach map then needs NO `--flip`); RViz reversed =
  axis flip (then `--flip`).

## 3. `docs/session_v02.md`: the runbook (printable, with checkboxes)

0. Prerequisites and aborts: mechanical fixing done, cables secured, arms
   hanging at rest, hands clear, the ATX switch within reach; abort on a
   sustained hum, heat, smell, a joint moving hard against the expectation,
   or any uncommanded motion (`/soma/arm false` or ATX off). Capture 10.1:
   wide and close stills BEFORE the ATX.
1. Power-up: Jetson on its adapter, `compose up`, logs say "MOCK backend,
   DISARMED", `i2cdetect`, ATX on (twitch), 6.00 V on both blocks.
2. Mock rehearsal (optional, 3 min): `soma_sign_check --arm right` with the
   driver disarmed; `/joint_states` moves, the metal does not.
3. Gate 1: relaunch with `ALLOW_REAL=true`; the "WILL energize" warning is in
   the logs.
4. Gate 2 (Andres only, out loud): `/soma/arm true`; wait 3 s; the shoulders
   may enter the known limit cycle (not a fault, note it); anything else
   moving, ATX off.
5. Right-arm sign check with the printed expectation table (joint, command,
   expected, boxes ok / reversed / model). One verdict per joint. Copy the
   summary.
6. Decision: all `ok`, go to 7. Any `reversed` or `model`: `/soma/arm false`,
   ATX off, end of motion for the day; mirror the rows, sync the URDF,
   redesign the wave, rerun the tests. Never "try it anyway".
7. `wave --step` (one pose per ENTER, `q` unwinds), then without `--step`,
   filmed (20 s vertical clip).
8. Shoulder threshold: with the arm hanging, command `right_arm_shoulder_joint`
   to -0.05, -0.10, ... until the oscillation stops; note the angle and the
   pulse (`SERVO_MAP[...].command_to_us`) in `docs/log.md`.
9. Close: `relax` (home, then disarm), ATX off, Jetson stays; `docs/log.md`
   entry (date, what moved, verdicts, threshold, incidents, captures). All
   12 joints `ok` on both arms and the wave on video: tag `v0.2`. Left arm:
   same sequence, same day if everything was clean.

## Verification

- `python -m pytest soma_driver/test -q` green with the new tests;
  `python -m py_compile` of the CLI.
- `docker compose -f docker-compose.jetson.yml config` validates the YAML.
- No em dashes in touched files. CI green on `main` after the push.
- The runbook is checked against `docs/safety.md` rules 1 to 5: no step arms
  anything without Andres.
