# v0.2 bench session: the runbook

Print this, take a pen. One session: the first joints ever moved through the
ROS driver, the right-arm sign check, the wave, the shoulder threshold, and
the left arm if everything was clean. Every box is ticked by hand, every
verdict is written on paper before it is typed.

Rules that hold for the whole session ([safety.md](safety.md), CLAUDE.md):

- Nothing is armed without Andres saying so, out loud, for this session.
- **If in doubt, ATX rear switch off.** It is never the wrong move.
- After any ATX-off stop, **disarm before the ATX comes back on**
  (`/soma/arm false`): an armed driver drives every servo the moment the
  rail returns.
- In step mode abort with `q`, which unwinds. Ctrl-C ends the program and
  leaves the arm where it is, held by the driver (recovery in step 7).

Commands run on the Jetson, from `/home/jetson/soma-arms`, with the `soma`
helper of [bench.md](bench.md) section 6 pasted into the SSH session.

---

## 0. Before anything is powered

- [ ] Mechanical fixing done: boards, UBECs and cables screwed or tied down,
      nothing loose on the bench that a moving arm can tug.
- [ ] Servo connectors seated with their retainers; I2C harness checked.
- [ ] Both arms hanging at rest, nothing under or around them.
- [ ] Hands and tools clear. The ATX rear switch within reach of the hand
      that is not on the keyboard.
- [ ] Capture (plan section 10.1), BEFORE the ATX: 1 still wide (the bench
      as it is), 1 still close (the right arm, which is about to move).
      Destination `~/Robotics/capturas/YYYY-MM-DD/`.

Abort at any moment, for any of: a sustained hum (a stall), heat, smell, a
joint moving hard against its expectation, or any motion nobody commanded.
The abort is the ATX rear switch; `/soma/arm false` only when everything is
calm and the goal is simply to stop.

## 1. Power-up

- [ ] Jetson on its own adapter (never from the ATX). `ssh jetson@192.168.1.2`,
      `cd /home/jetson/soma-arms`, paste the `soma` helper.
- [ ] Current code, container recreated (it rebuilds on boot, about a minute):

```bash
git pull --ff-only
docker compose -f docker-compose.jetson.yml up -d --force-recreate
docker compose -f docker-compose.jetson.yml logs -f      # until "MOCK backend, DISARMED", then Ctrl-C
```

- [ ] The log says `SOMA driver up: MOCK backend, DISARMED`.
- [ ] Gate 1 is closed: `soma ros2 param get /soma_driver allow_real` says False.
- [ ] Quoting self-test: `soma printf '[%s]\n' "{data: true}"` prints `[{data: true}]`.
- [ ] `i2cdetect -y -r 7` shows `0x40` and `0x43` (and `0x70`). A missing board
      is a harness fault: stop.
- [ ] Silence both boards before the rail comes up ([wiring.md](wiring.md),
      bring-up step 3). With V+ off this cannot move anything:

```bash
i2cset -y 7 0x40 0xfd 0x10
i2cset -y 7 0x43 0xfd 0x10
```

- [ ] Arms still hanging at rest, hands clear. **ATX on.** The power-on
      twitch is expected.
- [ ] Multimeter: 6.00 V on both V+ terminal blocks. ______ V / ______ V

## 2. Mock rehearsal (optional, 3 minutes)

The driver is disarmed and gate 1 is closed: this walk moves `/joint_states`
and nothing on the bench, with the rail live. It proves the gates and
rehearses the prompts.

```bash
soma ros2 run soma_driver soma_sign_check --arm right
```

- [ ] The tool started (it refuses unless the driver listens and every right
      joint reads 0.0).
- [ ] Six prompts, six returns to 0.0, **no metal moved**. Answer ENTER to
      everything; the summary of a rehearsal means nothing.

## 3. Gate 1: relaunch with `allow_real`

```bash
ALLOW_REAL=true docker compose -f docker-compose.jetson.yml up -d --force-recreate
docker compose -f docker-compose.jetson.yml logs -f      # wait for both lines below
```

- [ ] `SOMA driver up: MOCK backend, DISARMED`.
- [ ] `allow_real:=true. Calling /soma/arm with data:=true WILL energize the servos.`
- [ ] `soma ros2 topic echo /joint_states --once`: every arm joint at 0.0
      (the relaunch is a fresh boot).

## 4. Gate 2: arming (Andres only, out loud)

What happens ([safety.md](safety.md) rule 2): the next tick sends every
servo the pulse of its calibrated zero, all at once and at the servo's own
speed. The hanging joints are already there; a wrist roll or a gripper left
elsewhere snaps to zero.

- [ ] Hands clear of both grippers. ATX switch in reach.
- [ ] Andres: "arming".

```bash
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"
```

- [ ] The reply says `ARMED: real PCA9685 output is live`.
- [ ] Wait 3 s and watch. The shoulders may fall into the known limit cycle
      at the hanging zero (2026-10-01): not a fault, note it. **Anything else
      moving: ATX off**, then disarm before anything else.

Shoulder limit cycle at arming: [ ] none [ ] right [ ] left

## 5. Right-arm sign check

Each joint: the tool prints the command and what the metal should do. ENTER
moves it 0.25 rad toward the hug side (never within 0.1 rad of a stop), holds
2 s, brings it back to 0.0, then asks. Judge the METAL. Write the verdict in
the table first, then type it: ENTER ok, `r` reversed, `u` unclear (the same
joint again), `s` skip, `q` quit. Every excursion starts from the hang, so
it can only lift the fingertips; nothing on the bench is in reach of a
0.25 rad step, whichever way the metal turns.

RViz is not part of this session: there is no validated way yet to show it
beside the bench. Leave the RViz column for the desk check of section 10,
and mark `m` (model) only if RViz happens to be open next to the metal.

- [ ] One-way door (plan section 10.2): start a vertical clip before the
      first ENTER. The yaw is the first joint ever moved through the ROS
      driver.

```bash
soma ros2 run soma_driver soma_sign_check --arm right --report /ros2_ws/src/soma-arms/calibration/sign_check_$(date +%F)_right.json
```

The report lands in `/home/jetson/soma-arms/calibration/` on the Jetson.

<!-- Generated by soma_driver.sign_check.expectation_table('right'); a test keeps it identical. -->
| # | joint | command (rad) | the metal should | metal | RViz | notes |
|---|---|---|---|---|---|---|
| 1 | `right_arm_yaw_joint` | -0.25 | the whole arm swings FORWARD, away from the column (verified on the metal 2026-10-01) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 2 | `right_arm_finger_l_joint` | +0.25 | the gripper OPENS 25% of its travel | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 3 | `right_arm_wrist_roll_joint` | -0.25 | the gripper rolls INWARD, toward the end captured as MIN (note CW or CCW seen from above) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 4 | `right_arm_wrist_pitch_joint` | -0.25 | the gripper pitches FORWARD | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 5 | `right_arm_elbow_joint` | -0.25 | the forearm folds FORWARD (the hug bend) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 6 | `right_arm_shoulder_joint` | -0.25 | the upper arm rises to the FRONT | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |

- [ ] The yaw went FORWARD. If it did not move at all, or went backward, `q`
      and stop: the yaw is already verified, so the pipeline is what failed
      (armed? which board? which channel?).
- [ ] Summary copied (photo of the screen is fine) and the report written.

## 6. Decision

- [ ] **All six `ok`**: go to 7.
- [ ] **Any `reversed`** (or a `model`, if RViz was open after all): end of
      motion for the day.

```bash
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: false}"
```

  then ATX off, then section 9. The fixes are desk work, with the driver
  disarmed and the ATX off: paste the `SERVO_MAP` row and the xacro limits
  the summary printed, update the rows pinned in
  `soma_driver/test/test_servo_map.py` with the reason, redesign every pose
  the summary flagged (a mirrored right elbow or yaw breaks the wave), rerun
  the tests and `scripts/workspace_map.py` (without `--flip`). Never "try it
  anyway".
- [ ] **Any `unclear` left**: that joint is not verified. The wave may only
      run if the right yaw, elbow and finger (the joints it moves) are `ok`.

## 7. The wave

Step mode first: one pose per ENTER, `q` unwinds along the walked path.

```bash
soma ros2 run soma_driver soma_primitives wave --step
```

- [ ] `wave_probe`: the base swings FORWARD (the same move as sign check #1).
- [ ] `wave_raise_1`, `wave_raise_2`: the forearm curls up in front.
- [ ] Three out-swings with the hand opening, three in-swings closing.
- [ ] Back down the same way, ending at home.

Then the real thing, filmed (20 s vertical clip):

```bash
soma ros2 run soma_driver soma_primitives wave
```

- [ ] Played end to end, the ATX switch never needed. Clip saved.

If Ctrl-C was pressed instead of `q`, the arm stays at the last pose, held by
the driver. Walk it home by hand, one pose per command, back along the path
(from the top: `wave_raise_2`, `wave_raise_1`, `wave_probe`, `home`):

```bash
soma ros2 run soma_driver soma_primitives wave_raise_2
```

## 8. Shoulder threshold

The hanging zero is where the backlash limit cycle lives (2026-10-01). Find
how far forward the shoulder has to sit for it to stop: that number sizes the
rest pose. Arm hanging, every other joint at 0.0. Step forward (the hug side),
5 s per step, never past -0.25:

```bash
soma ros2 topic pub --once /soma/command sensor_msgs/msg/JointState "{name: [right_arm_shoulder_joint], position: [-0.05]}"
```

| command (rad) | -0.05 | -0.10 | -0.15 | -0.20 | -0.25 |
|---|---|---|---|---|---|
| pulse (us, `SERVO_MAP` of 2026-10-01; zero is 990) | 958 | 926 | 895 | 863 | 831 |
| still oscillating? | [ ] | [ ] | [ ] | [ ] | [ ] |

- [ ] First angle where it stops and stays stopped: ______ rad, ______ us.
- [ ] Back to 0.0 (the same command with `position: [0.0]`).

The pulse of any angle, from the map itself:

```bash
soma python3 -c "from soma_driver.servo_map import SERVO_MAP as M; print(M['right_arm_shoulder_joint'].command_to_us(-0.10))"
```

## 8b. The left arm (same day, only if everything above was clean)

Still armed from step 4. Step 5 with `--arm left` and the table below,
then the decision of step 6, then step 8 with `left_arm_shoulder_joint`
(its zero is 1075 us; use the one-liner for the pulses). The wave is
right-arm only. Then close.

```bash
soma ros2 run soma_driver soma_sign_check --arm left --report /ros2_ws/src/soma-arms/calibration/sign_check_$(date +%F)_left.json
```

<!-- Generated by soma_driver.sign_check.expectation_table('left'); a test keeps it identical. -->
| # | joint | command (rad) | the metal should | metal | RViz | notes |
|---|---|---|---|---|---|---|
| 1 | `left_arm_yaw_joint` | -0.25 | the whole arm swings FORWARD, away from the column (verified on the metal 2026-10-01) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 2 | `left_arm_finger_l_joint` | +0.25 | the gripper OPENS 25% of its travel | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 3 | `left_arm_wrist_roll_joint` | -0.25 | the gripper rolls INWARD, toward the end captured as MIN (note CW or CCW seen from above) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 4 | `left_arm_wrist_pitch_joint` | -0.25 | the gripper pitches FORWARD | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 5 | `left_arm_elbow_joint` | -0.25 | the forearm folds FORWARD (the hug bend) | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |
| 6 | `left_arm_shoulder_joint` | -0.25 | the upper arm rises to the FRONT | [ ] ok [ ] reversed [ ] unclear | [ ] ok [ ] model | |

## 9. Close

```bash
soma ros2 run soma_driver soma_primitives relax          # home, settle, then disarm
```

- [ ] `disarm: DISARMED: signal cut, back on MOCK`.
- [ ] ATX off.
- [ ] Gate 1 closed with a plain relaunch (a container created with
      `ALLOW_REAL=true` would come back open after a reboot):

```bash
docker compose -f docker-compose.jetson.yml up -d --force-recreate
```

- [ ] The Jetson stays up.
- [ ] Capture (plan section 10.1): 1 clip of 20 s, the most surprising thing
      of the day; 1 line per capture in the log.
- [ ] The report comes home (from the Mac, in the repo root) and goes in
      with the log entry:
      `scp jetson@192.168.1.2:soma-arms/calibration/sign_check_*.json calibration/`
- [ ] `docs/log.md` entry: date, what moved, the verdicts, the shoulder
      threshold, the arming limit cycle, incidents (with a row in
      `docs/safety.md` if any), captures.
- [ ] Tag `v0.2` only when all 12 joints are `ok` on both arms and the wave
      is on video.

## 10. Desk check of the model (no hardware)

The RViz column. It needs no metal: the model is right when RViz draws each
command the way the expectation says, and the metal has nothing to add once
its own verdicts are in. Run it at a desk, any day after the session, in the
VNC container of [bench.md](bench.md) section 3, with the mock driver:

```bash
ros2 launch soma_driver driver.launch.py                      # mock, disarmed
ros2 launch soma_description display.launch.py gui:=false     # RViz follows /joint_states
ros2 run soma_driver soma_sign_check --arm both
```

- [ ] For each joint, ENTER if RViz draws the expectation, `m` if it draws
      the other way. For the wrist rolls, compare with the CW or CCW written
      in the notes at the bench.
- [ ] Every `model` line of the summary is an `<axis>` change in
      `soma_arm.xacro`; the line serves both arms, so a fix only one arm
      needs takes a per-side parameter. The driver and the reach map do not
      change.
