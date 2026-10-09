# SOMA master plan

How this robot gets built, in order, with what hardware, and what has to be
true before each step counts as done. The README holds the short roadmap
table; this file holds the engineering behind it.

Versions are gates, not dates: a version is done when its acceptance test
passes on the bench, filmed, and the tag is pushed. Nothing is planned by
the calendar. This is a hobby bench with evenings, and a plan with dates on
it was already proven wrong once (the 2026-08 estimates).

One track. The learning track (MuJoCo and MJX, system identification, RL
policies) that this file carried until 2026-10-03 is gone; see section 8
for why and for what stays in the repository.

The project ends at v1.0: a two-station tending cell that runs for six to
seven hours doing the job of an operator, with a public cycle counter that
a reasoning model verifies from the camera. Then the arms move on.

---

## 1. Hardware, and what each piece is for

| Device | Role | Why it and not something else |
|---|---|---|
| 2 x 6DOF aluminum arm, 12 x MG996R | The robot | Already built, wired and calibrated (2026-10-01) |
| Actuonix L16-140-63-6-R | Torso lift, one prismatic DOF | Off the bench until the printed torso; self locking, so it holds with no power |
| 2 x PCA9685 (`0x40` right arm and L16, `0x43` left arm) | 13 PWM channels on one I2C bus | One bus, no motor drivers needed; channels 0 to 2 of `0x40` are free (the status lamp of v0.4 goes there) |
| 2 x UBEC 6 V, from the ATX bench supply | Servo rails, one per arm | The only thing between the supply and twelve dead servos; the Jetson runs on its own adapter after the inrush incident of 2026-09-01 |
| **Jetson Orin Nano Super 8 GB** | Robot brain: ROS 2, driver, OAK pipeline, ER 2 client, the cell controller | It is the computer the bench already runs on (JetPack 7.2, bus `i2c-7`) |
| **OAK-D Lite** | Eyes: RGB, stereo depth, fixed on the column looking down at the deck | Depth and a fixed pose are what turn a 2D point from ER 2 into a deck coordinate the arm can reach |
| MacBook Pro M3 Pro | Development, RViz, docs, CI watching | Not in the loop |
| Laser-cut wooden deck with two lanes and six pockets | The cell | Pockets fix where parts can be, which is what a planar arm needs (section 2) |
| Wooden blocks, about 40 mm, under 50 g, laser-engraved id | The parts | Light for a 330 g payload, square for a scissor gripper, readable by the model |

**Compute topology, and the rule that holds it together:**

```
Jetson Orin Nano                       PCA9685 x2 -> 13 actuators
  ROS 2 Humble                                 ^
  soma_driver  (armed by a human) --------------
  soma_agent   (ER 2 client, v0.5)
  depthai pipeline <--- OAK-D Lite
         |
         v
Gemini Robotics ER 2 (cloud, reasoning only)
```

Latency decides the split. Anything inside the control loop runs on the
Jetson. Anything that thinks in seconds (what is where, did the cycle
succeed, what does the part say) may live in the cloud. **The cloud is
never in the safety path**, and no remote response can arm the driver.

---

## 2. The cell: what these arms can physically do

Both arms hang from the central box with their four big joints (yaw,
shoulder, elbow, wrist pitch) on parallel horizontal axes. Each arm is a
planar 4R chain in its own vertical plane, 62.3 mm outboard of the bench
centerline (`mount_ly / 2 + disc_h`), so the two planes are **124.6 mm
apart**. The fingertip center sits on the roll axis and never leaves its
plane, for any joint configuration. Three consequences, none negotiable:

- Each arm works a **lane**: a strip of the deck under its plane. A part
  that is not in the lane cannot be reached, and a part that falls out of
  the lane stays out until a human moves it.
- The arms cannot hand anything to each other and cannot sort into bins
  that are not in their own lane. "Bimanual" on this bench means two
  independent single-lane cells side by side.
- A conveyor, a ramp or a rotary table would only matter as a way to
  bring parts INTO a lane. None is needed for the task of section 3.

`scripts/workspace_map.py` computes the reachable part of each lane from
the measured dimensions and the measured joint limits, with the gripper
vertical (or within a chosen tilt). Fingertip height is deck height plus
20 mm (the grasp point on a 40 mm block). Numbers of 2026-10-03, URDF sign
hypothesis, 0.1 rad kept inside every limit:

| deck (mm) | fingertip height (mm) | left lane x (mm, + forward) | right lane x (mm, + forward) |
|---|---|---|---|
| 0 | 20 | [-66, +71] | [-69, +71] |
| 40 | 60 | [-119, +131] | [-122, +120] |
| 60 | 80 | [-133, +149] | [-137, -20], [+66, +123] |
| 80 | 100 | [-143, -57], [+81, +162] | [-147, -32], [+105, +111] |

With the tool allowed to tilt up to 15 degrees from vertical:

| deck (mm) | fingertip height (mm) | left lane x (mm, + forward) | right lane x (mm, + forward) |
|---|---|---|---|
| 0 | 20 | [-92, +96] | [-95, +96] |
| 40 | 60 | [-153, +165] | [-156, +164] |
| 60 | 80 | [-169, +184] | [-172, +180] |

What the table says, honestly:

- Reach is short and it is **not monotonic in deck height**. At plate
  level the arm is fully stretched (fingertips 5.7 mm above the plate at
  home) and the lane is a 140 mm band around the axis. It widens up to
  about 60 mm of fingertip height, then a hole opens under the disc axis
  and the front run of the right arm shrinks, because the right wrist
  pitch may lean only 22 degrees forward (+0.4869 rad, measured) while the
  left one leans 74 degrees (+1.2959 rad).
- **Design choice: deck top 40 mm above the plate**, pockets at about
  -70, 0 and +70 mm along each lane (46 mm pockets for 40 mm parts at a
  70 mm pitch), tool tilt allowed up to 15 degrees for the picks.
- The plate ends 55 mm in front of the disc axis (`plate_x_back`, still an
  estimate). The deck overhangs it to the front, so the deck is a box that
  stands on the table around the plate, not a tray on the plate.
- The rear pocket sits beside the column: lane at 62.3 mm, column at 30 mm
  from the centerline, gripper 45 mm wide, about 10 mm of clearance. The
  first fit check happens with the deck in cardboard before the laser.
- The whole map rests on sign hypotheses. Only the yaw direction is
  verified on the metal (2026-10-01: negative = forward). Shoulder, elbow
  and wrist pitch are URDF guesses until the v0.2 sign check. A joint
  whose metal disagrees gets its `SERVO_MAP` row mirrored, and the map is
  then rerun as is, with no `--flip`: it reads the corrected limits, and
  its frame is the physical hug convention, not the URDF.

Torque, measured and estimated: an MG996R delivers about 10 kg.cm, not the
25 the frame was sold for; a hanging arm weighs about 0.7 kg (estimate,
never weighed). The base servo is near stall with the whole arm
horizontal. Stations stay close to the vertical, holds are short, parts are
light, and a shoulder is never parked at the gravitational zero, where the
backlash limit cycle of 2026-10-01 lives.

---

## 3. The task: a two-station tending cell

Each arm tends its own lane with three pockets: INPUT, MACHINE (a fixture
with a status lamp) and OUTPUT. One cycle:

1. The supervisor (ER 2, from v0.5) looks at the deck and points at the
   part in INPUT and at the empty MACHINE pocket. Before v0.5 the
   positions are the deck geometry and nothing looks.
2. The arm picks the part from INPUT and loads it into MACHINE.
3. The lamp goes on: the "machine cycle", a few seconds.
4. The arm unloads the part into OUTPUT.
5. The supervisor verifies the end state from the camera and reads the id
   engraved on the part. The counter increments only on a verified cycle;
   anything else is logged as a failure, with the frame.

When INPUT is empty the roles of INPUT and OUTPUT swap, so the same parts
flow back and the cell runs with nobody at the bench. The two arms work
interleaved: one moves while the other holds or waits, so no supply rail
sees both arms accelerating at once and the video always has something
moving.

Why this task and not another:

- It is the one job that respects section 2: one lane, no lateral
  correction, no handoff, stations near the vertical.
- It needs no mechanism that can fail in seven hours, and it resets
  itself.
- It gives the reasoning model a real, visible job (perceive, verify,
  read, recover) without putting the cloud in the safety path.
- It is, literally, machine tending: the most common job of a cobot.

Failure handling that the video has to show at least once: a part dropped
inside the lane is re-localized by the supervisor and picked again; a part
outside the lane is reported, the pocket is marked empty and the cell goes
on with the remaining parts. The cell never stops on its own for a
perception error; it stops for a power or driver fault.

---

## 4. The demo track, version by version

Each version ends with a git tag, a short video, and CI green. Every
"FILM BEFORE" is a one-way door (section 10.2).

### v0.1 Calibrated model, TAGGED 2026-08-10

Caliper session of 2026-08-05, the real hanging bench modeled, FK matched
the photos (fingertips 5.7 mm above the plate). The servo calibration was
re-captured on 2026-10-01 after the harness rebuild; `SERVO_MAP` and the
URDF carry it.

### v0.2 Real driver

The code exists and is tested (mock fleet, two arming gates, minimum-jerk
ramp, board-reset watchdog). This phase is the hardware run.

1. Bring-up order from `docs/wiring.md`, arms compact and resting, the
   Jetson on its own adapter.
2. `allow_real:=true`, then an explicit `/soma/arm` call by Andres, then
   `soma_sign_check` on the right arm, joint by joint
   (`docs/session_v02.md` is the runbook). Metal that moves opposite to
   the hug convention is a DRIVER fix: mirror that joint's `SERVO_MAP` row
   and sync its xacro limits (the tool prints both), never an axis flip,
   because every planar `<axis>` line serves both arms. RViz drawing a
   joint the other way while the metal is right is a MODEL fix (the
   joint's `<axis>` line) and needs no hardware to check. Then rerun
   `scripts/workspace_map.py` WITHOUT `--flip`: after a driver fix the
   command sign is the physical one again, and a model fix does not touch
   the map at all. `--flip` is only a preview for an evening before a
   driver fix lands. A mirrored right elbow or yaw sends the wave back to
   design before it plays.
3. `ros2 run soma_driver soma_primitives wave --step`, then without
   `--step`, filmed. The wave raises the right arm with the base, waves
   the forearm three times with the hand opening on every out-swing, and
   comes back the way it went. Step mode walks it one pose at a time and
   unwinds along the proven path if anything looks wrong.
4. Sign check of the left arm. Shoulder preload (band or spring, one
   direction, only what beats the backlash) and the "microseconds from zero
   where the oscillation stops" measurement, which sizes the rest pose.
5. Weigh one arm when it is off the rig, if it ever is. Until then 0.7 kg
   stays labeled an estimate.

**Acceptance**: all 12 joints sign-checked through the ROS driver, the
wave played end to end under the ramp with the emergency switch never
needed, filmed. Tag `v0.2`.

### v0.3 Cell geometry and eye-deck calibration

1. **FILM BEFORE.** The deck: lanes and pocket positions from the
   workspace map with the verified signs, mocked in cardboard first (fit
   beside the column, reach of every pocket by hand with the driver), then
   laser cut. `docs/workspace_map.svg` is the drawing the cutter starts
   from; pocket size follows the parts actually bought.
2. **FILM BEFORE.** The camera fixed to the column (a short mast if the
   35 cm minimum to the deck needs it), looking down at both lanes.
   Extrinsics die every time something shifts, so this is bolts, not tape.
3. Pixel to deck: a homography from four markers on the deck, stored in
   `dimensions.yaml` with provenance and date, published as a static TF.
   Depth from the OAK stays available for part height, not for the plane.
4. Planar IK from `scripts/workspace_map.py` promoted into the driver as a
   module with tests: fingertip (x, z) plus tool angle in, four joint
   targets out, limits respected, the yaw chosen for the least bend.
5. **Acceptance, deliberately brutal**: click a pocket in the camera image
   and the gripper touches its center within 5 mm, ten pockets out of ten,
   repeated after a power cycle to prove the calibration persists.

Tag `v0.3`. Video: the click to touch loop.

### v0.4 Scripted tending cycle, no cloud

1. Pick and place between pockets as driver primitives: approach from
   above, descend, close at the calibrated contact angle (foam pads on the
   fingers, never a stall), lift, move, descend, open, retreat.
2. The cycle of section 3 as a script, with the lamp on a free channel of
   board `0x40` (channels 0 to 2) as the machine signal.
3. 30 consecutive cycles per arm without a drop, both arms interleaved.
4. **Soak test: one hour continuous**, servo case temperatures measured
   every ten minutes (IR thermometer), rail current if the INA3221 is back
   on the bus, failures counted. If a servo passes 60 C the rest pose, the
   dwell or the parts change before anything longer is attempted.

**Acceptance**: 30 clean cycles and the one hour soak with its temperature
log committed. Tag `v0.4`.

### v0.5 `soma_agent`: the supervisor

1. `soma_agent` package: a client for `gemini-robotics-er-2-preview`
   behind a thin adapter (`scripts/er2_probe.py` is its seed). Pointing
   returns `[y, x]` in 0 to 1000; the homography of v0.3 turns it into a
   deck coordinate; the IK of v0.3 turns that into joints.
2. Function calling exposes exactly the v0.4 primitives and nothing more.
   **It cannot arm the driver, change a limit, or bypass the ramp**, and a
   test asserts the tool schema contains no such capability.
3. Cycle verification: the model's success detection on the end-state
   frame decides whether a cycle counts; the engraved id is read and
   logged; a dropped part is re-localized by pointing.
4. Fallback: with the API down or rate limited, the cell keeps cycling on
   geometry alone and counts those cycles as unverified, visibly.
5. Cost and latency log per call (tokens in, tokens out, thinking tokens,
   seconds), because the price doubles on 2027-01-01 and the preview can be
   retired without notice.

**Acceptance**: 50 verified cycles with one induced fault (a part knocked
over inside the lane) recovered without a human. Tag `v0.5`.

### v1.0 The operator

Six to seven hours of continuous cycles, both arms, a public counter
verified by the supervisor, timestamp on screen, the video unedited,
thermal and failure numbers logged and published as they are. Then the
project closes and the arms move to the mobile platform. Anything missing
becomes an issue, not a reason to keep it open.

---

## 5. Sequencing, and what unblocks what

```
v0.1 model ──► v0.2 driver ──► v0.3 cell + eye-deck ──► v0.4 cycle + soak ──► v0.5 agent ──► v1.0
                  │                 ▲
                  └── signs ────────┘  (the deck is cut only with verified signs)
```

- The workspace map, the deck drawing and the probe of the API cost
  nothing on the bench and are done.
- The API key, billing and the Developer Program credit are a one-evening
  admin task that gates v0.5 only; the first real pointing call on a frame
  of the bench can happen any time after v0.3 fixes the camera.
- Parts (six wooden blocks) and the lamp can be bought now.

---

## 6. Invariants that survive every phase

These do not bend for a demo, a deadline or an agent.

1. Motors move only after explicit prior confirmation, per session.
2. The driver boots on the mock backend and disarmed. Two independent
   gates, both closed by default.
3. No command is ever held against a mechanical stop. Soft limits,
   clamping on arrival, and auto release on the self locking joint.
4. Every motion is ramped, in sim and on hardware, with the same numbers.
5. The 6 V rail is energised only with the arms compact and resting, and
   the switch that feeds it (today the ATX rear switch) is the real
   emergency stop.
6. The cloud reasoner proposes; the armed driver disposes.
7. A safety rule without a test is a rule that will be refactored away.

---

## 7. Risks, named before they bite

| Risk | Impact | Mitigation |
|---|---|---|
| **MG996R endurance over seven hours** (gripper stall, limit cycle at the vertical, base near stall) | The run dies at hour two, on camera | Grip at the calibrated contact angle with foam pads, shoulders preloaded, rest pose off the vertical, stations near the vertical, the one hour soak of v0.4 as a gate with temperatures measured |
| Payload is about 330 g at 30 cm | Limits every task | Parts under 50 g, compact poses, short holds |
| Reach is a 240 mm lane per arm, under unverified signs | The deck is cut for the wrong geometry | Cut only after the sign check; cardboard mock first |
| No encoders, no force sensing | Open loop everything | Pockets constrain the parts; vision verifies every cycle |
| Backlash in aluminum joints | Repeatability of a few millimeters | Chamfered pockets, 6 mm of play, the real repeatability number reported |
| ER 2 is a preview API | The id disappears mid-project | Model id is configuration; the adapter is thin; the cell runs without it |
| Cloud dependency in a demo | A network hiccup ruins the run | Unverified cycles continue and are counted as such; retries with backoff |
| Scope creep past v1.0 | The project never closes | The closure rule is in the README and in CLAUDE.md: at v1.0 it ends |

---

## 8. Out of scope, and why

- **The learning track** (RL-ready URDF, MJCF export, actuator
  identification, L1 to L5) was removed on 2026-10-03. It competed for the
  same evenings as the demo and it was not needed for any gate above. What
  stays: `sim/pendulum_oracle.xml` and `test_sim_oracle.py`, a cheap check
  that the MuJoCo wheel CI installs still agrees with pencil-and-paper
  physics; and the inertia values in the xacro, which cost nothing to keep.
- **MoveIt** as the IK path. The chain is planar; a 20-line analytic IK
  with the measured limits (section 2) replaces a planner, a broken SRDF
  and a package that never existed. `srdf/soma.srdf` and
  `config/kinematics.yaml` stay as reference files, off the ladder.
- **Bimanual handoff.** Physically impossible on this bench: the planes
  are 124.6 mm apart and no joint moves laterally.
- **Vision teleop by mimicry (BlazePose).** Post-1.0 backlog, as before.
- **Deferred past v1.0**: a rotary index table to move parts between the
  two lanes (the honest way to get a flow between arms), and a gravity
  ramp that returns parts to a pick point.

---

## 9. Immediate next actions

1. Arming ritual, `soma_sign_check` on the right arm, then the wave with
   `--step`, filmed. Tag `v0.2` once the left arm is checked too.
2. Measure the shoulder oscillation threshold in microseconds; fit the
   preload.
3. Rerun `scripts/workspace_map.py` with the verified signs; cardboard
   deck; order the laser cut.
4. Buy six wooden blocks of about 40 mm and a lamp; engrave or mark the
   ids.
5. AI Studio auth key, billing, the Developer Program credit; run
   `scripts/er2_probe.py` on one frame of the bench and write the latency
   and token numbers into this file.

---

## 10. Capture

Raw material is unrecoverable. Engineering that was not captured did not
happen as far as anyone outside this bench is concerned, and every
milestone in section 4 is supposed to ship with a video.

**This is not a new rule.** A capture rule already existed in the rover
project and produced exactly one tracked image in eight months. A rule
fails when it has no defined moment, no defined destination and no cost
ceiling. This section supplies all three.

### 10.1 The session checklist

Four items. Three minutes total. If it takes longer, the checklist is
wrong, not the engineer.

```
BEFORE the 6 V rail comes up (or before the first command of the session):
  [ ] 1 still, wide: the bench exactly as it is right now.
  [ ] 1 still, close: the thing this session is about to change.

AT SESSION CLOSE, before anything is put away:
  [ ] 1 clip, 20 s vertical: the most surprising thing that happened today.
      If nothing surprised you, film what failed. If nothing failed, film
      the terminal showing the number that changed.
  [ ] 1 line appended to the log (docs/log.md):
      capture: <folder>/<file> - <what it shows>
```

Rules that keep this survivable:

- Phone, vertical, no edit, no narration, no lighting rig. Raw only.
- Destination: `~/Robotics/capturas/YYYY-MM-DD/`. Never a git repo, never
  deleted. Video never enters a repository; only a still that is already
  used inside a document does.
- Security sweep before anything leaves the phone: no tokens, no keys, no
  dashboard URLs, no full floor plan of a home, no client material.
- The two "before" shots happen while the rail is still off, which is the
  only window a tired person actually has.

### 10.2 One-way doors

A step that cannot be undone does not feel different at 11pm, so it gets
marked in the plan itself. Every step tagged **FILM BEFORE** in section 4
destroys a state that can never be filmed again:

| Step | Where | What it destroys |
|---|---|---|
| First joint moved through the ROS driver | v0.2 step 2 | The unproven signs, the arm that has only ever moved from a slider |
| Cut the deck and fix the camera to the column | v0.3 steps 1 and 2 | The era of a bare plate under loose-hanging arms |
| Definitive harness (JST, 16 AWG) | `docs/wiring.md` | The provisional wiring |
| Diagnose the left wrist | open issue | An unresolved fault, on camera, undiagnosed |

### 10.3 Incidents carry a capture

`docs/safety.md` keeps an incident log where every row pairs a failure with
the change it forced, and it closes with "an incident with no change is an
incident that will happen again". The same applies to evidence: an incident
with no footage is an incident that gets described instead of shown, and
description does not survive a phone screen with the sound off. New rows in
that table should name their capture when one exists.
