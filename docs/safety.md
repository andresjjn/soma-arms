# SOMA safety model

SOMA is about 1.8 kg of aluminum on twelve servos with no position feedback,
plus a linear actuator that can push 100 N. The rules below are not
aspirations. Each one is enforced by code, and each one is covered by a test.

## The rules

### 1. Never move a motor without explicit prior confirmation

No script, no agent and no automation energises a servo because it seemed
like the next reasonable step. Motion happens when the operator has said so,
in advance, for that specific action.

**Enforced by two independent gates**, and both must be open:

| Gate | What it is | Default |
|---|---|---|
| `allow_real` | a launch parameter, decided when the node starts | `false` |
| `/soma/arm` | a service call, made deliberately while it runs | disarmed |

`RealPca9685.__init__` raises `PermissionError` unless it is constructed with
`armed=True`, so even importing and instantiating it by accident cannot emit a
pulse. Tested by `test_real_backend_without_arming_is_impossible`.

### 2. The node boots on the mock backend and disarmed

`ArmController` always constructs `MockPca9685` first. There is no
configuration that starts it live. The mock records the pulses that would have
been sent, publishes `/joint_states` exactly as the real backend would, and
touches no hardware, so RViz and the whole pipeline can be developed and
demonstrated with zero risk.

Arming is a transition, never an initial state:

```bash
# 1. start it (safe, mock, disarmed) but permitted to arm later
ros2 launch soma_driver driver.launch.py allow_real:=true

# 2. only after Andres has confirmed, out loud, for this specific run:
ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"

# 3. cut the signal at any time
ros2 service call /soma/arm std_srvs/srv/SetBool "{data: false}"
```

On the Jetson the driver lives in the `soma_driver` container, so gate 1 is
a relaunch and gate 2 goes through `docker exec` (the `soma` helper is in
[bench.md](bench.md), section 6):

```bash
# 1. relaunch with gate 1 open; it still boots MOCK and DISARMED
ALLOW_REAL=true docker compose -f docker-compose.jetson.yml up -d --force-recreate

# 2. only after Andres has confirmed, out loud, for this specific run:
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"

# 3. cut the signal at any time
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: false}"

# 4. end of session: close gate 1 with a relaunch WITHOUT the variable
docker compose -f docker-compose.jetson.yml up -d --force-recreate
```

`ros2 param set /soma_driver allow_real true` opens gate 1 without a
restart and is the documented alternative. Step 4 matters: the container
restarts with the configuration it was created with, so one created with
`ALLOW_REAL=true` comes back after a reboot with gate 1 still open.

Calling `/soma/arm` on a node started without `allow_real:=true` is **refused**
and logged. Disarming cuts every channel, drops back to the mock, and always
succeeds.

**What arming does to the metal.** An earlier version of this page said the
arming call never causes motion. It does. Arming swaps in the real boards
with every output in FULL_OFF, and the next 50 Hz tick sends every channel
the pulse of the pose the driver *believes* it holds, all at once. After a
clean boot, or a walk that came back home, that pose is the calibrated zero
of every joint. The ramp cannot shape this first move: the driver is open
loop and believes each joint is already there, so a servo that is
physically elsewhere goes to its pulse at its own full speed (roughly
6 rad/s per [hardware.md](hardware.md), more than twice the 2.5 rad/s the
ramp allows). With the arms hanging at rest the four
gravity joints of each arm already sit near zero and little or nothing
visible happens; the wrist rolls and the grippers stay wherever they were
left, so they can snap to zero. Before every arming: hands clear of both
grippers, and `/joint_states` reads 0.0 on every arm joint.

Tested by `test_boots_on_mock_and_disarmed`,
`test_arming_is_refused_without_allow_real`,
`test_disarm_always_succeeds_and_returns_to_mock`, and, for what the first
armed tick sends, `TestWhatArmingSends` in `test_node_safety.py`.

### 3. Never hold a command against a physical stop

This rule was written in blood, or at least in a wedged lead screw. On
2026-07-22 a retract command held for about 8 seconds against an already
retracted L16 jammed the screw against its stop. It had to be freed by hand.

Three mechanisms, all active at once:

- **Soft limits.** The lift is commandable over **5 to 135 mm**, never
  0 to 140. Saturation happens in `ServoSpec.command_to_us`, so it applies to
  every path into the hardware. The URDF carries the same limits, so planners
  cannot aim at a stop either, and CI compares the two
  (`scripts/check_model_driver_sync.py`).
- **Clamping on arrival.** Incoming targets are clamped when they are
  received, so `/joint_states` never reports a pose the hardware is not
  allowed to reach.
- **Auto release.** After 0.5 s settled on target, the driver cuts the PWM on
  the lift channel. The L16 lead screw is self locking and holds 46 N with no
  power, so the load stays put and there is no signal left to grind.

**Auto release applies only to self locking joints.** Arm servos hold their
position with active torque; cut their signal and the arm falls. That is why
`RELEASE_WHEN_SETTLED` contains exactly one joint, and why a test asserts
exactly that.

### 4. Every motion is ramped

`rate_limit` moves each joint toward its target at no more than its own
maximum rate: 2.5 rad/s for arm servos, well below what an MG996R could do,
and 0.020 m/s for the lift, which is what the L16 actually does. No command
jumps a servo straight to a new position. The ramp works on the pose the
driver believes, though, so it has two blind moments it cannot cover: the
twitch when the 6 V rail comes up (rule 5) and the first pulse after
arming (rule 2).

### 5. Energise V+ only with the arms compact and resting

MG996R clones **twitch when the rail comes up**. An arm extended in mid air at
that moment can slam itself, its gripper, or whatever is under it.

Before the 6 V rail is switched on: both arms folded into a compact pose,
resting on the bench, nothing fragile underneath. Every time, including the
times it feels unnecessary.

This one lives outside the software. No parameter, no service call and no test
can enforce it, which is exactly why it is written down here.

### 6. The harness is part of the safety system

In the first real session, every unexplained fault was a connector:

- a dupont jumper broken **inside its insulation** dropped the I2C bus four
  times, the last time unrecoverably,
- loose servo connectors produced "ghost" servos twitching at power-up.

`retry_i2c()` is a net under isolated glitches. It is **not** a fix for a bad
connection, and it did not save that session: the last drop failed after 20
retries. New cables did.

Required for anything beyond a supervised bench test: **JST with a positive
lock for I2C**, silicone retainers on servo connectors, and **16 AWG or
thicker** for V+. Details in [wiring.md](wiring.md).

A control loop cannot be safer than the wire it runs on.

## What the safety model does not cover

Being honest about this matters more than the list above.

- **No position feedback.** RC servos report nothing. On startup the driver
  assumes every joint is at zero, and it can be wrong. The ramp, the soft
  limits and the arming gates exist precisely because the true pose is
  unknown.
- **No force sensing.** Nothing detects a collision or a stalled servo. A
  stalled MG996R draws heavy current and gets hot.
- **No emergency stop in software worth trusting.** `/soma/arm false` cuts the
  signal, but it needs a working node, a working bus and a working Jetson.
  **The real emergency stop is the ATX rear switch.** It cuts the mains to
  the supply that feeds both UBECs; once its 12 V output collapses the UBECs
  drop out and the 6 V rail goes with them, on the order of tens of
  milliseconds under load (an estimate, never measured on this supply: the
  ATX spec holds the output up at least 16 ms after the mains go, and a
  light load stretches that). The Jetson, on its own adapter, stays up and
  keeps its logs. Keep that switch in reach, and a hand near it, whenever
  the arms are armed. The driver cannot see the rail drop (the boards'
  logic runs on the Jetson's 3.3 V), so after an ATX stop it is still
  ARMED: disarm before the ATX ever comes back on, or every servo is
  driven the moment the rail returns. Pulling the XT60 is **not** a stop:
  it takes a firm two-handed pull on a connector built to stay mated, it
  breaks a DC load in the air (the arc pits the contacts), and it yanks
  the harness that rule 6 calls part of the safety system.

## Incident log

| Date | What happened | What changed |
|---|---|---|
| 2026-07-22 | L16 wedged against its internal stop by an 8 s held retract command, already fully retracted. Freed by hand with an extend command plus gentle traction | soft limits 5 to 135 mm, clamping on arrival, auto release after 0.5 s settled |
| 2026-07-22 | I2C dropped four times with `OSError 121`, the last unrecoverable after 20 retries. Cause: a dupont jumper broken inside its insulation | `retry_i2c` (3 attempts, 5 ms) as a net, and new short cables as the actual fix. JST with lock specified for the permanent harness |
| 2026-07-22 | "Ghost" servos twitching at power-up | traced to loose connectors. Silicone retainers specified, and the compact resting pose rule |
| 2026-07-22 | Channel 0 suspected during the L16 fault, never cleared | L16 moved to channel 3, channel 0 left out of service |
| 2026-09-01 | Hot plugging the servo branch on the shared ATX sank the 12 V rail and hard-shut the Jetson (inrush of two UBECs plus twelve servos; the supply is the common root of the star) | power domains split: the Jetson on its own adapter, the ATX for servos only; nothing is ever hot plugged onto the servo rail |
| 2026-10-01 | The harness was reported as "channels 1 to 12" and the map followed it: a dead right arm, a left arm driven by neighbouring joints' pulses. The boards have no silkscreen and the header was counted from the wrong end | map restored to the proven layout (15 to 10 on `0x40`, 9 to 4 on `0x43`); rule in `docs/wiring.md`: a channel number is proven by moving a servo, never by counting pins |
| 2026-10-01 | A 3.3 V logic glitch put both PCA9685 back to power-on defaults (asleep, 200 Hz) while software believed it was driving pulses | the driver re-checks MODE1 and PRESCALE while armed and disarms on a reset; arming re-runs the init (`test_board_reset.py`) |

New incidents belong in this table, with the change that came out of them. An
incident with no change is an incident that will happen again. Note that two
of the four rows above were fixed with cable, not with code.
