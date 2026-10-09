# Bench guide

Validated recipes for working on SOMA: what to run, in what order, and the
gotchas that already cost a session. Nothing here moves a motor unless it says
so explicitly, and the parts that do are gated by
[safety.md](safety.md).

---

## 1. Fast loop, on a laptop

`servo_map.py` is pure Python. No ROS, no hardware, no Docker.

```bash
python3 -m venv venv && venv/bin/pip install pytest
venv/bin/python -m pytest soma_driver/test -q
```

This is the loop to stay in while changing the channel map, the limits or the
safety rules. The ROS node tests skip automatically when `rclpy` is missing.

## 2. Full validation, in ROS 2 Humble

Build, unit tests, all three xacro models through `check_urdf`, and an end to
end run where the mock driver bends the left elbow while TF confirms the
fingertip rising 311 mm:

```bash
bash scripts/smoke_test.sh
```

Same script CI runs. Inside the image from `docker/Dockerfile`, or any Humble
environment with `xacro`, `robot_state_publisher`, `tf2_ros` and
`liburdfdom-tools`.

**Gotcha, already paid for**: the URDF goes to `robot_state_publisher` through
a **`--params-file` YAML**. Passing it with `-p` on the command line breaks the
rcl parser. The script does it the right way.

## 3. RViz in a browser

The validated path for seeing the model without fighting X11 forwarding:

```bash
docker run -d --name soma_vnc -p 6080:80 --shm-size=512m \
  -v /ABSOLUTE/PATH/TO/ros2_ws:/ros2_ws \
  tiryoh/ros2-desktop-vnc:humble
```

Then, once per container:

```bash
docker exec soma_vnc bash -c "apt-get update -qq && \
  apt-get install -y -qq ros-humble-xacro ros-humble-joint-state-publisher-gui"
docker exec soma_vnc bash -c "source /opt/ros/humble/setup.bash && \
  cd /ros2_ws && colcon build --symlink-install"
```

Launch it **as the `ubuntu` user**, who owns the X session:

```bash
docker exec -d -u ubuntu soma_vnc bash -c "export DISPLAY=:1 HOME=/home/ubuntu && \
  source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
  ros2 launch soma_description display.launch.py"
```

Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>, then drag
`torso_lift_joint` and watch the column extend.

Three gotchas, all of them learned the hard way:

- Launch as `-u ubuntu` with `DISPLAY=:1` and `HOME=/home/ubuntu`. As root the
  window never appears.
- Mount with an **absolute** path.
- `--shm-size=512m`, or RViz dies quietly.

## 4. On the Pi

I2C lives outside Docker, so bare metal `smbus2` is the tool for bring-up.

```bash
ssh <your-pi>          # the ROS user
i2cdetect -y 1         # must show 0x40
```

Two bring-up scripts live in `scripts/pi/` in this repository. Copy them to
the Pi (`scp scripts/pi/*.py <your-pi>:`) and run them there with bare metal
`smbus2`:

| Script | What it does | Can it move a motor |
|---|---|---|
| `scripts/pi/pca9685_check.py` | sets 50 Hz, echoes the registers of all 13 channels, leaves every output in FULL_OFF | no, and it is meant to run with the servo rail OFF |
| `scripts/pi/servo_discovery.py` | identifies channels one at a time with 60 ms bursts | **yes**, needs the rail live and prior confirmation |

---

## 5. Calibration

### 5.1 Link lengths, the v0.1 deliverable

Every length in `soma_arm.xacro` and `soma_torso.xacro` is marked
`[calibrate]`: they are estimates from the manual photographs. Measuring them
is a **prerequisite for v0.1**, and it needs no power at all.

Per arm, in mm, with calipers:

| Property | Today (estimated) | What to measure |
|---|---|---|
| `base_lx` / `base_ly` / `base_lz` | 95 / 95 / 62 | the base box |
| `turntable_r` / `turntable_h` | 42 / 12 | the disc on the slew bearing |
| `shoulder_off_z` | 45 | disc surface to the shoulder axis |
| `upper_len` | 120 | shoulder axis to elbow axis |
| `fore_len` | 90 | elbow axis to wrist pitch axis |
| `wrist_len` | 55 | wrist pitch axis to the roll flange |
| `gripper_base_h` | 45 | roll flange to the finger axis |
| `finger_len` | 75 | the geared finger |

Also still open, and not a caliper job:

- **real mass per arm** on a scale, which feeds the inertias and the lifted
  mass budget,
- the **base bolt pattern**, needed to design the torso plate.

Correct the xacro properties **once** and everything downstream follows: FK,
RViz, planning. Then run `scripts/check_model_driver_sync.py` to confirm the
URDF and the driver still agree.

### 5.2 Servo calibration, with power

Requires explicit confirmation before every session. Per servo, one at a time:

- **horn offset**: which mechanical zero the horn was splined at,
- **sign**: whether positive joint angle turns the way the URDF says,
- **safe range**: the real travel before the frame binds, which may be
  narrower than +/- 90 degrees.

Rules while doing it: **one servo at a time**, short bursts, never a sustained
command, and never approach a mechanical stop.

### 5.3 L16 real end stops

The 5 to 135 mm soft band is derived from the nominal 140 mm stroke. The real
stops in microseconds have not been measured yet.

Procedure, and it is deliberately slow:

1. Move in **25 us steps**, measuring the rod with a ruler at each step.
2. Stop as soon as it **stops advancing**. That is a stop, not a target.
3. Back off **5 mm** from there, and take that as the soft limit.
4. Update the anchors in `SERVO_MAP` and the matching limits in
   `soma_torso.xacro`, then let the sync check confirm they agree.

**Never hold a command while probing.** A sustained command against a stop is
exactly what wedged this actuator on 2026-07-22.

### 5.4 If the L16 gets wedged anyway

It has happened once. Symptoms: retract gives silence, extend gives a twitch
with no travel, and neither a fresh battery nor a different channel changes
anything.

Fix: send an **extend** command and apply **gentle manual traction** on the
rod. The hand breaks the wedge. Afterwards it takes absolute positions
normally again.

---

## 6. On the Jetson

The bench computer since August: a Jetson Orin Nano Super at
`192.168.1.2` on WiFi, user `jetson`, the repository at
`/home/jetson/soma-arms`, the PCA9685 boards on header bus `i2c-7`.
JetPack 7.2 has no ROS 2 Humble, so the driver runs in the `soma_driver`
container of `docker-compose.jetson.yml`, always from the repo root.

```bash
ssh jetson@192.168.1.2
cd /home/jetson/soma-arms
i2cdetect -y -r 7      # 0x40 and 0x43 (0x70 is the all-call address)
```

`0x41` no longer appears: the INA3221 is off the bus since 2026-10-01.
A missing `0x40` or `0x43` is a harness fault: stop there.

The driver container, which boots MOCK and DISARMED every time:

```bash
docker compose -f docker-compose.jetson.yml up -d
docker compose -f docker-compose.jetson.yml logs -f      # Ctrl-C leaves it running
docker compose -f docker-compose.jetson.yml down
```

New code reaches the driver by pulling and recreating the container,
which rebuilds the workspace on boot (expect about a minute before the
log says "MOCK backend, DISARMED"):

```bash
git pull --ff-only
docker compose -f docker-compose.jetson.yml up -d --force-recreate
```

### The `soma` helper

Every ROS command runs inside the container with both setup files
sourced. One shell function per SSH session saves the boilerplate (or
add it to `~/.bashrc` on the Jetson):

```bash
soma() { docker exec -it soma_driver bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec "$@"' soma "$@"; }
```

The arguments travel as positional parameters, so quoted YAML such as
`"{data: true}"` arrives as one word. (Splicing `$*` into the `bash -c`
string instead would split it in two.) Check that once per session; it
must print `[{data: true}]` on one line:

```bash
soma printf '[%s]\n' "{data: true}"
```

Then:

```bash
soma ros2 topic echo /joint_states --once
soma ros2 param get /soma_driver allow_real        # gate 1: False unless relaunched
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: true}"    # gate 2
soma ros2 service call /soma/arm std_srvs/srv/SetBool "{data: false}"   # disarm
soma ros2 run soma_driver soma_sign_check --arm right
soma ros2 run soma_driver soma_primitives wave --step
```

Gate 1 (`allow_real`) is a launch parameter, opened by relaunching the
container and closed by relaunching it without the variable. See the
header of `docker-compose.jetson.yml` and [safety.md](safety.md) rule 2:

```bash
ALLOW_REAL=true docker compose -f docker-compose.jetson.yml up -d --force-recreate   # open
docker compose -f docker-compose.jetson.yml up -d --force-recreate                   # close
```

### Can it move a motor

| Command | Can it move a motor |
|---|---|
| `i2cdetect -y -r 7`, `compose up`, `logs`, `down` | no |
| `soma ros2 topic echo ...`, `soma ros2 param get ...` | no |
| `ALLOW_REAL=true ... up -d --force-recreate` (gate 1) | no by itself: the driver boots MOCK and DISARMED, but the next gate is live |
| `soma ros2 service call /soma/arm ... "{data: true}"` (gate 2) | **yes**: every servo starts receiving the pulse of the pose the driver holds (rule 2 of [safety.md](safety.md)) |
| `soma ros2 run soma_driver soma_sign_check`, `soma_primitives ...` | only while ARMED; disarmed they move `/joint_states` and nothing else |
| `soma ros2 service call /soma/arm ... "{data: false}"` | no drive: the signal is cut and the servos go limp, so an arm that is not hanging falls to the hang |
| the ATX rear switch | it is the emergency stop: 12 V, both UBECs and the 6 V rail go down, the Jetson stays up |

Arming is Andres's call, out loud, every session. Nothing in this table
changes that.
