#!/usr/bin/env bash
# End to end smoke test for SOMA. Runs inside a ROS 2 Humble environment
# (see docker/Dockerfile) and is the same script CI runs.
#
# It checks, in order:
#   1. the workspace builds
#   2. the unit tests pass (channel map, safety rules, golden rule)
#   3. all three xacro models process and are valid URDF
#   4. end to end: the driver in MOCK mode raises the torso and TF agrees
#
# Nothing here can move a motor: the driver boots on the mock backend and
# is never armed.
set -e
source /opt/ros/humble/setup.bash

WS="${WS:-/ros2_ws}"
OUT="${OUT:-/tmp}"

echo '=== 1/4 colcon build ==='
cd "$WS"
colcon build --symlink-install
source install/setup.bash

echo '=== 2/4 unit tests (servo map, safety rules, golden rule) ==='
python3 -m pytest src/soma-arms/soma_driver/test/ -q

echo '=== 3/4 xacro and URDF validation of the three models ==='
xacro_run() {
  if command -v xacro >/dev/null; then
    xacro "$1"
  else
    python3 -c 'import sys, xacro; print(xacro.process_file(sys.argv[1]).toxml())' "$1"
  fi
}
SHARE="$(ros2 pkg prefix soma_description)/share/soma_description/urdf"
for m in single_arm soma_bench soma_bench_sim; do
  xacro_run "${SHARE}/${m}.urdf.xacro" > "${OUT}/${m}.urdf"
  if command -v check_urdf >/dev/null; then
    check_urdf "${OUT}/${m}.urdf" > /dev/null
    echo "  ${m}: $(grep -o '<joint' "${OUT}/${m}.urdf" | wc -l) joints, check_urdf OK"
  else
    echo "  ${m}: $(grep -o '<joint' "${OUT}/${m}.urdf" | wc -l) joints (check_urdf not installed)"
  fi
done

echo '=== 4/4 end to end: robot_state_publisher + soma_driver (mock) + TF ==='
# the URDF goes in through a params file: passing it with -p on the command
# line breaks the rcl parser
python3 - "${OUT}" <<'EOF'
import sys
import yaml
out = sys.argv[1]
urdf = open(f'{out}/soma_bench.urdf').read()
yaml.safe_dump(
    {'robot_state_publisher': {'ros__parameters': {'robot_description': urdf}}},
    open(f'{out}/rsp_params.yaml', 'w'))
EOF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "${OUT}/rsp_params.yaml" &
RSP=$!
ros2 run soma_driver arm_controller &
DRV=$!
sleep 5

read_z() {
  timeout 15 ros2 run tf2_ros tf2_echo world left_arm_tool0 2>/dev/null \
    | grep -m1 'Translation' | sed 's/.*, \([0-9.]*\)\].*/\1/'
}

echo '--- initial pose (torso at the bottom soft limit) ---'
Z0=$(read_z)
echo "  tool0 z initial = ${Z0} (expected ~0.662)"

echo '--- command: torso to 0.14 m, clamped to 0.135, L16 ramp is 6.5 s ---'
ros2 topic pub --once /soma/command sensor_msgs/msg/JointState \
  "{name: [torso_lift_joint], position: [0.14]}"
sleep 9

Z1=$(read_z)
echo "  tool0 z final = ${Z1} (expected ~0.792)"

kill $RSP $DRV 2>/dev/null || true

python3 - "$Z0" "$Z1" <<'EOF'
import sys
z0, z1 = float(sys.argv[1]), float(sys.argv[2])
assert abs(z0 - 0.662) < 0.005, f'initial z {z0} != 0.662'
assert abs(z1 - 0.792) < 0.005, f'final z {z1} != 0.792'
# 140 mm of physical stroke minus the two 5 mm safety margins
assert abs((z1 - z0) - 0.130) < 0.005, 'travel is not 130 mm'
print(f'\nSMOKE TEST OK: the torso rose {1000*(z1-z0):.1f} mm, confirmed by TF')
EOF
