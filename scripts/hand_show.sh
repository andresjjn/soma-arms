#!/usr/bin/env bash
# One take of the hand show on the Jetson: check, arm on Andres's word, run.
#
#   bash scripts/hand_show.sh left --present --lift 0.785   # the show at 45 deg
#   bash scripts/hand_show.sh left                          # gripper only
#
# Run it ON the Jetson, from the repository, in a terminal: it asks before
# arming. Everything after the arm goes to the bridge (--present, --lift,
# --roll, --present-max); the tracker runs with its defaults and draws the
# view on http://localhost:8090.
#
# Arming stays a person's act (CLAUDE.md rules 1, 2 and 4). The script arms
# only when every arm joint of both arms reads 0.0 on /joint_states (the
# first armed tick sends the believed pose with no ramp), after printing
# the rule 4 checklist, and only if "arm" is typed at the prompt. Gate 1 is
# not touched. hand_gripper.py itself still cannot arm; a lost hand makes
# the bridge walk home and disarm, which is why each take starts here.
set -euo pipefail

arm="${1:-}"
case "$arm" in
  left|right) shift ;;
  *) echo "usage: $0 left|right [--present] [--lift RAD] [--roll RAD] [--present-max S]" >&2
     exit 2 ;;
esac
repo="$(cd "$(dirname "$0")/.." && pwd)"
py="${SOMA_DEPTHAI_PY:-$HOME/venvs/depthai3/bin/python}"
bridge_py=/ros2_ws/src/soma-arms/scripts/hand_gripper.py
# Arguments travel as positional parameters (as in docs/bench.md, soma()).
inside='. /opt/ros/humble/setup.bash && . /ros2_ws/install/setup.bash && exec "$@"'
in_driver() { docker exec soma_driver bash -c "$inside" soma "$@"; }
last_state() { docker logs soma_driver 2>&1 | grep 'SOMA: ' | tail -n 1 || true; }

if ! docker inspect soma_driver >/dev/null 2>&1; then
  echo "REFUSED: no soma_driver container (bash scripts/jetson_driver.sh open)" >&2
  exit 3
fi

state="$(last_state)"
if [[ "$state" == *"SOMA: ARMED"* || "$state" == *"already ARMED"* ]]; then
  echo "The driver is already ARMED."
else
  if ! in_driver python3 "$bridge_py" --home-check; then
    echo "REFUSED, nothing armed: bring every joint to 0.0 first (soma_primitives home)." >&2
    exit 3
  fi
  echo "Rule 4: both arms hanging at rest, nothing under or around them, the ATX on."
  printf 'Type arm and ENTER to arm the driver (anything else cancels): '
  read -r answer || answer=''
  if [ "$answer" != arm ]; then
    echo "Cancelled: nothing armed."
    exit 1
  fi
  reply="$(in_driver ros2 service call /soma/arm std_srvs/srv/SetBool '{data: true}' 2>&1 || true)"
  if [[ "$reply" != *"success=True"* ]]; then
    echo "REFUSED by the driver: $reply" >&2
    exit 3
  fi
  echo "ARMED."
fi

# Ctrl-C reaches the tracker and the bridge; this script stays to report.
trap 'true' INT
"$py" "$repo/scripts/hand_gripper.py" --emit --show --log /tmp/soma_cam/hand.log \
  | docker exec -i soma_driver bash -c "$inside" soma python3 "$bridge_py" --bridge "$arm" "$@" \
  || true
state="$(last_state)"
echo "Driver now: ${state#*SOMA: } (after a Ctrl-C the bridge may still be walking home for a few seconds)"
