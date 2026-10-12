#!/usr/bin/env bash
# Recreate the soma_driver container on the Jetson, with gate 1 open or closed.
#
#   bash scripts/jetson_driver.sh closed    # allow_real:=false, the default state
#   bash scripts/jetson_driver.sh open      # allow_real:=true, gate 1 by relaunch
#
# Run it ON the Jetson, from the repository (today
# /home/jetson/Projects/soma/soma-arms). The Jetson has no `docker compose`
# (checked 2026-10-12), so this is docker-compose.jetson.yml written as one
# `docker run`, with the same flags. Either way the driver boots MOCK and
# DISARMED; gate 2 stays the /soma/arm call, made by Andres.
#
# Recreating the container also rebuilds the workspace on boot: it is how new
# code reaches the driver after a `git pull`.
#
# It refuses while the driver is ARMED: removing an armed driver kills it
# before it can cut the signal, and the boards would keep holding the last
# pulse with nothing in charge. Disarm first (/soma/arm false).
set -euo pipefail

case "${1:-}" in
  closed) allow=false ;;
  open)   allow=true ;;
  *) echo "usage: $0 closed|open   (gate 1, the allow_real launch parameter)" >&2
     exit 2 ;;
esac
repo="$(cd "$(dirname "$0")/.." && pwd)"

if docker inspect soma_driver >/dev/null 2>&1; then
  last="$(docker logs soma_driver 2>&1 | grep 'SOMA: ' | tail -n 1 || true)"
  if [[ "$last" == *"SOMA: ARMED"* || "$last" == *"SOMA: already ARMED"* ]]; then
    echo "REFUSED: the driver is ARMED. Disarm first (/soma/arm false), then rerun." >&2
    exit 3
  fi
  docker stop -t 10 soma_driver >/dev/null
  docker rm soma_driver >/dev/null
fi

docker run -d --name soma_driver --network host --ipc host \
  --restart unless-stopped --memory 1g \
  --device /dev/i2c-7:/dev/i2c-7 \
  -v "$repo":/ros2_ws/src/soma-arms -w /ros2_ws soma_driver \
  bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch soma_driver driver.launch.py allow_real:=$allow" >/dev/null

echo "soma_driver recreated with gate 1 ${1} (allow_real:=$allow). It boots MOCK and DISARMED; wait for 'SOMA driver up' in: docker logs -f soma_driver"
