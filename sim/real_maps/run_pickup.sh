#!/usr/bin/env bash
# Desktop-only; creates and cleans up ONLY its own uniquely named isolated container.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
[[ "$(uname -m)" == x86_64 ]] || { echo 'REFUSED: desktop x86_64 simulation only' >&2; exit 2; }
export PACKAGU_SIM_CONTAINER="packagu_sim_pickup_$$"
export PACKAGU_SIM_NETWORK=packagu_sim_pickup_net
export PACKAGU_SIM_DOMAIN=79
cleanup() {
  bash sim/real_maps/sim_container.sh down
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash sim/real_maps/sim_container.sh up --gui # X11 is needed for rendered camera, also without Gazebo GUI.
bash sim/real_maps/sim_container.sh exec python3 -m compileall -q sim/real_maps/pickup
bash sim/real_maps/sim_container.sh exec python3 -u sim/real_maps/pickup/test_pickup.py
bash sim/real_maps/sim_container.sh exec python3 -u sim/real_maps/pickup/run.py "$@"
