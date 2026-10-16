#!/usr/bin/env bash
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ROS_DOMAIN_ID="${HW290_REMOTE_DOMAIN_ID:-48}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export HW290_DDS_CONFIG="$ROOT/network/sophon_board.xml"
exec "$ROOT/run_hw290_container.sh" "$@"
