#!/usr/bin/env bash
# Source this file in each shell used for the hardware pipeline.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "Open a fresh shell: this environment requires ROS 2 Humble." >&2
  return 1
fi
export VIO_WS
VIO_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$VIO_WS/install_vio/setup.bash" ]]; then
  echo "Run $VIO_WS/hw290_stereo/setup_hw290_openvins.sh first." >&2
  return 1
fi
source /opt/ros/humble/setup.bash || return
source "$VIO_WS/install_vio/setup.bash" || return
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export CAL_TOOLS="${CAL_TOOLS:-$HOME/vio_calibration_tools}"
export ROSBAGS_CONVERT="$CAL_TOOLS/rosbags-venv/bin/rosbags-convert"
export KALIBR_IMAGE=kalibr-hw290:doc-20261009
export ALLAN_IMAGE=kalibr-hw290:allan-20261009
case ":$PATH:" in
  *":$HOME/.local/bin:"*) ;;
  *) export PATH="$HOME/.local/bin:$PATH" ;;
esac
# Keep the conversion venv off PATH so its Python cannot replace ROS Python.
