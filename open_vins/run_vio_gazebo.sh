#!/usr/bin/env bash
set -Eeuo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install_vio/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1

auto=true
gui=true
rviz=true
vins=true
mode=circle
stereo=false
drive_pid=""

usage() {
  cat <<'EOF'
Usage: ./run_vio_gazebo.sh [options]

Options:
  --auto       Automatic rover motion (default)
  --teleop     Keyboard teleoperation instead of automatic motion
  --headless   Disable Gazebo and RViz2 windows
  --no-rviz    Disable RViz2 only
  --no-vio     Start the simulator without OpenVINS
  --stereo     Use both simulated cameras
  --mono       Use the left camera plus IMU (default)
  --square     Use the square drive pattern
  -h, --help   Show this help
EOF
}

while (($#)); do
  case "$1" in
    --auto) auto=true ;;
    --teleop) auto=false ;;
    --headless) gui=false; rviz=false ;;
    --no-rviz) rviz=false ;;
    --no-vio) vins=false ;;
    --stereo) stereo=true ;;
    --mono) stereo=false ;;
    --square) mode=square ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

config="$repo_dir/ov_rover_sim/config/rover_stereo/estimator_config.yaml"
[[ -f "$config" ]] || { echo "Missing OpenVINS config: $config" >&2; exit 1; }

echo "Starting camera plus IMU VIO simulation"
max_cameras=1
if [[ "$stereo" == true ]]; then max_cameras=2; fi
auto_launch="$auto"
# Give stereo OpenVINS a stationary interval for inertial initialization, then
# start automatic motion after both camera streams are being consumed.
if [[ "$stereo" == true && "$auto" == true ]]; then auto_launch=false; fi
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID auto=$auto mode=$mode gui=$gui rviz=$rviz vio=$vins cameras=$max_cameras"

ros2 launch ov_rover_sim rover_sim.launch.py \
  auto:="$auto_launch" mode:="$mode" gui:="$gui" rviz:="$rviz" > /tmp/vio_gazebo_launch.log 2>&1 &
launch_pid=$!
vio_pid=""

cleanup() {
  trap - INT TERM EXIT
  kill -INT "$launch_pid" 2>/dev/null || true
  if [[ -n "$vio_pid" ]]; then
    kill -INT "$vio_pid" 2>/dev/null || true
    wait "$vio_pid" 2>/dev/null || true
  fi
  if [[ -n "$drive_pid" ]]; then
    kill -INT "$drive_pid" 2>/dev/null || true
    wait "$drive_pid" 2>/dev/null || true
  fi
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

if [[ "$vins" == true ]]; then
  sleep 12
  ros2 run ov_msckf run_subscribe_msckf "$config" \
    --ros-args -r __ns:=/ov_msckf -p use_sim_time:=true \
    -p use_stereo:="$stereo" -p max_cameras:="$max_cameras" \
    > /tmp/ov_msckf_vio.log 2>&1 &
  vio_pid=$!
  echo "OpenVINS PID=$vio_pid, log=/tmp/ov_msckf_vio.log"
  if [[ "$stereo" == true && "$auto" == true ]]; then
    sleep 3
    ros2 run ov_rover_sim auto_loop.py --ros-args \
      -p enabled:=true -p mode:="$mode" -p use_sim_time:=true \
      > /tmp/ov_rover_auto_loop.log 2>&1 &
    drive_pid=$!
    echo "Stereo drive PID=$drive_pid, log=/tmp/ov_rover_auto_loop.log"
  fi
else
  vio_pid=""
fi

if [[ "$auto" == false ]]; then
  ros2 run ov_rover_sim key_teleop.py
else
  wait "$launch_pid"
fi
