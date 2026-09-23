#!/usr/bin/env bash
set -Eeuo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$repo_dir/install_vio/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1
# Keep this simulator independent from Gazebo instances on other ROS domains.
# An explicitly supplied URI still takes precedence.
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:$((11345 + ROS_DOMAIN_ID))}"

lock_file="/tmp/openvins_rover.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
  echo "Another OpenVINS rover launcher is already active." >&2
  echo "Stop that run before starting another one." >&2
  exit 1
fi

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

# A terminal closed during an earlier run can leave child processes alive even
# though the launcher lock was released. Stop only processes whose command line
# points into this OpenVINS simulator or uses this exact estimator config.
mapfile -t stale_pids < <(
  {
    pgrep -f "$repo_dir/install_vio/ov_rover_sim" || true
    pgrep -f "run_subscribe_msckf $config" || true
  } | sort -u
)
if ((${#stale_pids[@]})); then
  echo "Stopping ${#stale_pids[@]} orphaned OpenVINS rover process(es)."
  kill -INT "${stale_pids[@]}" 2>/dev/null || true
  sleep 2
  for pid in "${stale_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
  done
fi

echo "Starting camera plus IMU VIO simulation"
max_cameras=1
if [[ "$stereo" == true ]]; then max_cameras=2; fi
auto_launch="$auto"
# Give stereo OpenVINS a stationary interval for inertial initialization, then
# start automatic motion after both camera streams are being consumed.
if [[ "$stereo" == true && "$auto" == true ]]; then auto_launch=false; fi
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "auto=$auto mode=$mode gui=$gui rviz=$rviz vio=$vins cameras=$max_cameras"

ros2 launch ov_rover_sim rover_sim.launch.py \
  auto:="$auto_launch" mode:="$mode" gui:="$gui" rviz:="$rviz" > /tmp/vio_gazebo_launch.log 2>&1 &
launch_pid=$!
vio_pid=""

stop_pid() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -INT "$pid" 2>/dev/null || true
  for _ in {1..30}; do
    if ! kill -0 "$pid" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  trap - INT TERM EXIT
  stop_pid "$drive_pid"
  stop_pid "$vio_pid"
  stop_pid "$launch_pid"
}
trap cleanup INT TERM EXIT

if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo "The Gazebo launch process exited during startup." >&2
  tail -20 /tmp/vio_gazebo_launch.log >&2 || true
  exit 1
fi
topic_wait_args=(--timeout 30)
if [[ "$stereo" == true ]]; then topic_wait_args+=(--stereo); fi
if ! python3 "$repo_dir/ov_rover_sim/scripts/wait_for_topics.py" "${topic_wait_args[@]}"; then
  echo "Gazebo startup failed. Inspect /tmp/vio_gazebo_launch.log." >&2
  exit 1
fi

if [[ "$vins" == true ]]; then
  sleep 2
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
  if [[ "$stereo" == true && "$vins" == true ]]; then
    echo "Stereo initialization: the rover will drive a gentle arc before teleop starts."
    echo "Keep the Gazebo window focused only after keyboard control is announced."
    timeout --signal=INT 14s ros2 run ov_rover_sim auto_loop.py --ros-args \
      -p enabled:=true -p mode:=circle -p use_sim_time:=true \
      -p start_delay:=2.0 -p linear:=0.25 -p circle_angular:=0.25 \
      > /tmp/ov_rover_stereo_init.log 2>&1 || true
    sleep 1
    if tr -d '\000' < /tmp/ov_msckf_vio.log | grep -q 'successful initialization'; then
      echo "Stereo OpenVINS initialization succeeded. Keyboard control is now active."
    else
      echo "Stereo OpenVINS did not initialize during the first arc; extending initialization."
      timeout --signal=INT 10s ros2 run ov_rover_sim auto_loop.py --ros-args \
        -p enabled:=true -p mode:=circle -p use_sim_time:=true \
        -p start_delay:=0.5 -p linear:=0.25 -p circle_angular:=0.25 \
        >> /tmp/ov_rover_stereo_init.log 2>&1 || true
      sleep 1
      if ! tr -d '\000' < /tmp/ov_msckf_vio.log | grep -q 'successful initialization'; then
        echo "Stereo initialization failed. Teleop was not started to prevent an invalid trajectory." >&2
        echo "Inspect /tmp/ov_msckf_vio.log and /tmp/ov_rover_stereo_init.log." >&2
        exit 1
      fi
      echo "Stereo OpenVINS initialization succeeded. Keyboard control is now active."
    fi
  fi
  ros2 run ov_rover_sim key_teleop.py
else
  wait "$launch_pid"
fi
