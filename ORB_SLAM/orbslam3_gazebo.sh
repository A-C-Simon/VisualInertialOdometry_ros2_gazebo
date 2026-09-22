#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
orbslam3_root="${ORB_SLAM3_ROOT:-/home/ac/ORB_SLAM3}"
mode="auto"
mode_was_set=false
build_requested=false
gui=true
rviz=true
orb_viewer=true
localization_only=false
dry_run=false

usage() {
  cat <<'EOF'
Usage: ./orbslam3_gazebo.sh [--auto | --teleop] [options]

Drive mode (mutually exclusive):
  --auto            Drive a camera-friendly circle (default)
  --teleop          Drive with arrow keys in this terminal

Options:
  --build           Rebuild both local ROS 2 packages before launching
  --headless        Do not open the Gazebo GUI
  --no-rviz         Do not open RViz2
  --no-orb-viewer   Do not open the ORB-SLAM3 Pangolin viewer
  --localization-only  Stop local mapping after stereo initialization
  --dry-run         Print the launch commands without running them
  -h, --help        Show this help
EOF
}

select_mode() {
  local requested="$1"
  if [[ "$mode_was_set" == true && "$mode" != "$requested" ]]; then
    echo "Error: --auto and --teleop cannot be used together." >&2
    exit 2
  fi
  mode="$requested"
  mode_was_set=true
}

while (($#)); do
  case "$1" in
    --auto) select_mode auto ;;
    --teleop) select_mode teleop ;;
    --build) build_requested=true ;;
    --headless) gui=false ;;
    --no-rviz) rviz=false ;;
    --no-orb-viewer) orb_viewer=false ;;
    --localization-only) localization_only=true ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Error: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

launch_args=(
  "auto:=$([[ "$mode" == auto ]] && echo true || echo false)"
  "gui:=$gui"
  "rviz:=$rviz"
  "orb_viewer:=$orb_viewer"
  "localization_only:=$localization_only"
  "vocabulary_path:=$orbslam3_root/Vocabulary/ORBvoc.txt"
  "trajectory_path:=$workspace_dir/gazebo_trajectory.txt"
  "keyframe_trajectory_path:=$workspace_dir/gazebo_keyframes.txt"
)

print_command() {
  printf '%q ' "$@"
  printf '\n'
}

if [[ "$dry_run" == true ]]; then
  if [[ "$build_requested" == true ]]; then
    print_command colcon build --packages-select orb_slam_ros2 \
      --symlink-install --cmake-args "-DORB_SLAM3_ROOT=$orbslam3_root"
    print_command colcon build --packages-select orbslam3_rover_sim --symlink-install
  fi
  print_command ros2 launch orbslam3_rover_sim rover_sim.launch.py "${launch_args[@]}"
  if [[ "$mode" == teleop ]]; then
    print_command ros2 run orbslam3_rover_sim key_teleop.py
  fi
  exit 0
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi
if [[ ! -f "$orbslam3_root/Vocabulary/ORBvoc.txt" ]]; then
  echo "Error: ORB-SLAM3 vocabulary not found under $orbslam3_root." >&2
  exit 1
fi

# ROS setup scripts may reference unset variables, so temporarily relax nounset.
set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$build_requested" == true || \
      ! -x "$workspace_dir/install/orb_slam_ros2/lib/orb_slam_ros2/stereo_node" || \
      ! -f "$workspace_dir/install/orbslam3_rover_sim/share/orbslam3_rover_sim/package.xml" ]]; then
  cd "$workspace_dir"
  colcon build \
    --packages-select orb_slam_ros2 \
    --symlink-install \
    --cmake-args "-DORB_SLAM3_ROOT=$orbslam3_root"
  colcon build --packages-select orbslam3_rover_sim --symlink-install
fi

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Error: local ROS 2 overlay was not created." >&2
  exit 1
fi
set +u
source "$workspace_dir/install/setup.bash"
set -u

# A terminal may already have /home/ac/ros2_ws sourced. That workspace also
# contains an older package with the same name but a different launch file.
# Put this workspace first explicitly, then verify what the ROS index resolves.
local_sim_prefix="$workspace_dir/install/orbslam3_rover_sim"
local_orb_prefix="$workspace_dir/install/orb_slam_ros2"
export AMENT_PREFIX_PATH="$local_sim_prefix:$local_orb_prefix:${AMENT_PREFIX_PATH:-/opt/ros/humble}"
export CMAKE_PREFIX_PATH="$local_sim_prefix:$local_orb_prefix:${CMAKE_PREFIX_PATH:-}"
export COLCON_PREFIX_PATH="$workspace_dir/install:${COLCON_PREFIX_PATH:-}"
export ROS2CLI_NO_DAEMON=1
resolved_sim_prefix="$(ros2 pkg prefix orbslam3_rover_sim 2>/dev/null || true)"
if [[ "$resolved_sim_prefix" != "$local_sim_prefix" ]]; then
  echo "Error: orbslam3_rover_sim resolved to '$resolved_sim_prefix', expected '$local_sim_prefix'." >&2
  echo "Run this script from /home/ac/ORB_SLAM or rebuild with --build." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/orbslam3_gazebo_logs}"
mkdir -p "$ROS_LOG_DIR"

launch_command=(ros2 launch orbslam3_rover_sim rover_sim.launch.py "${launch_args[@]}")

if [[ "$mode" == auto ]]; then
  echo "Launching local camera-only ORB-SLAM3 Gazebo simulation in automatic mode."
  echo "Package: $resolved_sim_prefix (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  exec "${launch_command[@]}"
fi

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$launch_pid" 2>/dev/null || return
      sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || kill -TERM "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Launching local camera-only ORB-SLAM3 Gazebo simulation in teleop mode."
echo "Package: $resolved_sim_prefix (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
setsid "${launch_command[@]}" &
launch_pid=$!

echo "Waiting for the rover command topic..."
for _ in {1..60}; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" || true
    echo "Error: the simulation stopped before teleop became ready." >&2
    exit 1
  fi
  if ros2 topic list 2>/dev/null | rg -q '^/cmd_vel$'; then
    break
  fi
  sleep 0.25
done

if ! ros2 topic list 2>/dev/null | rg -q '^/cmd_vel$'; then
  echo "Error: /cmd_vel did not appear within 15 seconds." >&2
  exit 1
fi

ros2 run orbslam3_rover_sim key_teleop.py
