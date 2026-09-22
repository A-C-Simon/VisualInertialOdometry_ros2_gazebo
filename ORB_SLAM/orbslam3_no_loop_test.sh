#!/usr/bin/env bash
set -Eeuo pipefail

test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$test_dir/.." && pwd)"
sim_prefix="$repo_dir/install/orbslam3_rover_sim"
orb_prefix="$test_dir/ros2_install/orb_slam_ros2"
orbslam3_root="$test_dir/ORB_SLAM3"
settings="$test_dir/config/rover_gazebo_stereo_no_loop.yaml"
vocabulary="${ORB_SLAM3_VOCABULARY:-/home/ac/ORB_SLAM3/Vocabulary/ORBvoc.txt}"

mode=auto
gazebo=true
rviz=true
gazebo_gui=true
orb_viewer=true
localization_only=true
build=false

usage() {
  cat <<'EOF'
Usage: ./orbslam3_no_loop_test.sh [options]

Options:
  --auto                 Drive the simulated rover automatically (default)
  --teleop              Drive with keyboard controls in this terminal
  --no-gazebo           Use already-published stereo topics
  --headless             Disable Gazebo, RViz2, and Pangolin windows
  --no-rviz              Do not open RViz2
  --rviz                Open RViz2
  --gazebo-gui          Open the Gazebo GUI
  --orb-viewer          Open the Pangolin viewer
  --with-mapping        Keep local mapping enabled after initialization
  --build               Rebuild the isolated ROS 2 wrapper
  -h, --help            Show this help

Headless test example:
  ./orbslam3_no_loop_test.sh --auto

Teleoperation example:
  ./orbslam3_no_loop_test.sh --teleop --rviz
EOF
}

while (($#)); do
  case "$1" in
    --auto) mode=auto ;;
    --teleop) mode=teleop ;;
    --no-gazebo) gazebo=false ;;
    --headless) gazebo_gui=false; rviz=false; orb_viewer=false ;;
    --no-rviz) rviz=false ;;
    --rviz) rviz=true ;;
    --gazebo-gui) gazebo_gui=true ;;
    --orb-viewer) orb_viewer=true ;;
    --with-mapping) localization_only=false ;;
    --build) build=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Error: unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble not found." >&2; exit 1; }
[[ -f "$orb_prefix/lib/orb_slam_ros2/stereo_node" ]] || build=true
[[ -f "$vocabulary" ]] || { echo "Vocabulary not found: $vocabulary" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "$build" == true ]]; then
  colcon build --log-base "$test_dir/ros2_log" \
    --build-base "$test_dir/ros2_build" \
    --install-base "$test_dir/ros2_install" \
    --base-paths "$test_dir/src" --packages-select orb_slam_ros2 \
    --symlink-install --cmake-args "-DORB_SLAM3_ROOT=$orbslam3_root"
fi

export AMENT_PREFIX_PATH="$sim_prefix:$orb_prefix:/opt/ros/humble"
export CMAKE_PREFIX_PATH="$sim_prefix:$orb_prefix:/opt/ros/humble"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS2CLI_NO_DAEMON=1
export LD_LIBRARY_PATH="$orbslam3_root/lib:$orbslam3_root/Thirdparty/DBoW2/lib:$orbslam3_root/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

launch_args=(
  "auto:=$([[ "$mode" == auto ]] && echo true || echo false)"
  "gui:=$gazebo_gui"
  "rviz:=$rviz"
  "orb_slam:=true"
  "orb_viewer:=$orb_viewer"
  "localization_only:=$localization_only"
  "vocabulary_path:=$vocabulary"
  "orb_settings_path:=$settings"
  "trajectory_path:=$test_dir/trajectory.txt"
  "keyframe_trajectory_path:=$test_dir/keyframes.txt"
)

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$gazebo" == false ]]; then
  echo "Starting isolated ORB-SLAM3 on existing /cam0/image_raw and /cam1/image_raw topics."
  exec ros2 run orb_slam_ros2 stereo_node --ros-args \
    -p input_mode:=topics -p left_topic:=/cam0/image_raw -p right_topic:=/cam1/image_raw \
    -p vocabulary_path:="$vocabulary" -p settings_path:="$settings" \
    -p use_viewer:="$orb_viewer" -p localization_only:="$localization_only" \
    -p trajectory_path:="$test_dir/trajectory.txt"
fi

echo "Launching isolated no-loop-closure ORB-SLAM3 Gazebo test."
echo "Gazebo GUI: $gazebo_gui, RViz2: $rviz, Pangolin: $orb_viewer, mode: $mode"
setsid ros2 launch orbslam3_rover_sim rover_sim.launch.py "${launch_args[@]}" &
launch_pid=$!

if [[ "$mode" == teleop ]]; then
  for _ in {1..60}; do
    ros2 topic list 2>/dev/null | rg -q '^/cmd_vel$' && break
    sleep 0.25
  done
  ros2 run orbslam3_rover_sim key_teleop.py
else
  wait "$launch_pid"
fi
