#!/usr/bin/env bash
# Run the ORB-SLAM3 Gazebo rover.
#
# Examples:
#   ./orbslam3_gazebo.sh
#   ./orbslam3_gazebo.sh --gui false --rviz false
#   ./orbslam3_gazebo.sh --teleop true --gui false
#   ./orbslam3_gazebo.sh --no-gui --no-rviz --auto true --mode circle
#   ./orbslam3_gazebo.sh gui:=false rviz:=false teleop:=true
set -Eeuo pipefail

GUI=true
RVIZ=true
TELEOP=false
AUTO=true
MODE=circle
ORB_VIEWER=false
BUILD=false
DRY_RUN=false

usage() {
  sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# *//'
  cat <<'EOF'

Options (booleans accept true/false, 1/0, yes/no, or on/off):
  --gui BOOL          Gazebo graphical client (default: true)
  --rviz BOOL         RViz2 visualization (default: true)
  --teleop BOOL       Keyboard control; automatically disables auto drive
  --auto BOOL         Automatic rover drive (default: true)
  --mode MODE         Automatic path: circle or square (default: circle)
  --orb-viewer BOOL   Native Pangolin viewer (default: false)
  --build             Build the two ROS packages before launching
  --dry-run           Print the resolved commands without running them
  -h, --help          Show this help

Short boolean aliases are also accepted: --no-gui, --headless, --no-rviz,
--teleop, --no-teleop, --auto, --no-auto, --orb-viewer, --no-orb-viewer.
EOF
}

bool_value() {
  case "${1,,}" in
    true|1|yes|on) printf 'true' ;;
    false|0|no|off) printf 'false' ;;
    *) echo "Invalid boolean value: $1" >&2; exit 2 ;;
  esac
}

while (($#)); do
  case "$1" in
    --gui) GUI=$(bool_value "${2:?--gui requires true or false}"); shift 2 ;;
    --gui=*) GUI=$(bool_value "${1#*=}"); shift ;;
    gui:=*) GUI=$(bool_value "${1#*:=}"); shift ;;
    --no-gui|--headless) GUI=false; shift ;;
    --rviz|--rviz2)
      if (($# > 1)) && [[ "$2" != --* ]]; then RVIZ=$(bool_value "$2"); shift 2
      else RVIZ=true; shift; fi ;;
    --rviz=*|--rviz2=*) RVIZ=$(bool_value "${1#*=}"); shift ;;
    rviz:=*) RVIZ=$(bool_value "${1#*:=}"); shift ;;
    --no-rviz|--no-rviz2) RVIZ=false; shift ;;
    --teleop)
      if (($# > 1)) && [[ "$2" != --* ]]; then TELEOP=$(bool_value "$2"); shift 2
      else TELEOP=true; shift; fi ;;
    --teleop=*) TELEOP=$(bool_value "${1#*=}"); shift ;;
    teleop:=*) TELEOP=$(bool_value "${1#*:=}"); shift ;;
    --no-teleop) TELEOP=false; shift ;;
    --auto)
      if (($# > 1)) && [[ "$2" != --* ]]; then AUTO=$(bool_value "$2"); shift 2
      else AUTO=true; shift; fi ;;
    --auto=*) AUTO=$(bool_value "${1#*=}"); shift ;;
    auto:=*) AUTO=$(bool_value "${1#*:=}"); shift ;;
    --no-auto) AUTO=false; shift ;;
    --mode) MODE="${2:?--mode requires circle or square}"; shift 2 ;;
    --mode=*) MODE="${1#*=}"; shift ;;
    mode:=*) MODE="${1#*:=}"; shift ;;
    --circle) MODE=circle; shift ;;
    --square) MODE=square; shift ;;
    --orb-viewer)
      if (($# > 1)) && [[ "$2" != --* ]]; then ORB_VIEWER=$(bool_value "$2"); shift 2
      else ORB_VIEWER=true; shift; fi ;;
    --orb-viewer=*) ORB_VIEWER=$(bool_value "${1#*=}"); shift ;;
    orb_viewer:=*) ORB_VIEWER=$(bool_value "${1#*:=}"); shift ;;
    --no-orb-viewer) ORB_VIEWER=false; shift ;;
    --build) BUILD=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; echo "Run with --help for usage." >&2; exit 2 ;;
  esac
done

case "$MODE" in
  circle|square) ;;
  *) echo "Invalid mode '$MODE'; expected circle or square." >&2; exit 2 ;;
esac

# Only one process may own /cmd_vel.
if [[ "$TELEOP" == true ]]; then
  AUTO=false
fi

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd -- "$SCRIPT_DIR/.." && pwd)
ROS_SETUP=/opt/ros/humble/setup.bash
PANGOLIN_DIR="$WS/src/Pangolin/install/lib/cmake/Pangolin"
ORB_ROOT="$WS/src/ORB_SLAM3"
cd "$WS"

[[ -f "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
# shellcheck disable=SC1091
set +u
source "$ROS_SETUP"
set -u

if [[ "$BUILD" == true ]]; then
  [[ -f "$PANGOLIN_DIR/PangolinConfig.cmake" ]] || {
    echo "Pangolin is not built. Follow $WS/src/ORB_SLAM3_ROS2/README.md first." >&2
    exit 1
  }
  [[ -f "$ORB_ROOT/lib/libORB_SLAM3.so" ]] || {
    echo "ORB-SLAM3 core is not built. Follow $WS/src/ORB_SLAM3_ROS2/README.md first." >&2
    exit 1
  }
  colcon build --symlink-install --packages-select orbslam3 orbslam3_rover_sim \
    --cmake-args "-DORB_SLAM3_ROOT_DIR=$ORB_ROOT" "-DPangolin_DIR=$PANGOLIN_DIR"
fi

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash; run this script once with --build." >&2
  exit 1
fi
# shellcheck disable=SC1091
set +u
source "$WS/install/setup.bash"
set -u

# Prefer the workspace-local Pangolin build.  A different ABI-compatible
# SONAME may be installed in /usr/local (and can pull in unavailable OpenEXR
# versions), so relying on the dynamic linker's default order is unsafe.
export LD_LIBRARY_PATH="$WS/src/Pangolin/install/lib:$WS/src/ORB_SLAM3/lib:$WS/src/ORB_SLAM3/Thirdparty/DBoW2/lib:$WS/src/ORB_SLAM3/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if ! ros2 pkg prefix orbslam3_rover_sim >/dev/null 2>&1; then
  echo "orbslam3_rover_sim is not installed; run this script with --build." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROVER_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/orbslam3_ros_logs}"
export GAZEBO_LOG_PATH="${GAZEBO_LOG_PATH:-/tmp/orbslam3_gazebo_logs}"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"

LAUNCH=(ros2 launch orbslam3_rover_sim rover_orbslam.launch.py
  "gui:=$GUI" "rviz:=$RVIZ" "auto:=$AUTO" "mode:=$MODE"
  "orb_viewer:=$ORB_VIEWER")
TELEOP_CMD=(ros2 run orbslam3_rover_sim key_teleop.py)

printf 'Workspace:  %s\n' "$WS"
printf 'Settings:   gui=%s rviz=%s teleop=%s auto=%s mode=%s orb_viewer=%s\n' \
  "$GUI" "$RVIZ" "$TELEOP" "$AUTO" "$MODE" "$ORB_VIEWER"
printf 'ROS domain: %s\n' "$ROS_DOMAIN_ID"
printf 'Launch:    '; printf ' %q' "${LAUNCH[@]}"; printf '\n'

if [[ "$DRY_RUN" == true ]]; then
  if [[ "$TELEOP" == true ]]; then
    printf 'Teleop:   '; printf ' %q' "${TELEOP_CMD[@]}"; printf '\n'
  fi
  exit 0
fi

if pgrep -x gzserver >/dev/null; then
  echo "A gzserver is already running; stop it or use a separate Gazebo master." >&2
  exit 1
fi

XVFB_PID=''
LAUNCH_PID=''
cleanup() {
  trap - INT TERM EXIT
  [[ -n "$LAUNCH_PID" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  [[ -n "$XVFB_PID" ]] && kill "$XVFB_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Gazebo cameras still need an OpenGL display when the GUI is hidden.
if [[ "$GUI" == false && -z "${DISPLAY:-}" ]] && command -v Xvfb >/dev/null; then
  DISPLAY_FILE=$(mktemp /tmp/orbslam3_xvfb.XXXXXX)
  Xvfb -displayfd 3 -screen 0 1280x1024x24 3>"$DISPLAY_FILE" &
  XVFB_PID=$!
  for _ in {1..50}; do [[ -s "$DISPLAY_FILE" ]] && break; sleep 0.1; done
  DISPLAY_NUMBER=$(<"$DISPLAY_FILE")
  rm -f "$DISPLAY_FILE"
  if [[ -n "$DISPLAY_NUMBER" ]]; then
    export DISPLAY=":$DISPLAY_NUMBER"
    echo "Headless camera rendering uses Xvfb on $DISPLAY."
  fi
fi

"${LAUNCH[@]}" &
LAUNCH_PID=$!

if [[ "$TELEOP" == true ]]; then
  sleep 5
  echo "Teleop active: use arrow keys; Q or Ctrl+C stops it."
  "${TELEOP_CMD[@]}"
else
  wait "$LAUNCH_PID"
fi
