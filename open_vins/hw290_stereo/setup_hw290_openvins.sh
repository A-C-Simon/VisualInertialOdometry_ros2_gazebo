#!/usr/bin/env bash
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
JOBS=2
DRY_RUN=false
CHECK=false
SKIP_SYSTEM=false
BUILD=true
CALIBRATION=false
CAL_TOOLS="${CAL_TOOLS:-$HOME/vio_calibration_tools}"
ARDUINO_VERSION=1.5.1
ROS_SETUP=/opt/ros/humble/setup.bash

usage() {
  cat <<'EOF'
Usage: ./hw290_stereo/setup_hw290_openvins.sh [options]

Install ROS 2 Humble and dependencies on Ubuntu 22.04, build the four hardware
packages and corrected camera driver, and prepare Arduino and bag tools.
Run as your normal account; system installation uses sudo.

  --with-calibration-tools  Also install Docker if absent and build Kalibr/Allan images
  --jobs N                  Compilation jobs, default 2
  --skip-system             Reuse system packages and group settings; do not use apt
  --skip-build              Install dependencies and tools without compiling VIO/camera
  --dry-run                 Show the plan without writes, downloads or sudo
  --check                   Read-only software readiness check
  --help                    Show this help

Firmware flashing, device paths and calibration require the actual hardware.
This script does not change firmware, calibration files or shell startup files.
EOF
}
die() { echo "ERROR: $*" >&2; exit 1; }
while (($#)); do
  case "$1" in
    --with-calibration-tools) CALIBRATION=true ;;
    --jobs) (($# >= 2)) || die '--jobs needs a positive integer'; JOBS=$2; shift ;;
    --skip-system) SKIP_SYSTEM=true ;;
    --skip-build) BUILD=false ;;
    --dry-run) DRY_RUN=true ;;
    --check) CHECK=true ;;
    --help|-h) usage; exit 0 ;;
    *) die "Unknown option: $1" ;;
  esac
  shift
done
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || die '--jobs needs a positive integer'
[[ "$DRY_RUN:$CHECK" != true:true ]] || die 'Choose --dry-run or --check'
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || die 'Supported host: Ubuntu 22.04 (ROS 2 Humble)'
case "$(uname -m)" in
  x86_64|aarch64) ;;
  *) die 'Supported build architectures: x86_64 and aarch64; hardware validation is x86_64 only' ;;
esac

PACKAGES=(
  ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep
  build-essential cmake git pkg-config v4l-utils ffmpeg usbutils openscad
  libavcodec-dev libavutil-dev libswscale-dev libeigen3-dev libboost-all-dev
  libceres-dev libopencv-dev libopencv-contrib-dev libyaml-cpp-dev libssl-dev qtbase5-dev
  python3-dev python3-numpy python3-scipy python3-yaml python3-serial python3-pip python3-venv
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-image-transport-plugins
  ros-humble-camera-info-manager ros-humble-ament-cmake-auto
  ros-humble-rosidl-default-generators ros-humble-message-filters
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-rosbag2
)
BINARIES=(
  ov_msckf/run_subscribe_msckf ov_hw290/hw290_imu ov_hw290/stereo_splitter
  ov_hw290/calibration_screen ov_hw290/build_rectified_profile ov_hw290/check_mount_calibration
)
plan() {
  echo "Workspace: $ROOT"
  echo "Host: $PRETTY_NAME ($(uname -m)); build jobs: $JOBS"
  if [[ "$SKIP_SYSTEM" == false ]]; then
    echo 'Install prerequisites, generate UTF-8 locale and enable Ubuntu Universe.'
    echo 'Configure the official ROS apt source when needed; install:'
    printf '  %s\n' "${PACKAGES[@]}"
    echo 'Add this account to dialout/video; a new login may be required.'
  fi
  echo "Prepare Arduino CLI $ARDUINO_VERSION if absent, AVR core 1.8.8 and a rosbags venv."
  [[ "$BUILD" == false ]] || echo 'Build ov_core, ov_init, ov_msckf, ov_hw290 and patched usb_cam 0.8.1.'
  [[ "$CALIBRATION" == false ]] || echo 'Prepare Docker and build pinned Kalibr and Allan images; this can take considerable time/disk.'
  echo 'Keep firmware, calibration and shell startup files unchanged.'
}
ready_check() {
  local missing=0 item verify_images=${1:-true}
  for item in "${PACKAGES[@]}"; do
    if [[ "$(dpkg-query -W -f='${Status}' "$item" 2>/dev/null || true)" != 'install ok installed' ]]; then
      echo "MISSING package: $item"; missing=1
    fi
  done
  for item in "${BINARIES[@]}"; do
    local package=${item%%/*} name=${item#*/}
    [[ -x "$ROOT/install_vio/$package/lib/$package/$name" ]] || {
      echo "MISSING binary: $item"; missing=1;
    }
  done
  [[ -r "$ROS_SETUP" && -r "$ROOT/install_vio/setup.bash" ]] || {
    echo 'MISSING ROS environment'; missing=1;
  }
  [[ -x "$ROOT/install_camera/usb_cam/lib/usb_cam/usb_cam_node_exe" ]] || {
    echo 'MISSING patched camera driver'; missing=1;
  }
  if command -v arduino-cli >/dev/null; then
    if ! arduino-cli core list | grep -Eq '^arduino:avr[[:space:]]+1\.8\.8([[:space:]]|$)'; then
      echo 'MISSING Arduino AVR core 1.8.8'; missing=1
    fi
  else
    echo 'MISSING Arduino CLI'; missing=1
  fi
  [[ -x "$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" ]] || {
    echo 'MISSING bag conversion venv'; missing=1;
  }
  if [[ "$CALIBRATION" == true && "$verify_images" == true ]]; then
    "$ROOT/hw290_stereo/build_calibration_tools.sh" --check || missing=1
  fi
  echo 'Device calibration and physical motion acceptance must be checked separately.'
  ((missing == 0)) || return 1
  echo 'Software dependencies and expected build outputs are present.'
}
export PATH="$HOME/.local/bin:$PATH"
export ARDUINO_UPDATER_ENABLE_NOTIFICATION=false
if [[ "$DRY_RUN" == true ]]; then plan; exit 0; fi
if [[ "$CHECK" == true ]]; then ready_check; exit; fi
((EUID != 0)) || die 'Run as your normal desktop account, without sudo before the script'
if [[ "$SKIP_SYSTEM" == false ]]; then
  command -v sudo >/dev/null || die 'sudo is required for system installation'
  sudo -v
fi
LOG_DIR=$(mktemp -d "$ROOT/benchmark/results/setup_$(date +%Y%m%d_%H%M%S)_XXXXXX" 2>/dev/null) || {
  mkdir -p "$ROOT/benchmark/results"
  LOG_DIR=$(mktemp -d "$ROOT/benchmark/results/setup_$(date +%Y%m%d_%H%M%S)_XXXXXX")
}
exec > >(tee "$LOG_DIR/setup.log") 2>&1
STAGE=preflight
trap 'status=$?; echo "Setup failed during $STAGE (exit $status). Log: $LOG_DIR/setup.log" >&2; exit "$status"' ERR
plan
download() { curl --fail --show-error --location --retry 3 --connect-timeout 20 "$1" -o "$2"; }
apt_install() { sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"; }

if [[ "$SKIP_SYSTEM" == false ]]; then
  STAGE='system prerequisites'
  sudo apt-get update
  apt_install ca-certificates curl gnupg software-properties-common locales python3 systemd udev
  sudo add-apt-repository -y universe
  sudo locale-gen en_US.UTF-8
  export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  STAGE='ROS apt source'
  # Reuse a working existing Humble source rather than create duplicate sources.
  if [[ ! -r "$ROS_SETUP" ]] || ! apt-cache show ros-humble-desktop >/dev/null 2>&1; then
    download https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest "$LOG_DIR/ros-apt-release.json"
    source_version=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$LOG_DIR/ros-apt-release.json")
    [[ "$source_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die 'Unexpected ROS apt source release version'
    download "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$source_version/ros2-apt-source_${source_version}.jammy_all.deb" "$LOG_DIR/ros2-apt-source.deb"
    sudo dpkg -i "$LOG_DIR/ros2-apt-source.deb"
  fi
  sudo apt-get update
  STAGE='ROS and build dependencies'
  apt_install "${PACKAGES[@]}"
  STAGE='device access groups'
  sudo usermod -aG dialout,video "$(id -un)"
fi
[[ -r "$ROS_SETUP" ]] || die 'ROS 2 Humble is missing; rerun without --skip-system'
command -v curl >/dev/null || die 'curl is required'

STAGE='Arduino tools'
mkdir -p "$HOME/.local/bin" "$CAL_TOOLS"
if ! command -v arduino-cli >/dev/null; then
  # Official installer at the same release as the requested binary.
  download "https://raw.githubusercontent.com/arduino/arduino-cli/v$ARDUINO_VERSION/install.sh" "$LOG_DIR/arduino-install.sh"
  BINDIR="$HOME/.local/bin" bash "$LOG_DIR/arduino-install.sh" "$ARDUINO_VERSION"
fi
arduino-cli version
arduino-cli core update-index
arduino-cli core install arduino:avr@1.8.8

STAGE='bag conversion tools'
if [[ ! -x "$CAL_TOOLS/rosbags-venv/bin/python" ]]; then
  python3 -m venv "$CAL_TOOLS/rosbags-venv"
fi
"$CAL_TOOLS/rosbags-venv/bin/python" -m pip install rosbags
"$CAL_TOOLS/rosbags-venv/bin/python" -m pip freeze > "$CAL_TOOLS/rosbags-requirements.txt"
"$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" --help >/dev/null

if [[ "$BUILD" == true ]]; then
  STAGE='OpenVINS and sensor build'
  (
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
    set +u
    source "$ROS_SETUP"
    set -u
    export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" MAKEFLAGS="-j$JOBS"
    cd "$ROOT"
    colcon --log-base log_vio build --executor sequential \
      --base-paths ov_core ov_init ov_msckf ov_hw290 \
      --build-base build_vio --install-base install_vio --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    STAGE='patched camera build'
    bash "$ROOT/hw290_stereo/build_camera_driver.sh"
  )
fi

if [[ "$CALIBRATION" == true ]]; then
  STAGE='Docker installation'
  if ! command -v docker >/dev/null; then
    [[ "$SKIP_SYSTEM" == false ]] || die 'Docker is missing; install it or rerun without --skip-system'
    for conflict in containerd runc podman-docker docker.io; do
      [[ "$(dpkg-query -W -f='${Status}' "$conflict" 2>/dev/null || true)" != 'install ok installed' ]] || \
        die "Existing $conflict needs review before installing Docker CE; see Docker Ubuntu installation guide"
    done
    download https://download.docker.com/linux/ubuntu/gpg "$LOG_DIR/docker.asc"
    sudo install -D -m 0644 "$LOG_DIR/docker.asc" /etc/apt/keyrings/docker.asc
    printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu jammy stable\n' \
      "$(dpkg --print-architecture)" > "$LOG_DIR/docker.list"
    sudo install -m 0644 "$LOG_DIR/docker.list" /etc/apt/sources.list.d/docker.list
    sudo apt-get update
    apt_install docker-ce docker-ce-cli containerd.io docker-buildx-plugin
    sudo systemctl enable --now docker
  fi
  STAGE='offline calibration images'
  CAL_TOOLS="$CAL_TOOLS" "$ROOT/hw290_stereo/build_calibration_tools.sh" --jobs "$JOBS"
fi

STAGE='software verification'
# The image builder already smoke-checked Docker, including sudo when needed.
if [[ "$BUILD" == true ]]; then ready_check false; fi
git -C "$ROOT" rev-parse HEAD > "$LOG_DIR/source_revision.txt"
dpkg-query -W -f='${Package}\t${Version}\n' "${PACKAGES[@]}" > "$LOG_DIR/packages.txt"
arduino-cli version > "$LOG_DIR/arduino_version.txt"
arduino-cli core list > "$LOG_DIR/arduino_cores.txt"
cp "$CAL_TOOLS/rosbags-requirements.txt" "$LOG_DIR/"
echo
echo "Setup complete. Log and versions: $LOG_DIR"
if [[ "$BUILD" == true ]]; then
  printf 'Environment: source %q\n' "$ROOT/hw290_stereo/env_hw290.sh"
else
  echo 'Compilation deferred. Rerun without --skip-build before sourcing the environment.'
fi
echo 'Log out and in if dialout/video access was just added.'
echo 'New device: follow OPENVINS_IMPLEMENTATION_GUIDE.md Sections 5-9 before normal VIO.'
echo 'No firmware has been uploaded and no calibration has been activated.'
