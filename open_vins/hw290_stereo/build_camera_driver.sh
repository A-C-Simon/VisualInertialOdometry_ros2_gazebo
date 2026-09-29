#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
work=$(mktemp -d /tmp/hw290-camera-build.XXXXXX)
git clone --depth 1 --branch 0.8.1 https://github.com/ros-drivers/usb_cam.git "$work/source"
git -C "$work/source" apply "$root/hw290_stereo/patches/usb_cam-0.8.1-timestamps.patch"
colcon --log-base "$work/log" build --base-paths "$work/source" \
  --build-base "$work/build" --install-base "$root/install_camera" \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
