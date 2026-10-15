#!/usr/bin/env bash
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE="${HW290_CONTAINER_IMAGE:-openvins-hw290:humble-arm64}"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'EOF'
Usage: ./hw290_stereo/run_hw290_container.sh [OpenVINS launcher options]

Run the calibrated tower headlessly in the prepared ROS 2 Humble container image.
Recommended: --benchmark. Ctrl-C stops the pipeline and saves its run summary.

Overrides: HW290_CONTAINER_IMAGE, HW290_CAMERA_DEVICE, HW290_IMU_PORT,
ROS_DOMAIN_ID (default 48), ROS_LOCALHOST_ONLY (default 1).
HW290_DDS_CONFIG supplies an optional Cyclone DDS XML file.
EOF
  exit 0
fi
command -v docker >/dev/null || { echo 'Docker is required.' >&2; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "Prepared image unavailable: $IMAGE. See SOPHON_DEPLOYMENT.md." >&2; exit 1;
}
if [[ -z "${HW290_CAMERA_DEVICE:-}" ]]; then
  shopt -s nullglob
  cameras=(/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_*-video-index0)
  ((${#cameras[@]} == 1)) || {
    echo 'Connect one ELP stereo camera, or set HW290_CAMERA_DEVICE explicitly.' >&2; exit 1;
  }
  HW290_CAMERA_DEVICE=${cameras[0]}
fi
HW290_CAMERA_DEVICE=$(readlink -f -- "$HW290_CAMERA_DEVICE")
HW290_IMU_PORT=$(readlink -f -- "${HW290_IMU_PORT:-/dev/ttyUSB0}")
[[ -c "$HW290_CAMERA_DEVICE" && -c "$HW290_IMU_PORT" ]] || {
  echo "Sensor device unavailable: camera=$HW290_CAMERA_DEVICE IMU=$HW290_IMU_PORT" >&2
  exit 1
}
tty_args=()
[[ ! -t 0 || ! -t 1 ]] || tty_args=(-it)
dds_args=()
if [[ -n "${HW290_DDS_CONFIG:-}" ]]; then
  dds_config=$(readlink -f -- "$HW290_DDS_CONFIG")
  [[ -r "$dds_config" ]] || { echo "DDS configuration unavailable: $dds_config" >&2; exit 1; }
  dds_args=(--mount "type=bind,src=$dds_config,dst=/tmp/openvins-dds.xml,readonly"
            --env CYCLONEDDS_URI=file:///tmp/openvins-dds.xml)
fi
exec docker run --rm --init "${tty_args[@]}" --network host --shm-size=256m \
  --stop-timeout 20 --name "openvins-hw290-run-$(id -u)" \
  --user "$(id -u):$(id -g)" \
  --device "$HW290_CAMERA_DEVICE:$HW290_CAMERA_DEVICE:rwm" \
  --device "$HW290_IMU_PORT:$HW290_IMU_PORT:rwm" \
  --group-add "$(stat -c '%g' "$HW290_CAMERA_DEVICE")" \
  --group-add "$(stat -c '%g' "$HW290_IMU_PORT")" \
  --mount "type=bind,src=$ROOT,dst=$ROOT" --workdir "$ROOT" \
  --env HOME=/tmp --env ROS_LOG_DIR=/tmp/openvins_ros_logs \
  --env "HW290_CAMERA_DEVICE=$HW290_CAMERA_DEVICE" --env "HW290_IMU_PORT=$HW290_IMU_PORT" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-48}" \
  --env "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}" \
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" "${dds_args[@]}" \
  --env OPENBLAS_NUM_THREADS=1 --env OMP_NUM_THREADS=1 \
  "$IMAGE" bash "$ROOT/hw290_stereo/run_hw290_openvins.sh" --no-rviz "$@"
