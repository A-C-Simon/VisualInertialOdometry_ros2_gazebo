#!/usr/bin/env bash
set -Eeo pipefail
SENSORS_ONLY=false
SHOW_RVIZ=true
for arg in "$@"; do
  case "$arg" in
    --sensors-only) SENSORS_ONLY=true ;;
    --no-rviz) SHOW_RVIZ=false ;;
    --help|-h)
      echo "Usage: $0 [--sensors-only] [--no-rviz]"
      echo "A valid VIO trajectory requires a rigid camera-IMU mount and measured extrinsics."
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT}/../install_vio/setup.bash"
CAMERA_PREFIX="${ROOT}/../install_camera/usb_cam"
CAMERA_NODE="$CAMERA_PREFIX/lib/usb_cam/usb_cam_node_exe"
[[ -x "$CAMERA_NODE" ]] || { echo "Build the corrected camera driver: hw290_stereo/build_camera_driver.sh" >&2; exit 1; }
export LD_LIBRARY_PATH="$CAMERA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export DISPLAY="${DISPLAY:-:1}"
SPLITTER="${ROOT}/../install_vio/ov_hw290/lib/ov_hw290/stereo_splitter"
ESTIMATOR="${ROOT}/../install_vio/ov_msckf/lib/ov_msckf/run_subscribe_msckf"
CALIBRATION="${ROOT}/../../calibration/elp_3dgs1200p01/calib/calibration_opencv.yaml"
[[ -x "$SPLITTER" && -x "$ESTIMATOR" ]] || { echo "Build ov_hw290 and ov_msckf first" >&2; exit 1; }
[[ -r "$CALIBRATION" ]] || { echo "Stereo calibration unavailable: $CALIBRATION" >&2; exit 1; }
[[ -r /dev/video0 && -r /dev/ttyUSB0 ]] || echo "Camera or Nano unavailable; RViz diagnostics will remain available." >&2
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
HW290_DIR="$ROOT"
RUN_LABEL=hw290_openvins
source "$ROOT/process_helpers.sh"
start_node camera /tmp/hw290_usb_cam.log "$CAMERA_NODE" --ros-args --params-file "${ROOT}/usb_cam_hw290.yaml"
sleep 3
# The local usb_cam fixes the clock-unit bug. Do not apply arrival-time
# correction to properly converted V4L2 capture timestamps.
start_node splitter /tmp/hw290_splitter.log "$SPLITTER" --ros-args -p calibration_file:="$CALIBRATION" -p auto_timestamp_correction:=false
start_hw290_imu /tmp/hw290_imu.log
IMU_READY=true
if ! wait_for_imu /tmp/hw290_imu.log; then
  IMU_READY=false
  echo "Starting RViz in sensor view; VIO is unavailable until the IMU connection is restored and this launcher is restarted." >&2
fi
for camera in cam0 cam1; do
  mapfile -t TF_ARGS < <(python3 "$ROOT/static_transform_args.py" "$camera")
  start_node "tf_$camera" "/tmp/hw290_tf_${camera}.log" /opt/ros/humble/lib/tf2_ros/static_transform_publisher "${TF_ARGS[@]}"
done
if [[ "$SENSORS_ONLY" == false && "$IMU_READY" == true ]]; then
  echo "WARNING: camera-to-IMU calibration is provisional. Validate the trajectory before using it as a measurement." >&2
  sleep 2
  start_node estimator /tmp/hw290_openvins.log "$ESTIMATOR" "${ROOT}/estimator_config.yaml" --ros-args -r __ns:=/ov_msckf -p use_sim_time:=false -p publish_calibration_tf:=false
fi
if [[ "$SHOW_RVIZ" == true ]]; then
  RVIZ_CONFIG="${ROOT}/rviz_hw290.rviz"
  [[ "$SENSORS_ONLY" == true || "$IMU_READY" == false ]] && RVIZ_CONFIG="${ROOT}/rviz_hw290_sensors.rviz"
  start_node rviz /tmp/hw290_rviz.log rviz2 -d "$RVIZ_CONFIG"
fi
echo "HW290 stereo sensors running (sensors_only=${SENSORS_ONLY}, rviz=${SHOW_RVIZ}). Ctrl-C stops all nodes."
# Keep the diagnostic window available even when a sensor fails.
wait_for_run
