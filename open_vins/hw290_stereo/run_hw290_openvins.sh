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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export DISPLAY="${DISPLAY:-:1}"
SPLITTER="${ROOT}/../install_vio/ov_hw290/lib/ov_hw290/stereo_splitter"
ESTIMATOR="${ROOT}/../install_vio/ov_msckf/lib/ov_msckf/run_subscribe_msckf"
CALIBRATION="${ROOT}/../../calibration/elp_3dgs1200p01/calib/calibration_opencv.yaml"
[[ -x "$SPLITTER" && -x "$ESTIMATOR" ]] || { echo "Build ov_hw290 and ov_msckf first" >&2; exit 1; }
[[ -r "$CALIBRATION" ]] || { echo "Stereo calibration unavailable: $CALIBRATION" >&2; exit 1; }
[[ -r /dev/video0 && -r /dev/ttyUSB0 ]] || { echo "Camera or Nano is unavailable" >&2; exit 1; }
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
PIDS=()
cleanup() { trap - INT TERM EXIT; for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup INT TERM EXIT
/opt/ros/humble/lib/usb_cam/usb_cam_node_exe --ros-args --params-file "${ROOT}/usb_cam_hw290.yaml" > /tmp/hw290_usb_cam.log 2>&1 & PIDS+=("$!")
sleep 3
# The new timestamp correction is experimental until the camera-to-IMU offset
# is recalibrated and the desk trajectory passes an out-and-back check.
"$SPLITTER" --ros-args -p calibration_file:="$CALIBRATION" -p auto_timestamp_correction:=false > /tmp/hw290_splitter.log 2>&1 & PIDS+=("$!")
python3 "${ROOT}/hw290_imu.py" > /tmp/hw290_imu.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 2.74403274 0.11300759 -1.62777045 imu camera_link > /tmp/hw290_tf_imu.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 camera_link cam0 > /tmp/hw290_tf_cam0.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0.05989 0 0 0 0 0 camera_link cam1 > /tmp/hw290_tf_cam1.log 2>&1 & PIDS+=("$!")
if [[ "$SENSORS_ONLY" == false ]]; then
  echo "WARNING: camera-to-IMU calibration is provisional. Validate the trajectory before using it as a measurement." >&2
  sleep 2
  "$ESTIMATOR" "${ROOT}/estimator_config.yaml" --ros-args -r __ns:=/ov_msckf -p use_sim_time:=false -p publish_calibration_tf:=false > /tmp/hw290_openvins.log 2>&1 & PIDS+=("$!")
fi
if [[ "$SHOW_RVIZ" == true ]]; then
  sleep 4
  RVIZ_CONFIG="${ROOT}/rviz_hw290.rviz"
  [[ "$SENSORS_ONLY" == true ]] && RVIZ_CONFIG="${ROOT}/rviz_hw290_sensors.rviz"
  rviz2 -d "$RVIZ_CONFIG" > /tmp/hw290_rviz.log 2>&1 & PIDS+=("$!")
fi
echo "HW290 stereo sensors running (sensors_only=${SENSORS_ONLY}, rviz=${SHOW_RVIZ}). Ctrl-C stops all nodes."
wait
