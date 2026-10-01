#!/usr/bin/env bash
set -Eeo pipefail
SENSORS_ONLY=false
SHOW_RVIZ=true
DIAGNOSTICS=false
RAW_STEREO=false
IMU_ALLAN=false
for arg in "$@"; do
  case "$arg" in
    --sensors-only) SENSORS_ONLY=true ;;
    --no-rviz) SHOW_RVIZ=false ;;
    --diagnostics) DIAGNOSTICS=true ;;
    --raw-stereo) RAW_STEREO=true ;;
    --imu-allan) IMU_ALLAN=true ;;
    --help|-h)
      echo "Usage: $0 [--sensors-only] [--no-rviz] [--diagnostics] [--raw-stereo] [--imu-allan]"
      echo "Diagnostics records a sensor bag, raw IMU records and estimator state/debug logs."
      echo "Raw stereo disables rectification for offline camera calibration."
      echo "IMU Allan records only /imu0 for a long stationary noise measurement."
      echo "A valid VIO trajectory requires a rigid camera-IMU mount and measured extrinsics."
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done
if [[ "$IMU_ALLAN" == true && "$RAW_STEREO" == true ]]; then
  echo "--imu-allan and --raw-stereo are separate recording modes." >&2
  exit 2
fi
if [[ "$IMU_ALLAN" == true ]]; then
  SENSORS_ONLY=true
  SHOW_RVIZ=false
  DIAGNOSTICS=true
fi
if [[ "$RAW_STEREO" == true && "$SENSORS_ONLY" == false ]]; then
  echo "--raw-stereo requires --sensors-only because the VIO configuration expects rectified images." >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT}/../install_vio/setup.bash"
CAMERA_PREFIX="${ROOT}/../install_camera/usb_cam"
CAMERA_NODE="$CAMERA_PREFIX/lib/usb_cam/usb_cam_node_exe"
export LD_LIBRARY_PATH="$CAMERA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export DISPLAY="${DISPLAY:-:1}"
SPLITTER="${ROOT}/../install_vio/ov_hw290/lib/ov_hw290/stereo_splitter"
ESTIMATOR="${ROOT}/../install_vio/ov_msckf/lib/ov_msckf/run_subscribe_msckf"
CALIBRATION="${ROOT}/../../calibration/elp_3dgs1200p01/calib/calibration_opencv.yaml"
if [[ "$IMU_ALLAN" == false ]]; then
  [[ -x "$CAMERA_NODE" ]] || { echo "Build the corrected camera driver: hw290_stereo/build_camera_driver.sh" >&2; exit 1; }
  [[ -x "$SPLITTER" && -x "$ESTIMATOR" ]] || { echo "Build ov_hw290 and ov_msckf first" >&2; exit 1; }
  [[ -r "$CALIBRATION" ]] || { echo "Stereo calibration unavailable: $CALIBRATION" >&2; exit 1; }
  [[ -r /dev/video0 && -r /dev/ttyUSB0 ]] || echo "Camera or Nano unavailable; RViz diagnostics will remain available." >&2
else
  [[ -r /dev/ttyUSB0 ]] || echo "Nano unavailable; the Allan recording cannot start." >&2
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
HW290_DIR="$ROOT"
RUN_LABEL=hw290_openvins
[[ "$IMU_ALLAN" == true ]] && RUN_LABEL=hw290_imu_allan
source "$ROOT/process_helpers.sh"
if [[ "$IMU_ALLAN" == true ]]; then
  start_node recorder /tmp/hw290_recorder.log ros2 bag record -o "$RUN_DIR/imu_bag" /imu0
  sleep 1
  start_hw290_imu /tmp/hw290_imu.log
  wait_for_imu /tmp/hw290_imu.log
  echo "Stationary IMU Allan recording running. Keep the rig untouched for at least 3 hours; Ctrl-C stops and saves it."
  wait_for_run
  exit 0
fi
start_node camera /tmp/hw290_usb_cam.log "$CAMERA_NODE" --ros-args --params-file "${ROOT}/usb_cam_hw290.yaml"
sleep 1
# usb_cam 0.8.1 does not reliably apply this camera's UVC exposure controls.
# Reapply the tested low-blur manual setting after startup and verify it.
command -v v4l2-ctl >/dev/null 2>&1 || { echo "v4l2-ctl is required for HW290 exposure control" >&2; exit 1; }
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1,exposure_time_absolute=50,brightness=0,backlight_compensation=0,gain=255 \
  >/tmp/hw290_v4l2_controls.log 2>&1 || { echo "Failed to set safe HW290 exposure controls" >&2; exit 1; }
v4l2-ctl -d /dev/video0 --get-ctrl=auto_exposure,exposure_time_absolute,brightness,backlight_compensation,gain \
  >>/tmp/hw290_v4l2_controls.log 2>&1 || { echo "Failed to read back HW290 exposure controls" >&2; exit 1; }
grep -Eq '^auto_exposure: 1([[:space:]]|$)' /tmp/hw290_v4l2_controls.log &&
grep -Eq '^exposure_time_absolute: 50([[:space:]]|$)' /tmp/hw290_v4l2_controls.log &&
grep -Eq '^gain: 255([[:space:]]|$)' /tmp/hw290_v4l2_controls.log || {
  echo "HW290 camera rejected the tested exposure controls; see /tmp/hw290_v4l2_controls.log" >&2
  exit 1
}
sleep 2
# The local usb_cam fixes the clock-unit bug. Do not apply arrival-time
# correction to properly converted V4L2 capture timestamps.
SPLITTER_RECTIFY=true
if [[ "$RAW_STEREO" == true ]]; then
  SPLITTER_RECTIFY=false
  echo "Raw unrectified stereo output enabled for offline calibration."
fi
start_node splitter /tmp/hw290_splitter.log "$SPLITTER" --ros-args -p calibration_file:="$CALIBRATION" -p auto_timestamp_correction:=false -p rectify_images:="$SPLITTER_RECTIFY"
if [[ "$DIAGNOSTICS" == true ]]; then
  echo 'Diagnostic recording enabled; recording/debug CPU costs are included in this run.'
  touch "$RUN_DIR/diagnostics_enabled"
  start_node recorder /tmp/hw290_recorder.log ros2 bag record -o "$RUN_DIR/sensors_bag" /cam0/image_raw /cam1/image_raw /cam0/camera_info /cam1/camera_info /imu0 /ov_msckf/poseimu
  sleep 2
fi
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
  ESTIMATOR_ARGS=()
  if [[ "$DIAGNOSTICS" == true ]]; then
    ESTIMATOR_ARGS=(-p verbosity:=DEBUG -p save_total_state:=true -p filepath_est:="$RUN_DIR/state_estimate.txt" -p filepath_std:="$RUN_DIR/state_deviation.txt" -p filepath_gt:="$RUN_DIR/state_groundtruth.txt")
  fi
  start_node estimator /tmp/hw290_openvins.log "$ESTIMATOR" "${ROOT}/estimator_config.yaml" --ros-args -r __ns:=/ov_msckf -p use_sim_time:=false -p publish_calibration_tf:=false "${ESTIMATOR_ARGS[@]}"
fi
if [[ "$SHOW_RVIZ" == true ]]; then
  RVIZ_CONFIG="${ROOT}/rviz_hw290.rviz"
  [[ "$SENSORS_ONLY" == true || "$IMU_READY" == false ]] && RVIZ_CONFIG="${ROOT}/rviz_hw290_sensors.rviz"
  start_node rviz /tmp/hw290_rviz.log rviz2 -d "$RVIZ_CONFIG"
fi
echo "HW290 stereo sensors running (sensors_only=${SENSORS_ONLY}, rviz=${SHOW_RVIZ}). Ctrl-C stops all nodes."
# Keep the diagnostic window available even when a sensor fails.
wait_for_run
