#!/usr/bin/env bash
# ORB-SLAM3 stereo-inertial live test with ELP camera + HW-290 IMU.
# Pipeline: usb_cam (1280x480 side-by-side) -> ov_hw290 stereo_splitter
# (rectified 640x480 /cam0,/cam1) + C++ hw290_imu (/imu0 ~100 Hz) ->
# orb_slam_ros2 stereo_imu_node (IMU_STEREO); viewers are opt-in.
#
# The measured OpenVINS camera/IMU profile is exported for every run.
# IMU noise remains provisional. See open_vins/hw290_stereo/DRIFT_FIX.md.
# Hold still ~2 s at start, then slow rotation+translation
# in a textured 0.5-2 m static scene for IMU initialization.
set -Eeo pipefail

SENSORS_ONLY=false
SHOW_RVIZ=false
USE_VIEWER=false
EFFICIENT=false
DIAGNOSTICS=false
for arg in "$@"; do
  case "$arg" in
    --sensors-only) SENSORS_ONLY=true ;;
    --efficient) EFFICIENT=true ;;
    --diagnostics) DIAGNOSTICS=true ;;
    --rviz) SHOW_RVIZ=true ;;
    --viewer) USE_VIEWER=true ;;
    --no-rviz) SHOW_RVIZ=false ;;
    --no-viewer) USE_VIEWER=false ;;
    --help|-h)
      echo "Usage: $0 [--sensors-only] [--efficient] [--diagnostics] [--rviz] [--viewer] [--no-rviz] [--no-viewer]"
      echo "  --efficient    use 600 features and capped local BA (validated on EuRoC)"
      echo "  --diagnostics  save sensor/pose bag and raw IMU log"
      echo "  --sensors-only  publish camera+IMU only, skip ORB-SLAM3"
      echo "  --no-rviz       skip RViz2 (default; --viewer enables Pangolin)"
      echo "  --no-viewer     disable Pangolin viewer (headless benchmark)"
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

ORB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIO_ROOT="/home/ac/VisualInertialOdometry_ros2_gazebo/open_vins"
HW290_DIR="${VIO_ROOT}/hw290_stereo"
CALIBRATION="${HW290_STEREO_CALIBRATION:-${HW290_DIR}/calibration/20261001/stereo_opencv.yaml}"
ESTIMATOR_CONFIG="${HW290_VIO_CONFIG:-${HW290_DIR}/estimator_config.yaml}"
CALIBRATION_EXPORTER="${VIO_ROOT}/install_vio/ov_hw290/lib/ov_hw290/export_orb_calibration"
MOUNT_METADATA="${HW290_DIR}/calibration/current_mount.yaml"
MOUNT_CHECK="${VIO_ROOT}/install_vio/ov_hw290/lib/ov_hw290/check_mount_calibration"
MOUNT_VALID=false
if [[ -x "$MOUNT_CHECK" ]] && "$MOUNT_CHECK" "$MOUNT_METADATA" "$ESTIMATOR_CONFIG" "$CALIBRATION"; then
  MOUNT_VALID=true
fi
if [[ "$SENSORS_ONLY" == false && "$MOUNT_VALID" == false ]]; then
  echo "VIO stopped: current mount requires spatial/time calibration. Use open_vins/hw290_stereo/run_hw290_openvins.sh --calibration-screen." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
# Underlay with the C++ splitter, then the local ORB-SLAM workspace last so its
# orb_slam_ros2 (with stereo_imu_node) wins over ~/ORB_SLAM/install.
if [[ -r "${VIO_ROOT}/install_vio/setup.bash" ]]; then
  source "${VIO_ROOT}/install_vio/setup.bash"
fi
source "${ORB_ROOT}/install/setup.bash"
export ORB_SLAM3_ROOT="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export DISPLAY="${DISPLAY:-:1}"

SPLITTER="${VIO_ROOT}/install_vio/ov_hw290/lib/ov_hw290/stereo_splitter"
USB_CAM_PARAMS="${HW290_DIR}/usb_cam_hw290.yaml"
CAMERA_PREFIX="${VIO_ROOT}/install_camera/usb_cam"
CAMERA_NODE="$CAMERA_PREFIX/lib/usb_cam/usb_cam_node_exe"
[[ -x "$CAMERA_NODE" ]] || { echo "Build open_vins/hw290_stereo/build_camera_driver.sh first" >&2; exit 1; }
export LD_LIBRARY_PATH="$CAMERA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
FEATURES=800
# The isolated core supplies the Rectified camera loader fix and safe export.
CORE_DIR="${ORB_CORE_DIR:-${VIO_ROOT}/benchmark/build_orb_core}"
[[ -r "$CORE_DIR/libORB_SLAM3.so" ]] || { echo "Run python3 open_vins/benchmark/build_orb_core.py first" >&2; exit 1; }
CORE_DIR="$(cd "$CORE_DIR" && pwd)"
export ORB_CORE_DIR="$CORE_DIR"
export LD_LIBRARY_PATH="$CORE_DIR:${LD_LIBRARY_PATH:-}"
if [[ "$EFFICIENT" == true ]]; then
  export ORB_LOCAL_BA_WINDOW="${ORB_LOCAL_BA_WINDOW:-12}"
  FEATURES=600
else
  export ORB_LOCAL_BA_WINDOW="${ORB_LOCAL_BA_WINDOW:-25}"
fi
RVIZ_VIO="${ORB_ROOT}/orb_slam_ros2/config/rviz_hw290_vio.rviz"
RVIZ_SENSORS="${HW290_DIR}/rviz_hw290_sensors.rviz"
[[ -x "$SPLITTER" ]] || { echo "Splitter missing: $SPLITTER (build ov_hw290 first)" >&2; exit 1; }
[[ -r "$CALIBRATION" ]] || { echo "Stereo calibration unavailable: $CALIBRATION" >&2; exit 1; }
[[ -r "$ESTIMATOR_CONFIG" && -x "$CALIBRATION_EXPORTER" ]] || { echo "Build ov_hw290 and provide its measured estimator profile first" >&2; exit 1; }
[[ -r /dev/video0 && -r /dev/ttyUSB0 ]] || echo "Camera or Nano unavailable; RViz diagnostics will remain available." >&2
[[ -r "${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt" ]] || { echo "ORB vocabulary missing at ${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt" >&2; exit 1; }

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} DISPLAY=${DISPLAY} viewer=${USE_VIEWER} rviz=${SHOW_RVIZ} sensors_only=${SENSORS_ONLY}"
echo "TIP: keep the rig still ~2 s, then move slowly (rotation+translation) facing textured static objects 0.5-2 m away."

RUN_LABEL=hw290_orb
source "$HW290_DIR/process_helpers.sh"
[[ ! -r "$MOUNT_METADATA" ]] || cp "$MOUNT_METADATA" "$RUN_DIR/current_mount.yaml"
printf '%s\n' "$CORE_DIR" > "$RUN_DIR/orb_core_directory.txt"
sha256sum "$CORE_DIR/libORB_SLAM3.so" > "$RUN_DIR/orb_core_sha256.txt"
for manifest in build_manifest.json validation_manifest.json prototype_manifest.json; do
  [[ ! -r "$CORE_DIR/$manifest" ]] || cp "$CORE_DIR/$manifest" "$RUN_DIR/orb_core_$manifest"
done
echo "ORB core: $CORE_DIR"
SETTINGS_PATH="$RUN_DIR/orb_settings.yaml"
CALIBRATED_OFFSET=$("$CALIBRATION_EXPORTER" "$ESTIMATOR_CONFIG" "$CALIBRATION" "$SETTINGS_PATH" "$FEATURES")
cp "$CALIBRATION" "$RUN_DIR/stereo_opencv.yaml"
printf '%s\n' "$ESTIMATOR_CONFIG" "$CALIBRATION" > "$RUN_DIR/calibration_source.txt"
echo "Measured profile: camera/IMU offset=${CALIBRATED_OFFSET}s, features=${FEATURES}, local BA cap=${ORB_LOCAL_BA_WINDOW}"

start_node camera /tmp/orb_vio_usb_cam.log "$CAMERA_NODE" --ros-args --params-file "$USB_CAM_PARAMS"
sleep 1
configure_hw290_camera /tmp/orb_vio_v4l2_controls.log
sleep 2
start_node splitter /tmp/orb_vio_splitter.log "$SPLITTER" --ros-args -p calibration_file:="$CALIBRATION" -p auto_timestamp_correction:=false
if [[ "$DIAGNOSTICS" == true ]]; then
  touch "$RUN_DIR/diagnostics_enabled"
  cp "$HW290_DIR/record_qos.yaml" "$RUN_DIR/record_qos.yaml"
  start_node recorder /tmp/orb_vio_recorder.log ros2 bag record --qos-profile-overrides-path "$RUN_DIR/record_qos.yaml" -o "$RUN_DIR/sensors_bag" /cam0/image_raw /cam1/image_raw /imu0 /orbslam_vio/pose_imu /orbslam_vio/tracking_state
  sleep 2
fi
start_hw290_imu /tmp/orb_vio_imu.log
IMU_READY=true
if ! wait_for_imu /tmp/orb_vio_imu.log; then
  IMU_READY=false
  echo "Starting RViz in sensor view; VIO unavailable until IMU is restored and launcher restarted." >&2
fi
if [[ "$MOUNT_VALID" == true ]]; then
for camera in cam0 cam1; do
  HW290_TF_OUTPUT=$(python3 "$HW290_DIR/static_transform_args.py" "$camera" --estimator-config "$ESTIMATOR_CONFIG")
  mapfile -t TF_ARGS <<< "$HW290_TF_OUTPUT"
  start_node "tf_$camera" "/tmp/orb_vio_tf_${camera}.log" /opt/ros/humble/lib/tf2_ros/static_transform_publisher "${TF_ARGS[@]}"
done
else
  echo "Camera-to-IMU TF is unverified for the current mount." >&2
fi

if [[ "$SENSORS_ONLY" == false && "$IMU_READY" == true ]]; then
  HW290_TF_OUTPUT=$(python3 "$HW290_DIR/static_transform_args.py" orb_imu --estimator-config "$ESTIMATOR_CONFIG")
  mapfile -t ORB_IMU_TF <<< "$HW290_TF_OUTPUT"
  start_node tf_orb_imu /tmp/orb_vio_tf_orb_imu.log /opt/ros/humble/lib/tf2_ros/static_transform_publisher "${ORB_IMU_TF[@]}"
  sleep 2
  start_node estimator /tmp/orb_vio_slam.log ros2 launch orb_slam_ros2 stereo_inertial_topics.launch.py \
    namespace:=orbslam_vio viewer:="$USE_VIEWER" reliable_images:="${ORB_RELIABLE_IMAGES:-true}" settings_path:="$SETTINGS_PATH" camera_imu_offset:="${CAMERA_IMU_OFFSET:-$CALIBRATED_OFFSET}" \
    trajectory_path:="$RUN_DIR/vio_session.txt" keyframe_trajectory_path:="$RUN_DIR/kf_vio_session.txt" timing_path:="$RUN_DIR/vio_timing.txt" \
    online_trajectory_path:="$RUN_DIR/online_imu_poses.txt"
fi
if [[ "$SHOW_RVIZ" == true ]]; then
  RVIZ_CONFIG="$RVIZ_VIO"
  [[ "$SENSORS_ONLY" == true || "$IMU_READY" == false ]] && RVIZ_CONFIG="$RVIZ_SENSORS"
  start_node rviz /tmp/orb_vio_rviz.log rviz2 -d "$RVIZ_CONFIG"
fi

echo "ORB-SLAM3 HW290 VIO running (sensors_only=${SENSORS_ONLY}). Ctrl-C stops all nodes + prints benchmark."
wait_for_run
