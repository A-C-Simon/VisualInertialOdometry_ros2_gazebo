#!/usr/bin/env bash
# ORB-SLAM3 stereo-inertial live test with ELP camera + HW-290 IMU.
# Pipeline: usb_cam (1280x480 side-by-side) -> ov_hw290 stereo_splitter
# (rectified 640x480 /cam0,/cam1) + hw290_imu.py (/imu0 100 Hz) ->
# orb_slam_ros2 stereo_imu_node (IMU_STEREO) with Pangolin viewer ->
# RViz2 (/orbslam_vio/path, points, tracking image).
#
# Calibration status is provisional (see orb_slam_ros2/config/
# ELP_640x480_inertial.yaml header and open_vins/hw290_stereo/README.md):
# rotation-only cam-to-IMU estimate, zero translation, 0.155 s time offset
# NOT compensated. Hold still ~2 s at start, then slow rotation+translation
# in a textured 0.5-2 m static scene for IMU initialization.
set -Eeo pipefail

SENSORS_ONLY=false
SHOW_RVIZ=true
USE_VIEWER=true
for arg in "$@"; do
  case "$arg" in
    --sensors-only) SENSORS_ONLY=true ;;
    --no-rviz) SHOW_RVIZ=false ;;
    --no-viewer) USE_VIEWER=false ;;
    --help|-h)
      echo "Usage: $0 [--sensors-only] [--no-rviz] [--no-viewer]"
      echo "  --sensors-only  publish camera+IMU only, skip ORB-SLAM3"
      echo "  --no-rviz       skip RViz2 (Pangolin still shows unless --no-viewer)"
      echo "  --no-viewer     disable Pangolin viewer (headless benchmark)"
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

ORB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIO_ROOT="/home/ac/VisualInertialOdometry_ros2_gazebo/open_vins"
HW290_DIR="${VIO_ROOT}/hw290_stereo"
CALIBRATION="/home/ac/VisualInertialOdometry_ros2_gazebo/calibration/elp_3dgs1200p01/calib/calibration_opencv.yaml"

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
IMU_PY="${HW290_DIR}/hw290_imu.py"
USB_CAM_PARAMS="${HW290_DIR}/usb_cam_hw290.yaml"
RVIZ_VIO="${ORB_ROOT}/orb_slam_ros2/config/rviz_hw290_vio.rviz"
RVIZ_SENSORS="${HW290_DIR}/rviz_hw290_sensors.rviz"
[[ -x "$SPLITTER" ]] || { echo "Splitter missing: $SPLITTER (build ov_hw290 first)" >&2; exit 1; }
[[ -r "$IMU_PY" ]] || { echo "IMU bridge missing: $IMU_PY" >&2; exit 1; }
[[ -r "$CALIBRATION" ]] || { echo "Stereo calibration unavailable: $CALIBRATION" >&2; exit 1; }
[[ -r /dev/video0 && -r /dev/ttyUSB0 ]] || { echo "Camera (/dev/video0) or Nano (/dev/ttyUSB0) unavailable" >&2; exit 1; }
[[ -r "${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt" ]] || { echo "ORB vocabulary missing at ${ORB_SLAM3_ROOT}/Vocabulary/ORBvoc.txt" >&2; exit 1; }

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} DISPLAY=${DISPLAY} viewer=${USE_VIEWER} rviz=${SHOW_RVIZ} sensors_only=${SENSORS_ONLY}"
echo "TIP: keep the rig still ~2 s, then move slowly (rotation+translation) facing textured static objects 0.5-2 m away."

PIDS=()
cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "Stopping nodes..."
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  sleep 3
  for p in "${PIDS[@]:-}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  benchmark_summary || true
}
trap cleanup INT TERM EXIT

benchmark_sampler() {
  # 1 Hz sampler: timestamp + %CPU/RSS for the VIO processes + load average.
  echo "# epoch comm pid cpu% rss_kb" > /tmp/orb_vio_bench_ps.log
  while true; do
    now=$(date +%s)
    ps -eo pid,comm,%cpu,rss --sort=-%cpu 2>/dev/null | \
      awk -v t="$now" '$2 ~ /stereo_imu_node|stereo_splitter|usb_cam|hw290|rviz2|orb_slam/ {print t, $2, $1, $3, $4}' \
      >> /tmp/orb_vio_bench_ps.log
    sleep 1
  done
}

benchmark_summary() {
  echo "================ benchmark summary ================"
  echo "--- sensor rates (7 s sample each) ---"
  for t in /cam0/image_raw /imu0; do
    rate=$(timeout 9 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "n/a (no data)")
    echo "$t: $rate"
  done
  if [[ "$SENSORS_ONLY" == false ]]; then
    echo "--- ORB-SLAM3 output rate ---"
    timeout 9 ros2 topic hz /orbslam_vio/path 2>/dev/null | grep -m1 "average rate" || echo "/orbslam_vio/path: n/a (no poses yet - move the rig)"
    echo "--- TrackStereo timing (vio_timing.txt) ---"
    if [[ -r vio_timing.txt ]]; then
      python3 -c "
import numpy as np, sys
d = np.loadtxt('vio_timing.txt')
t = d[:,1] if d.ndim == 2 else d
print(f'frames={len(t)} mean={t.mean():.1f} ms median={np.median(t):.1f} p95={np.percentile(t,95):.1f} max={t.max():.1f} -> effective SLAM rate ~{1000.0/t.mean():.1f} Hz')
"
    else
      echo "vio_timing.txt not written yet (no frames processed or still running from another directory)"
    fi
    ls -lh vio_session.txt kf_vio_session.txt vio_timing.txt 2>/dev/null || true
  fi
  echo "--- CPU/MEM by process (mean over /tmp/orb_vio_bench_ps.log) ---"
  if [[ -r /tmp/orb_vio_bench_ps.log ]]; then
    awk 'NR>1 {cpu[$2]+=$4; rss[$2]+=$5; n[$2]++} END {for (k in n) printf "%-18s samples=%-5d avg_cpu=%6.1f%% avg_rss=%8.1f MB\n", k, n[k], cpu[k]/n[k], (rss[k]/n[k])/1024}' \
      /tmp/orb_vio_bench_ps.log | sort
    echo "host: $(nproc) cores, $(free -h | awk '/Mem:/{print $2}') RAM; load: $(cut -d' ' -f1-3 /proc/loadavg)"
  fi
  echo "full logs: /tmp/orb_vio_{usb_cam,splitter,imu,slam,rviz,bench_ps}.log"
  echo "==================================================="
}

/opt/ros/humble/lib/usb_cam/usb_cam_node_exe --ros-args --params-file "$USB_CAM_PARAMS" > /tmp/orb_vio_usb_cam.log 2>&1 & PIDS+=("$!")
sleep 3
"$SPLITTER" --ros-args -p calibration_file:="$CALIBRATION" -p auto_timestamp_correction:=false > /tmp/orb_vio_splitter.log 2>&1 & PIDS+=("$!")
python3 "$IMU_PY" > /tmp/orb_vio_imu.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 2.74403274 0.11300759 -1.62777045 imu camera_link > /tmp/orb_vio_tf_imu.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0 0 0 0 0 0 camera_link cam0 > /tmp/orb_vio_tf_cam0.log 2>&1 & PIDS+=("$!")
/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0.05989 0 0 0 0 0 camera_link cam1 > /tmp/orb_vio_tf_cam1.log 2>&1 & PIDS+=("$!")

benchmark_sampler > /tmp/orb_vio_bench_sampler.log 2>&1 & PIDS+=("$!")

if [[ "$SENSORS_ONLY" == false ]]; then
  sleep 2
  ros2 launch orb_slam_ros2 stereo_inertial_topics.launch.py \
    namespace:=orbslam_vio viewer:="$USE_VIEWER" \
    > /tmp/orb_vio_slam.log 2>&1 & PIDS+=("$!")
fi
if [[ "$SHOW_RVIZ" == true ]]; then
  sleep 4
  RVIZ_CONFIG="$RVIZ_VIO"
  [[ "$SENSORS_ONLY" == true ]] && RVIZ_CONFIG="$RVIZ_SENSORS"
  rviz2 -d "$RVIZ_CONFIG" > /tmp/orb_vio_rviz.log 2>&1 & PIDS+=("$!")
fi

echo "ORB-SLAM3 HW290 VIO running (sensors_only=${SENSORS_ONLY}). Ctrl-C stops all nodes + prints benchmark."
wait
