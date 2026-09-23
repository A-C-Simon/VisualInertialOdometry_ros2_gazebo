# Visual inertial odometry workspace

This repository contains the ROS 2 OpenVINS estimator and the Gazebo rover
simulation used to test camera plus IMU odometry. ORB-SLAM3 is kept in the
`ORB_SLAM/` directory for comparison, but it is not required by the VIO test.

## VIO architecture

```text
Gazebo stereo camera or ELP camera
              +
          IMU measurements
              |
              v
          OpenVINS MSCKF
              |
              v
       odometry, path, and TF
```

OpenVINS maintains a bounded estimator state. It does not run loop closure,
place recognition, a persistent global map, or Pangolin. Temporary feature
tracks are used by the estimator and are removed as the sliding window moves.

## Repository layout

| Path | Purpose |
|---|---|
| `open_vins/` | OpenVINS source packages |
| `open_vins/ov_msckf/` | ROS 2 VIO estimator node |
| `open_vins/ov_rover_sim/` | Gazebo rover, sensors, driving, and RViz2 setup |
| `open_vins/ov_rover_sim/config/rover_stereo/` | Rover camera, IMU, and estimator calibration |
| `run_vio_gazebo.sh` | Isolated VIO simulator launcher |
| `ORB_SLAM/` | Separate ORB-SLAM3 comparison work |
| `calibration/` | ELP calibration utilities and outputs |

## Build

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo
source /opt/ros/humble/setup.bash
colcon --log-base log_vio build \
  --base-paths open_vins \
  --build-base build_vio \
  --install-base install_vio \
  --symlink-install \
  --packages-select ov_core ov_data ov_eval ov_init ov_msckf ov_rover_sim
source install_vio/setup.bash
```

The isolated install space is used so it does not resolve packages from the
older `/home/ac/ros2_ws` workspace.

## Run the VIO simulation

The default mode uses the left camera and IMU. It avoids the current stereo
calibration issue and uses less computation:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo
./run_vio_gazebo.sh
```

This starts automatic rover motion, Gazebo GUI, RViz2, and OpenVINS. The
estimator subscribes to `/cam0/image_raw` and `/imu0`.

Use both cameras explicitly:

```bash
./run_vio_gazebo.sh --stereo
```

Use keyboard control:

```bash
./run_vio_gazebo.sh --teleop
```

Use a low-resource run without Gazebo or RViz2 windows:

```bash
./run_vio_gazebo.sh --headless
```

The simulator can be started without OpenVINS for sensor inspection:

```bash
./run_vio_gazebo.sh --no-vio
```

## Sensor topics

The rover publishes:

```text
/cam0/image_raw
/cam1/image_raw
/cam0/camera_info
/cam1/camera_info
/imu0
/odom
```

The IMU runs at 200 Hz. The cameras run at 20 Hz and produce 640 by 400
images. The estimator uses simulated time.

## OpenVINS outputs

The node runs in the `/ov_msckf` namespace and publishes:

```text
/ov_msckf/poseimu
/ov_msckf/pathimu
/ov_msckf/odomimu
/ov_msckf/trackhist
/ov_msckf/points_msckf
```

`/ov_msckf/pathimu` is the estimated inertial trajectory. `/ov_msckf/pathgt`
is the simulator ground truth path used for comparison. RViz2 displays these
paths and the estimator diagnostics.

## Calibration

The rover calibration is in
`open_vins/ov_rover_sim/config/rover_stereo/`:

* `estimator_config.yaml` contains estimator settings.
* `kalibr_imucam_chain.yaml` contains camera intrinsics and camera to IMU
  transforms.
* `kalibr_imu_chain.yaml` contains IMU noise and IMU frame settings.

The simulated IMU frame is colocated with the rover body frame. The camera
optical axes point forward, with a 0.11 m stereo baseline along the rover
lateral axis.

## Computational profile

This VIO path has no global mapping or loop closure. The main continuing costs
are feature tracking, IMU propagation, and the bounded MSCKF update. The state
size is controlled by `max_clones`, `max_slam`, and feature limits in the
estimator configuration.

The mono plus IMU mode is the default low-cost configuration. Stereo can add
geometric constraints, but it requires the camera extrinsics and topic pairing
to be correct. A first test of the stereo mode showed successful sensor
delivery but unstable initialization, so it remains an explicit diagnostic
option while the calibration is reviewed.

## Diagnostics

```bash
source /opt/ros/humble/setup.bash
source /home/ac/VisualInertialOdometry_ros2_gazebo/install_vio/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
ros2 topic hz /cam0/image_raw
ros2 topic hz /imu0
ros2 topic echo /ov_msckf/poseimu
ros2 topic echo /ov_msckf/pathimu
```

OpenVINS prints initialization status and timing to `/tmp/ov_msckf_vio.log`
when launched by `run_vio_gazebo.sh`. A healthy run reports successful
initialization and timing near the incoming sensor rates. Large position
growth while Gazebo ground truth remains near the rover indicates an
extrinsic, time, or initialization problem and should not be treated as valid
odometry.

## Stopping the test

Press Ctrl+C in the launcher terminal. The launch script stops the estimator
and simulator process it started. OpenVINS currently reports a segmentation
fault during some ROS 2 shutdown sequences after Ctrl+C, although the process
runs and publishes normally while active. This shutdown issue is separate
from the estimator output and remains to be corrected.
