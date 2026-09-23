# OpenVINS ROS 2 rover test

This directory contains the OpenVINS ROS 2 packages and the Gazebo rover test
used for camera plus IMU visual inertial odometry. OpenVINS uses a bounded
MSCKF estimator. It does not run loop closure, place recognition, a persistent
global map, or Pangolin.

## Layout

| Path | Purpose |
|---|---|
| `ov_msckf/` | ROS 2 OpenVINS estimator node |
| `ov_rover_sim/` | Gazebo rover, sensors, motion, truth, and RViz2 setup |
| `ov_rover_sim/config/rover_stereo/` | Camera, IMU, and estimator calibration |
| `run_vio_gazebo.sh` | Full simulation launcher |
| `build_vio/` | Isolated colcon build output |
| `install_vio/` | Isolated ROS 2 install space |
| `log_vio/` | Isolated colcon logs |

## Build

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
source /opt/ros/humble/setup.bash
colcon --log-base log_vio build \
  --base-paths . \
  --build-base build_vio \
  --install-base install_vio \
  --symlink-install \
  --packages-select ov_core ov_data ov_eval ov_init ov_msckf ov_rover_sim
source install_vio/setup.bash
```

The isolated install prevents this test from resolving packages in the older
`/home/ac/ros2_ws` workspace.

## Run

The default run starts automatic rover motion, Gazebo GUI, RViz2, and OpenVINS
with the left camera and IMU. Mono plus IMU is the lower-cost configuration.

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./run_vio_gazebo.sh
```

Available modes are `--stereo`, `--mono`, `--teleop`, `--headless`, `--no-rviz`,
`--no-vio`, and `--square`. Stereo uses both simulated cameras. Headless mode
keeps Gazebo running without its window and disables RViz2.

When `--stereo --teleop` is selected, the launcher first drives a short gentle
arc and checks the OpenVINS log for successful initialization. Keyboard control
starts only after that check passes. If the first arc is insufficient, a second
short arc is attempted. Teleop is withheld if initialization still fails, which
prevents an invalid VIO path from being treated as a usable trajectory.

The launcher permits one rover simulation at a time, removes orphaned processes
from an interrupted run, and assigns Gazebo a master port based on
`ROS_DOMAIN_ID`. Before OpenVINS starts, a direct ROS 2 subscriber verifies
`/gazebo/model_states`, `/odom`, `/imu0`, and the required camera streams. A
missing rover or sensor therefore stops the launch with a specific error instead
of leaving RViz with an incomplete TF tree.

## Topics and frames

The rover publishes `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`,
`/cam0/camera_info`, `/cam1/camera_info`, and `/gazebo/model_states`. The IMU
runs at 200 Hz. The cameras run at 20 Hz and produce 640 by 400 images.

OpenVINS publishes in the `/ov_msckf` namespace:

```text
/ov_msckf/poseimu
/ov_msckf/pathimu
/ov_msckf/odomimu
/ov_msckf/trackhist
/ov_msckf/points_msckf
```

`ov_rover_sim/scripts/ground_path.py` selects the `ov_rover` entry from
`/gazebo/model_states` and publishes `/ov_msckf/posegt` and `/ov_msckf/pathgt`
in the `world` frame. This is the Gazebo model pose, not wheel odometry.
`align_frames.py` timestamp-matches the truth and VIO poses, estimates the
initial yaw and translation, and publishes the `global` to `world` transform.
The same ground truth node publishes the dynamic `world` to `odom` transform,
which connects Gazebo's diff-drive TF to the robot_state_publisher tree. This
allows RViz to resolve `global` to `base_link` and display the robot model.

The world state plugin is enabled in `ov_rover_sim/worlds/small_room.world` so
that Gazebo provides the true model state.

## Calibration and estimator settings

The rover calibration files are in `ov_rover_sim/config/rover_stereo/`. The
estimator configuration controls the bounded clone and feature state. The
launcher gives stereo a stationary initialization interval, then starts the
automatic drive after both cameras and the IMU are subscribed. This avoids the
earlier startup race. The revised stereo run received both camera streams and
completed initialization successfully. Mono plus IMU remains the lower-cost
default.

## Trajectory comparison result

After switching the reference to true Gazebo model states and applying the
timestamped initial frame alignment, a live comparison produced 356 matched
pose pairs over 25 seconds. The best-fit planar residual was 0.152 m RMS and
0.270 m maximum. The estimated and true path spans were approximately 4.24 m
by 3.52 m and 3.87 m by 3.39 m. The larger error seen with `/odom` was caused
by comparing against wheel odometry rather than the rover model state.

The alignment correction removes the artificial origin and heading mismatch.
The remaining residual is VIO drift and should be evaluated separately from
frame alignment.

## Diagnostics

```bash
source /opt/ros/humble/setup.bash
source /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins/install_vio/setup.bash
ros2 topic list
ros2 topic hz /cam0/image_raw
ros2 topic hz /imu0
ros2 topic echo /ov_msckf/poseimu
ros2 topic echo /ov_msckf/pathgt
```

The launcher writes `/tmp/ov_msckf_vio.log` and
`/tmp/vio_gazebo_launch.log`. Press Ctrl+C to stop the test.
