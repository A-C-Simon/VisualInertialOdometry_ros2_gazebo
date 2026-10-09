# OpenVINS ROS 2 rover test

This directory contains the OpenVINS ROS 2 packages and the Gazebo rover test
used for camera plus IMU visual inertial odometry. OpenVINS uses a bounded
MSCKF estimator. It does not run loop closure, place recognition, a persistent
global map, or Pangolin.

## Physical OpenVINS engineering handover

For Ubuntu 22.04 setup, run as your normal account:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

This installs dependencies, builds the hardware packages and patched camera
driver, and prepares firmware/conversion tools and calibration containers.
Omit `--with-calibration-tools` for the smaller runtime and firmware tool setup.
Use `--dry-run` for a preview or `--check` to inspect software readiness.
Firmware upload and calibration require the actual rig and follow the guide.

Use [the implementation guide](hw290_stereo/OPENVINS_IMPLEMENTATION_GUIDE.md)
to build the working stereo/IMU pipeline on a clean host and calibrate another
device. It covers wiring, firmware, dependencies, timestamp and transform
conventions, the retained delivery fixes, validation and diagnostics.
[Offline calibration tools](hw290_stereo/CALIBRATION_TOOLS.md) includes the
Docker source build, bag conversion and noise analysis. The supplied tower
calibration is specific to that physical assembly.

## HW290 calibration that stopped the large drift

**For the physical rig, start with [the tested drift correction](hw290_stereo/DRIFT_FIX.md).**
The normal hardware launcher selects the measured October 2 rigid-tower profile.
The October 1 measurements below describe the earlier mount.
The previous-model replay reached 18.92 km; the fitted model stayed below
0.47 m, and a fresh desk test stayed below 0.852 m with no reported flights.
The guide records the camera/IMU calibration method, matching rectification,
evidence, limitations and quick launch commands.

```bash
./hw290_stereo/run_hw290_openvins.sh
```

Git reference: `hw290-openvins-drift-fix-20261001` (`0c9cf22`). Reuse the
[dated profile](hw290_stereo/calibration/20261001/README.md) for the unchanged
earlier mount; use the current tower profile for live operation and repeat
calibration when the mount changes.

## Selected ORB tower profile

```bash
./hw290_stereo/run_selected_orb.sh --rviz
```

The selected 600-feature, scale-1.6/five-level packed core passed the confirmed
two-minute tower test without observed flights, jumps or resets. See
[the selection, measurements and limits](hw290_stereo/ORB_SELECTED_PROFILE.md).
The cheapest public candidates retain mixed accuracy results. General equality
to OpenVINS CPU with preserved quality has not been established.

## Layout

| Path | Purpose |
|---|---|
| `ov_msckf/` | ROS 2 OpenVINS estimator node |
| `ov_rover_sim/` | Gazebo rover, sensors, motion, truth, and RViz2 setup |
| `ov_rover_sim/config/rover_stereo/` | Camera, IMU, and estimator calibration |
| `benchmark/` | EuRoC replay, resource measurement, ATE tools, and comparison report |
| `hw290_stereo/DRIFT_FIX.md` | Tested physical calibration correction and how to repeat it |
| `hw290_stereo/calibration/20261002_tower/` | Current measured tower camera/IMU profile and validation |
| `hw290_stereo/calibration/20261001/` | Historical calibration for the previous mount |
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

The common EuRoC stereo-inertial comparison report, including accuracy and
estimator CPU and memory results, is kept locally at
`/home/ac/Work_Reports/2026-09-25_VIO_Benchmark_Report.md`.
Work reports are stored outside this repository.

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

## Hardware VIO

The HW290 stereo launch instructions and current status are in
[hw290_stereo](hw290_stereo/README.md). The hardware report links there cover
sensor calibration, the IMU bridge and benchmark results.
