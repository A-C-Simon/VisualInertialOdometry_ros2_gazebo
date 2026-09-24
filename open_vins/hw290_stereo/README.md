# HW-290 and ELP stereo OpenVINS test

This directory runs the ELP side-by-side stereo camera and an HW-290 MPU6050
through an Arduino Nano. The ROS2 camera stream is split by the compiled
`ov_hw290/stereo_splitter` node. The older `stereo_splitter.py` is not used by
the launcher.

## Current status

The camera and IMU streams work. On the connected hardware, `/image_raw` ran
at about 26 Hz, `/cam0/image_raw` at about 25 Hz, and `/imu0` at about 100 Hz.
The splitter was not the main cause of the low frame rate. The ROS2 estimator
initializes and publishes `/ov_msckf/pathimu` and `global -> imu`, but its
trajectory is **not accurate** in the current desk test. The first run
accumulated about 15 metres on a desk less than one metre wide. A later run
diverged beyond 70 metres. Do not use this path as odometry yet.

The camera and HW-290 were rigidly attached on 2026-09-24. A rotation-only
motion capture gave a provisional camera-to-IMU rotation and a 0.155 second
time shift. The estimate came from one usable homography-based capture with a
0.64 degree median rotational residual. A second independent validation
capture did not contain usable motion, so repeatability is not established.
The camera-to-IMU translation remains an assumed zero at cam0. The static TF
and `T_imu_cam` values use this provisional estimate. Both image streams are
now rectified with the measured five-coefficient stereo calibration before
OpenVINS receives them. The input calibration is from 1600x1200 per eye and is
scaled to the 640x480 runtime mode. Online camera extrinsic calibration is
disabled for this test because it moved the known 59.9 mm stereo baseline to
an implausible value during a failed run.
`ov_msckf/src/core/VioManager.cpp` was changed to retain a full initialization
window of IMU data with positive camera-to-IMU time offsets. Without that
change, the 0.155 second offset caused the static initializer to discard too
many samples and report that its IMU window was too short indefinitely.
The live stereo rectification check found 247 to 376 descriptor matches with
about 1.0 to 1.2 pixels median vertical mismatch. An earlier short run
produced 0.68 metres accumulated travel and ended roughly 0.13 metres from
its start, but a larger motion still diverged. DEBUG logs showed many frames
with zero MSCKF feature updates and only about 15 to 21 retained SLAM
features. A captured left image showed a nearby moving person and hands across
much of the view while most fixed texture was distant. That is unsuitable for
evaluating VIO: moving foreground objects violate the static-scene model and
the distant background gives weak depth with this stereo baseline. Repeat the
test with both lenses pointed at fixed, textured objects around 0.5 to 2 m
away and with people and hands outside the images.
After aiming away from the moving foreground, a stationary test kept the
estimated position within about 2 cm of its start. The cumulative distance
counter still rose because it sums millimetre-scale pose jitter at each
update; it is not a measure of physical travel while the rig is at rest.
Controlled translation across the desk has not yet been measured in this
static-scene view.

## Run sensor diagnostics

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins/hw290_stereo
ROS_DOMAIN_ID=48 ./run_hw290_openvins.sh --sensors-only
```

Use `--no-rviz` for command-line measurements. In another terminal, use the
same `ROS_DOMAIN_ID` for each command:

```bash
ROS_DOMAIN_ID=48 ros2 topic hz /image_raw
ROS_DOMAIN_ID=48 ros2 topic hz /cam0/image_raw
ROS_DOMAIN_ID=48 ros2 topic hz /imu0
ROS_DOMAIN_ID=48 ros2 run tf2_ros tf2_echo imu cam0
```

The shell prints the effective ROS domain. If an existing `ROS_DOMAIN_ID` is
exported, it overrides the default of 48. The IMU message frame is `imu`,
matching the TF tree. Sensor-only RViz uses `cam0` as its fixed frame. The
OpenVINS RViz configuration uses `global` so the trajectory is not drawn in
the moving camera frame. `global` appears only when OpenVINS initializes.

## Required before evaluating VIO

1. Keep the camera and HW-290 rigidly attached. Do not change their relative
   position while running or calibrating.
2. Calibrate both camera intrinsics at the selected 1280x480 side-by-side
   video mode, the stereo baseline, camera-to-IMU rotation and translation,
   and camera-to-IMU time offset. The current camera intrinsics were scaled
   from a higher-resolution calibration and should be checked at this mode.
3. Update `kalibr_imucam_chain.yaml` and `kalibr_imu_chain.yaml` with the
   measured values. Replace the launcher static TF placeholders with the
   calibrated transforms.
4. Hold the fixed rig still during initialization, then move it through a
   textured, well-lit scene. Check that the path remains still when the rig
   is still and that a short out-and-back motion returns near its start.

`calibrate_rotation.py` is an experimental rotation-only check for a rigid
assembly, not a substitute for the full calibration in step 2. The plain
`./run_hw290_openvins.sh` command starts OpenVINS and prints a provisional
calibration warning until that work is complete.

`check_rectified_stereo.py` measures left-right epipolar mismatch from a live
pair. To inspect tracker update counts, temporarily set `verbosity: "DEBUG"`
in `estimator_config.yaml`; return it to `INFO` for normal runs.
