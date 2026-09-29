# HW290 and ELP stereo VIO

The hardware pipeline uses a side by side ELP stereo camera, an HW290 IMU
connected through an Arduino Nano, the OpenVINS estimator and an optional
ORB-SLAM3 estimator. The IMU reports identity 0x98, consistent with an
ICM-20689. The current camera and IMU mount calibration is provisional.

## Run

From `open_vins`:

```bash
./hw290_stereo/run_hw290_openvins.sh
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
```

RViz opens by default for OpenVINS and when requested with `--rviz` for ORB.
Both launchers keep a per-run CPU, memory and estimator summary under
`benchmark/results/`. OpenVINS selects the C++ IMU reader by default;
`HW290_IMU_BACKEND=python` selects the fallback. Build the ROS package with
the following command if the executable is missing:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ov_hw290 --build-base build_vio \
  --install-base install_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Keep the camera and IMU rigidly attached. Hold the rig still for initialization,
then move slowly through a static, textured scene with objects about 0.5 to 2 m
away. Do not assess accuracy from a stationary cumulative path length. Use a
measured motion and return test, and compare against ground truth when available.

## Current hardware status, 2026-09-29

Earlier, a 20 cm out and back test reached 21.1 cm estimated displacement and
returned within 2.46 cm. Later failures showed the IMU rate dropping to about
14 Hz instead of 100 Hz. OpenVINS then diverged and ORB-SLAM3 lost tracking and
reset. The firmware and both IMU readers now verify the acquisition and
delivery rates and stop VIO on a rate fault. After reconnecting USB, the source
reported about 100 Hz for more than three minutes with no packet gaps, checksum
errors, saturation or I2C read errors. A movement test after this change is
still required, so long duration trajectory stability is unconfirmed.

## Diagnostics and calibration

See [repair notes](REPAIR_NOTES.md) for the single source of calibration,
firmware and camera timestamp details, known limitations and reproduction
commands. See [the C++ bridge report](../benchmark/2026-09-29_CPP_IMU_Bridge.md)
for protocol and CPU measurements, and [the optimization report](../benchmark/2026-09-28_HW290_ORB_Optimization.md)
for the EuRoC comparison and ORB settings.

When the IMU fails startup, the launchers keep RViz available in sensor view
and skip VIO until the sensor is restored. During a run, a detected IMU rate or
connection failure stops the estimator and saves the summary.
