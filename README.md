# VisualInertialOdometry_ros2_gazebo

The OpenVINS source, Gazebo rover simulator, launcher, isolated build outputs,
and documentation are kept in [`open_vins/`](open_vins/).

## HW290 hardware: calibration that stopped the large drift

**Start with [the tested calibration correction](open_vins/hw290_stereo/DRIFT_FIX.md).**
It records the camera/IMU fitting method, the matching rectified image models,
and the physical test that showed no large flights. The
[October 1 profile](open_vins/hw290_stereo/calibration/20261001/README.md) is
already the normal OpenVINS default for the unchanged mount.

Git reference: `hw290-openvins-drift-fix-20261001`, calibration commit `0c9cf22`.
The previous-model replay reached 18.92 km; the fitted model stayed below
0.47 m. The fresh desk test stayed below 0.852 m displacement.

Quick hardware run with RViz:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./hw290_stereo/run_hw290_openvins.sh
```

Hold the rig still until initialization, then move. Ctrl+C saves the benchmark
summary. Changing the mount requires repeating the documented calibration.
The ORB hardware launcher now exports the same measured profile and timing
offset. Its October 2 desk test stayed at desk scale but still reset during
inertial initialization. See [ORB hardware commands and status](ORB_SLAM/README.md#use-the-same-measured-calibration-as-openvins).

## Gazebo simulation

Start the full simulated visual inertial odometry test with:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./run_vio_gazebo.sh
```

See [`open_vins/README.md`](open_vins/README.md) for build instructions,
sensor topics, trajectory ground truth, alignment, diagnostics, and launcher
modes. Work reports are kept separately in `/home/ac/Work_Reports`.
