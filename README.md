# VisualInertialOdometry_ros2_gazebo

The OpenVINS source, Gazebo rover simulator, launcher, isolated build outputs,
and documentation are kept in [`open_vins/`](open_vins/).

## HW290 hardware: current tower mount

The camera, IMU and Nano moved into a rigid printed tower on October 2.
[Current geometry and calibration status](open_vins/hw290_stereo/calibration/20261002_tower/README.md)
record the actual OpenSCAD output. Camera/IMU rotation is unverified and 20 ms
is only a timing starting estimate. Both hardware launchers require a fitted,
validated chain for this mount before live VIO. Record new raw calibration data:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./hw290_stereo/run_hw290_openvins.sh --calibration-screen
```

## Previous mount: calibration that stopped the large drift

[The tested calibration correction](open_vins/hw290_stereo/DRIFT_FIX.md) records
the repeatable fitting method and evidence for the earlier mount. Its
[October 1 profile](open_vins/hw290_stereo/calibration/20261001/README.md) is
preserved for historical replay. Git reference:
`hw290-openvins-drift-fix-20261001`, calibration commit `0c9cf22`.
The previous-model replay reached 18.92 km; the fitted model stayed below
0.47 m. The fresh desk test stayed below 0.852 m displacement.

ORB received that profile before the tower change, but initialization and a
live refinement jump remain unresolved. See
[ORB hardware commands and status](ORB_SLAM/README.md#use-the-same-measured-calibration-as-openvins).

## Gazebo simulation

Start the full simulated visual inertial odometry test with:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./run_vio_gazebo.sh
```

See [`open_vins/README.md`](open_vins/README.md) for build instructions,
sensor topics, trajectory ground truth, alignment, diagnostics, and launcher
modes. Work reports are kept separately in `/home/ac/Work_Reports`.
