# VisualInertialOdometry_ros2_gazebo

The OpenVINS source, Gazebo rover simulator, launcher, isolated build outputs,
and documentation are kept in [`open_vins/`](open_vins/).

## HW290 hardware: current tower mount

The camera, IMU and Nano share the rigid tower assembled on October 2.
The [selected tower profile](open_vins/hw290_stereo/calibration/20261002_tower/candidate/README.md)
passed a confirmed two-minute desk check with no flights, maximum displacement
0.893 m. Reproduction requires the measured camera/IMU fit, the 1,000-message
IMU queue and the camera queue lock release before visual processing.
Both launchers check the selected calibration against the current mount.

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./hw290_stereo/run_hw290_openvins.sh
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
```

ORB uses the same measured profile but still has initialization resets.
The desk check is not independent trajectory accuracy. Changing the mount
requires repeating [calibration and validation](open_vins/hw290_stereo/calibration/20261002_tower/README.md).

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
