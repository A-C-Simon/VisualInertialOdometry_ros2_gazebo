# Current HW290 tower mount, October 2

The camera, IMU and Nano now share the rigid printed tower described in
[vio_rig_stand_simple.scad](../../vio_rig_stand_simple.scad). The [selected tower profile](candidate/README.md) passed a confirmed two-minute
desk check after both ROS delivery fixes. The previous
October 1 camera/IMU fit belongs to a different mount. It is retained as
historical calibration and must not be used for live VIO on this tower.

## Geometry extracted with OpenSCAD

```bash
QT_QPA_PLATFORM=offscreen openscad -o /tmp/hw290_tower_calib.csg \
  hw290_stereo/vio_rig_stand_simple.scad
```

The saved [CLI echoes](openscad_echoes.txt) give these nominal CAD priors:

| Quantity | Value |
|---|---|
| Stereo baseline | 0.060 m |
| IMU board center in cam0 optical coordinates | [0.030, 0, -0.0317] m |
| IMU board center in cam1 optical coordinates | [-0.030, 0, -0.0317] m |
| Camera/IMU rotation | Unverified |
| Camera/IMU time shift starting estimate | +0.020 s |

[geometry.yaml](geometry.yaml) records the source fingerprint and conventions.
CAD locates the breakout board center, not a surveyed IMU die center. The
baseline is nominal CAD spacing, not a replacement for the measured stereo
baseline. The 20 ms offset must be fitted with the new recording.

The optical-frame lever arms are not the translation column of `T_imu_cam`,
which is expressed in IMU axes. No numerical transform is generated while
the rotation is unknown. Rectification also changes the camera coordinate
frame; use a consistent raw fit and rectified chain.

## Calibration and activation

Record raw stereo and timestamped IMU with the new tower rigidly assembled:

```bash
./hw290_stereo/run_hw290_openvins.sh --calibration-screen
```

Follow the [offline procedure](../README.md). Verify the fullscreen tag size,
fit stereo geometry and then camera/IMU rotation, translation and time shift.
If the stereo assembly is unchanged, its existing intrinsic fit remains a
candidate, but verify image mode and reprojection rather than replacing its
baseline with the CAD value. Inspect IMU continuity and held-out motion, then
repeat a fresh continuous movement test before activation.

[current_mount.yaml](../current_mount.yaml) currently says
`validated` for the [selected profile](candidate/README.md), with SHA256
fingerprints of its camera/IMU chain and stereo calibration.
Both hardware launchers reject normal live VIO with mismatching files. The C++ checker verifies those files at startup.
After inspecting a fresh fit, set `fitted_pending_validation` and its matching
chain/stereo fingerprints. Use the explicit validation mode for the desk test:

```bash
./hw290_stereo/run_hw290_openvins.sh --calibration-test --diagnostics
# ORB uses the same explicit mode after exporting that fitted profile:
../ORB_SLAM/orbslam3_hw290_vio.sh --calibration-test --efficient --diagnostics --rviz
```

Select the fitted files with `HW290_VIO_CONFIG` and
`HW290_STEREO_CALIBRATION`. This mode still rejects `requires_calibration` and
incorrect fingerprints. Mark `validated` only after the movement checks pass.
Normal runs reject `fitted_pending_validation`; trial runs save a
`calibration_test` marker. Raw recording remains available, and unverified
camera/IMU TF is omitted.
Keep the previous profiles and evidence for replay of the old mount.
