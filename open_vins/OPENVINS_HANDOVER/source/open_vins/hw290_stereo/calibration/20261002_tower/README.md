# Reference tower geometry and calibration

The ELP stereo camera, HW290 breakout and Nano share the rigid mount defined in
[vio_rig_stand_simple.scad](../../vio_rig_stand_simple.scad). The rig frame uses
X right, Y forward and Z up, with its origin at the base center.

## Nominal mechanical geometry

| Quantity | CAD value |
| --- | --- |
| Stereo baseline | 60 mm |
| Lens centers in rig frame | (-30, 15, 206) and (30, 15, 206) mm |
| IMU board center in rig frame | (0, -16.7, 206) mm |
| IMU board center in cam0 optical frame | (30, 0, -31.7) mm |
| IMU board center in cam1 optical frame | (-30, 0, -31.7) mm |

Generate the geometry with OpenSCAD:

```bash
QT_QPA_PLATFORM=offscreen openscad -o /tmp/hw290_tower_calib.csg \
  hw290_stereo/vio_rig_stand_simple.scad
```

[geometry.yaml](geometry.yaml) records conventions and the CAD fingerprint;
[openscad_echoes.txt](openscad_echoes.txt) contains the calculator output.
The board-center position is a mechanical prior. Determine camera/IMU rotation,
translation and timing from sensor calibration, including the effect of camera
rectification on the coordinate frame.

## Measured profile

The [reference calibration](candidate/README.md) contains the fitted raw chain,
rectified chain, splitter matrices, IMU model and estimator configuration.
The measured stereo baseline is 57.33584 mm and cam0 time offset is +14.58435 ms.
Use these values only for the physically unchanged reference assembly.

## Calibrating another assembly

Follow the [calibration procedure](../README.md) and review residuals and physical
geometry before acceptance. Set `current_mount.yaml` to `requires_calibration`
for a new mount. After fitting, set `fitted_pending_validation` with matching
chain and stereo hashes and run:

```bash
./hw290_stereo/run_hw290_openvins.sh --calibration-test --diagnostics
```

Select the fitted files with `HW290_VIO_CONFIG` and `HW290_STEREO_CALIBRATION`.
Set status `validated` after the movement checks pass. Metadata does not select
profile paths; configure both the estimator and splitter explicitly.
