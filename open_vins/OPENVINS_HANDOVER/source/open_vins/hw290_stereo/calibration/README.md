# Camera and IMU calibration

Calibrate each assembled camera/IMU rig before normal operation. The procedure
requires a measured AprilGrid, raw stereo recordings, dynamic IMU data and a
stationary noise measurement.

## Procedure

1. Complete the electrical and acquisition checks in
   [the implementation guide](../OPENVINS_IMPLEMENTATION_GUIDE.md#6-check-acquisition-before-calibration).
2. Measure the target dimensions at the printed or fullscreen size used during
   capture. Save a matching target YAML with each recording.
3. Record stationary IMU data for Allan analysis, stereo views covering both
   image fields, and dynamic roll, pitch, yaw and translation.
4. Build and run the pinned Kalibr and Allan tools using
   [CALIBRATION_TOOLS.md](../CALIBRATION_TOOLS.md).
5. Review residuals, baseline, spatial transforms and time offsets. Generate
   the matching raw and rectified profiles with `build_rectified_profile`.
6. Record file fingerprints in `current_mount.yaml` and complete
   [physical acceptance](../OPENVINS_IMPLEMENTATION_GUIDE.md#9-validate-and-activate-the-new-profile).

## Capture commands

Run from the source workspace:

```bash
# Fullscreen target and raw stereo/IMU recording
./hw290_stereo/run_hw290_openvins.sh --calibration-screen

# Stationary IMU recording for noise analysis
./hw290_stereo/run_hw290_openvins.sh --imu-allan

# Raw acquisition with a printed target
./hw290_stereo/run_hw290_openvins.sh \
  --sensors-only --no-rviz --diagnostics --raw-stereo
```

The fullscreen tool saves the measured target YAML and stops automatically.
For manual printed-target capture, stop with Ctrl-C and create a target YAML
using the measured black tag edge and gap-to-edge ratio.

## Reference calibration

The [tower geometry](20261002_tower/README.md) and
[measured profile](20261002_tower/candidate/README.md) document the reference
assembly. IMU noise coefficients are provisional. Use measured coefficients
when qualifying a new sensor.

`current_mount.yaml` binds the active profile to its chain and stereo-file hashes.
Normal operation requires status `validated`. Use `fitted_pending_validation`
with `--calibration-test` during acceptance; `requires_calibration` permits
raw acquisition but blocks estimation.
