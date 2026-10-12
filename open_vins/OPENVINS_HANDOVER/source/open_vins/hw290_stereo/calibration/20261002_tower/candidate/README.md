# Reference tower calibration

This profile defines the measured camera/IMU model for the rigid ELP stereo,
HW290 and Nano tower. The `candidate` directory is the active profile path
used by the supplied launcher. Assembly-specific validation is recorded in
`current_mount.yaml`.

## Calibration data and fit

The capture contains 95 seconds of data, 2,856 raw frames per camera and 9,424
IMU samples. The measured fullscreen target uses 40 mm black tag edges and
12 mm gaps. Stereo fitting used the full recording at 4 Hz; camera/IMU fitting
used the first 60 seconds.

| Quantity | Result |
| --- | --- |
| Stereo baseline | 57.33584 mm |
| Cam0 time offset, `t_imu = t_cam + shift` | +14.58435 ms |
| Cam1 time offset | +14.99032 ms |
| Mean cam0/cam1 reprojection error | 0.342 / 0.388 pixels |
| Mean gyro residual | 0.01449 rad/s |
| Mean accelerometer residual | 0.03650 m/s² |
| IMU origin in raw cam0 | [27.639, -9.326, -54.567] mm |

`build_rectified_profile` generated the rectified chain and splitter matrices
with OpenCV 4.5.4. The conversion rotates each raw camera frame before
inverting `T_cam_imu`; stereo estimation uses cam0's fitted time offset.
`fit_review.json` records file fingerprints and fit details.

## Profile files

| File | Purpose |
| --- | --- |
| `camchain_raw.yaml` | Kalibr raw camera/IMU result |
| `camchain.yaml` | Matching rectified camera/IMU model |
| `stereo_opencv.yaml` | Raw lens parameters and rectification matrices |
| `imu.yaml` | OpenVINS IMU noise and intrinsic model |
| `estimator_config.yaml` | Estimator settings with local profile paths |

## Physical validation

Continuous movement was tested for 120.052 seconds within a
1.40 x 0.80 x 0.60 m workspace. The trajectory remained within the workspace
bound, with no covariance failure.

| Metric | Result |
| --- | --- |
| Maximum displacement from initial saved pose | 0.89328 m |
| Maximum consecutive position change | 0.04777 m |
| Maximum pose timestamp interval | 36.170 ms |
| Maximum estimator IMU timestamp interval | 21.974 ms |
| Estimator average CPU | 75.1% of one core |
| Estimator peak RSS | 456.0 MiB |
| Mean / p95 tracking-update time | 18.75 / 25.50 ms |

The run used drivers, RViz, recording and debug logging. CPU and memory values
refer to the estimator process. These measurements establish bounded motion
for this test; independent ground truth is required for an accuracy score.

## Qualification requirements

The fitted IMU origin differs by 24.808 mm from the CAD board-center prior;
the full discrepancy is unresolved. IMU noise coefficients and intrinsic
correction matrices remain provisional. Measure these quantities when
qualifying another assembly.

Retain the sensor queue and callback requirements in
[DRIFT_FIX.md](../../../DRIFT_FIX.md). Recalibrate after a physical mount or
camera-mode change, then follow [the acceptance procedure](../README.md).
