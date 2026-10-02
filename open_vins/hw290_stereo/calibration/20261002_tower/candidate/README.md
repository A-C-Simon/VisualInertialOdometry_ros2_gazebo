# Tower calibration candidate, October 2

This fitted profile is **inactive**. It has not passed held-out replay or a
fresh physical movement test. The current mount manifest still rejects live
VIO. Do not treat small fitting residuals as trajectory validation.

Source: `benchmark/results/screen_calibration_20261002_140205_724/`.
The restarted recording includes 95 seconds, 2,856 raw frames per camera and
9,424 timestamped IMU samples. The saved ruler confirmation used 40 mm tags,
12 mm gaps and the same 2560x1440 fullscreen layout as the capture.

Stereo fitting used the full recording at 4 Hz. Camera/IMU fitting used the
first 60 seconds, without importing the old mount rotation. Noise remains
provisional. The remaining approximately 35 seconds are outside the IMU
extrinsic fit; they share the stereo camera calibration and are not independent
ground truth.

| Fitted quantity | Result |
|---|---|
| Stereo baseline | 57.336 mm |
| Cam0 time offset, `t_imu = t_cam + shift` | +14.584 ms |
| Cam1 time offset | +14.990 ms |
| Mean cam0/cam1 reprojection error | 0.342 / 0.388 pixels |
| Mean gyro residual | 0.01449 rad/s |
| Mean accelerometer residual | 0.03650 m/s² |
| IMU origin in raw cam0 | [27.639, -9.326, -54.567] mm |

The baseline is 2.664 mm below nominal CAD spacing. The cam0 IMU origin is
24.808 mm from the CAD board-center prior, mostly in depth. CAD board center
and IMU die center differ, but this discrepancy is not yet explained. Keep
the fit and CAD prior separate rather than forcing either into the other.

The C++ `build_rectified_profile` utility generated the matching rectified
chain and splitter matrices with OpenCV 4.5.4. It rotates each raw camera
frame before inverting `T_cam_imu`. OpenVINS uses cam0's offset for stereo;
the ORB exporter reads that same offset and chain. `fit_review.json` records
provenance, file fingerprints and outstanding checks. `estimator_config.yaml`
retains the previous estimator settings with local calibration paths.

After reviewing replay and geometry, use the explicit mount validation mode
described in [the tower procedure](../README.md). Normal live operation stays
blocked until a matching profile is validated.
