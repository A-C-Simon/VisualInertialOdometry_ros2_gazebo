# Tower calibration candidate, October 2

This fitted profile **failed its first physical validation**. It passed the recorded
OpenVINS consistency check below. Normal live VIO remains blocked; explicit
`--calibration-test` accepts the matching fingerprints for a validation run.
Do not treat small fitting residuals as trajectory validation.

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

## Recorded OpenVINS check

The complete 95-second capture replayed at half speed without flight or reset.
OpenVINS saved 2,681 poses over 89.28 seconds, maximum displacement 0.27522 m
and maximum consecutive position change 0.02683 m. After a rigid alignment
without fitting scale on the first 60 seconds, the 68 target-reference samples
beyond 60 seconds differed by 0.02007 m RMS and at most 0.10747 m. The target
reference shares camera data and calibration; it is not independent ground
truth. The CAD mismatch and fresh physical movement check remain outstanding.

To run this candidate explicitly from `open_vins`:

```bash
HW290_VIO_CONFIG="$PWD/hw290_stereo/calibration/20261002_tower/candidate/estimator_config.yaml" \
HW290_STEREO_CALIBRATION="$PWD/hw290_stereo/calibration/20261002_tower/candidate/stereo_opencv.yaml" \
./hw290_stereo/run_hw290_openvins.sh --calibration-test --diagnostics
```

## First physical check failed

The October 2 live run `hw290_openvins_20261002_144009_LinLH5` reached
15.856 m despite confirmed movement within the desk limits. The estimator
exited after negative covariance entries. Its IMU subscription reported gaps
up to 680.785 ms, while the recorded source contained 6,429 samples at 99.019 Hz
with a maximum interval of 21.523 ms and no intervals above 25 ms. This exposes
a delivery failure before the flight; it does not validate the calibration.
An ORB replay was running in another ROS domain, so this is not an isolated
compute measurement. Correct delivery and replay this failure recording before
repeating live validation. Normal activation remains blocked.

## Repeat with complete IMU delivery

The two-minute repeat `hw290_openvins_20261002_145045_4UIsx8` received 20,503
IMU samples without intervals above 50 ms, maximum 21.870 ms, and without
negative covariance entries. Confirmed desk motion nevertheless produced one
sharp 1.378 m step at 74.599 seconds after the cue, reaching 2.197 m maximum
displacement. The remainder followed the motion without another large jump.

Recorded stereo header intervals around that event stayed below 36.065 ms.
OpenVINS camera updates instead skipped from 1790942016.308 to
1790942018.308 seconds after a 1.429-second processing stall. Its update worker
held the camera queue lock across tracking and visualization, blocking image
callbacks. A change to pop a ready frame under the lock and process it after
releasing the lock is being checked on the saved motion segment. This repeat
still fails the desk bound; normal activation remains blocked.
