# Calibration and sensor delivery requirements

OpenVINS combines image feature measurements with IMU propagation. A valid
camera/IMU model and continuous measurement delivery are both required for
stable estimation. This document specifies the integration requirements for
the ELP stereo, HW290 and Nano pipeline.

## Calibration consistency

Keep these files together for each physical assembly:

| File | Function |
| --- | --- |
| `camchain_raw.yaml` | Raw camera intrinsics and fitted camera/IMU transforms |
| `stereo_opencv.yaml` | Raw lens models and stereo rectification matrices |
| `camchain.yaml` | Rectified intrinsics and camera-to-IMU transforms |
| `imu.yaml` | IMU noise coefficients and intrinsic correction matrices |
| `estimator_config.yaml` | Estimator options and paths to the selected models |

The [reference profile](calibration/20261002_tower/candidate/README.md) records
57.33584 mm stereo baseline and +14.58435 ms cam0 time offset. Its numerical
values apply only to the measured assembly. Recalibrate after changing the
mount, camera mode, lens configuration or timestamp source.

Rectification rotates the camera coordinate frame. Apply that rotation to
Kalibr's raw `T_cam_imu` before inversion to OpenVINS `T_imu_cam`. Use the
matching rectified projection intrinsics and zero distortion in the estimator.
Static camera TF and estimation must read the same chain.

## Timestamp handling

The patched `usb_cam` driver supplies capture timestamps. The Nano timestamps
IMU acquisition; the C++ bridge maps those timestamps to the host clock and
converts measurements to SI units. Apply cam0's fitted time shift once, in
the estimator. A second arrival-time correction changes the calibrated model.

Keep camera focus, exposure mode, image dimensions and acquisition rates fixed
during calibration and operation. Inspect source intervals, not only average rates.

## ROS 2 input buffering and worker lifecycle

The subscriber implementation is in
[ROS2Visualizer.cpp](../ov_msckf/src/ros/ROS2Visualizer.cpp).

| Requirement | Implementation |
| --- | --- |
| IMU buffering | SensorDataQoS subscription depth 1000 at approximately 100 Hz |
| Camera callback access | Remove a ready stereo frame under the queue mutex, then release the mutex before tracking and visualization |
| Asynchronous timestamp lifetime | Capture the measurement timestamp by value |
| Worker ownership | Join the update worker before reuse or destruction |
| Recording continuity | Apply `record_qos.yaml`, including a reliable depth-1000 IMU recording queue |

The IMU queue can hold about ten seconds of samples; this is capacity, not an
acceptable processing delay. Monitor subscriber intervals and end-to-end latency.
A complete sensor bag does not establish that estimator callbacks received every
sample. Compare recorder and estimator delivery separately.

## Commissioning

1. Verify approximately 100 Hz IMU and 30 Hz stereo delivery, valid packets,
   increasing timestamps and no source faults.
2. Measure the calibration target and record raw stereo plus dynamic IMU data.
3. Fit stereo geometry, camera/IMU extrinsics and timing using
   [the calibration tools](CALIBRATION_TOOLS.md).
4. Generate a consistent rectified profile and record its file fingerprints.
5. Complete stationary, measured-translation and continuous-motion checks using
   [the acceptance procedure](OPENVINS_IMPLEMENTATION_GUIDE.md#9-validate-and-activate-the-new-profile).

Validate motion within a measured workspace and inspect the complete trajectory
for nonphysical jumps. Use independent ground truth to qualify trajectory accuracy.

## Operation

Run from the source workspace after validating the active assembly:

```bash
./hw290_stereo/run_hw290_openvins.sh
./hw290_stereo/run_hw290_openvins.sh --no-rviz
./hw290_stereo/run_hw290_openvins.sh --diagnostics
```

The diagnostic mode records sensor, state and resource data. Use matching
recording and visualization settings when comparing computational cost.
