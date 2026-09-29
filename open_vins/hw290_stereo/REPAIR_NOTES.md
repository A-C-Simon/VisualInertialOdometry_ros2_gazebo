# HW290 timing and trajectory repair, 2026-09-25

## Confirmed findings

- Installed usb_cam 0.8.1 converted `tv_usec` to milliseconds while adding it
  to a microsecond epoch. This introduced a startup-dependent timestamp error.
  `patches/usb_cam-0.8.1-timestamps.patch` fixes that unit conversion. The local
  driver is built with `bash hw290_stereo/build_camera_driver.sh`.
- The connected chip reports WHO_AM_I **0x98**, matching the ICM-20689 identity.
  The former firmware treated it as an MPU6050. The new firmware handles both
  identities and configures the separate ICM accelerometer low-pass filter.
  This is an identity-register match, not independent verification of the chip.
- Serial-arrival timestamps are replaced with MCU acquisition timestamps,
  sequence numbers and checksums. The bridge preserves partial serial records,
  handles 32-bit micros wrap, reports losses, and refuses clock resets.
  Clock reconstruction still has USB latency uncertainty; it is not hardware sync.
- During the first stationary check, 2,002 IMU messages had positive intervals
  from 9.808 to 10.176 ms; there were no checksum errors or sequence gaps.
  Acceleration standard deviations were [0.0191, 0.0220, 0.0134] m/s²;
  gyro standard deviations were [0.00321, 0.00232, 0.00343] rad/s.
  These are short-term statistics, not an Allan-variance noise calibration.
- The first motion capture clipped the old ±2 g / ±250°/s range. Firmware IMU2
  uses ±4 g / ±500°/s; the packet version selects the matching SI conversion.
- An OpenVINS background callback captured a dead stack variable by reference.
  It now captures the timestamp by value, owns its threads and joins them before
  estimator destruction. A full EuRoC run subsequently exited normally.
- The calibration helper had applied raw-camera distortion to rectified images.
  It now consumes the actual CameraInfo, subtracts measured stationary gyro bias,
  and reports a held-out rotational residual.

## Current mount calibration

Confirmed that both position and orientation changed. A fresh rigid
assembly rotation capture produced 247 usable pairs, axis spread 0.200,
training median residual 0.399° and held-out median residual 0.386°.
The estimated camera-to-IMU time offset is **+0.020 s** (IMU time = camera time
+ offset). The old provisional value was +0.155 s and is obsolete with the new
camera driver. The rotation/time candidate is applied to the OpenVINS YAML.
The previous YAML is retained in `calibration_archive/`.

IMU measured about 4 cm from each lens and supplied front,
side and rear photos. The current initial lever arm is approximately
`p_IMU_in_cam0 = [0.02995, -0.02456, -0.01000] m` in optical coordinates.
The 1 cm depth component is estimated from the photographs. This replaces the
previous zero translation, but is **not a full translational calibration**.

### Physical displacement check, 2026-09-28

After initialization, movement was done between marks 20 cm apart,
held five seconds, returned and remained still. The 45-second capture contains
1,348 online poses and 4,496 IMU messages:

- Maximum estimated displacement: **0.21118 m** (nominal reference 0.20 m).
- Final position residual: **0.02462 m**.
- Maximum initial stationary displacement about the median: **0.00108 m**.
- Largest output timestamp interval: **0.03615 s**.

This is a manually referenced displacement check, not time-resolved trajectory
ATE or proof of performance on every scene. It demonstrates that this test
no longer turns centimetres of movement into metres or kilometres.
Artifacts: `benchmark/results/hw290_motion_2/`.

The launcher derives its static transforms from the calibration YAML. It uses
only the corrected local camera driver and disables the splitter's speculative
arrival-time correction. A child process exit terminates the rest of the run.
The IMU bridge reports firmware diagnostics and fails after missing startup
output, rather than leaving a silent stream. The restart failure was traced to an unsuccessful I2C identity read.
Firmware now clears a potentially interrupted bus transaction, uses 100 kHz
I2C, retries initialization and verifies range/rate registers. Three successive
serial reconnect tests each produced 235 valid records and no checksum errors.
The subsequent live movement test also started successfully.

### IMU rate collapse, 2026-09-29

Later failing runs reported about 14.3 IMU samples per second. ORB skipped
stereo frames for insufficient IMU coverage and reset its active map; OpenVINS
diverged. One earlier run dropped from about 100 Hz to about 4 Hz after
saturation and sequence gaps. A direct serial check reproduced failed sensor
identity reads. The exact electrical or register-state cause remains unknown.

Firmware now performs a full sensor reset, verifies operating mode, range,
filters and interrupt registers, and reports one-second sample and I2C error
counts. It halts on a bad configuration or an acquisition rate outside
80-120 Hz. The C++ and Python bridges independently check source and delivery
rates and large sample gaps. Both launchers wait for a verified 80-120 Hz stream
before starting VIO. RViz remains available on sensor failure.

After USB reconnection and the firmware update, a full camera/OpenVINS run
reported approximately 100 Hz for over three minutes, with no sequence gaps,
corrupt packets, saturation or I2C read errors. Stationary initialization
completed, but the attempted pose recorder did not receive initialized poses.
The run was stopped before movement validation. This does not establish that
the long-term trajectory drift is fixed; repeat the measured movement test and
check ORB reset recovery before claiming a trajectory improvement.


## Reproduction

```bash
# Compile/upload requires device access; this overwrites the Nano sketch.
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old \
  --build-path /tmp/hw290_timestamped_build hw290_stereo/firmware/hw290_openvins
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328old \
  --port /dev/ttyUSB0 --input-dir /tmp/hw290_timestamped_build
./hw290_stereo/run_hw290_openvins.sh --sensors-only --no-rviz
# In another ROS-sourced terminal, with the rig still:
ROS_DOMAIN_ID=48 python3 hw290_stereo/inspect_sensors.py \
  --seconds 20 --output /tmp/hw290_static.json
```

Register references: [TDK ICM-20689 datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000143-icm-20689-datasheet.pdf),
[TDK MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
