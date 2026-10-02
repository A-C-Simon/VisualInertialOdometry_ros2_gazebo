# HW290 timing and trajectory repair, 2026-09-25

## Current mount change, 2026-10-02

The ELP stereo camera, HW290 and Nano now share the rigid printed tower.
[The OpenSCAD geometry and calibration status](calibration/20261002_tower/README.md)
replace the old photo-based mount prior. OpenSCAD reports a nominal 60 mm
baseline and board-center optical lever arms [0.030, 0, -0.0317] m and
[-0.030, 0, -0.0317] m. Rotation is unverified; +20 ms is a timing starting
estimate. Neither is a completed calibration. Both hardware launchers now
require approved fingerprints for this mount before enabling live VIO.

## Tested previous-mount calibration, 2026-10-01

[calibration/20261001/](calibration/20261001/README.md) passed the earlier
OpenVINS desk test. [DRIFT_FIX.md](DRIFT_FIX.md) preserves its Git reference,
results and repeatable calibration method. ORB received that profile on
October 2 before the tower change. Numerical profiles and earlier history
below belong to their original mounts and recordings.

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

## Earlier mount calibration, 2026-09-25

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


### Repeated communication failure, 2026-09-30

A desk motion recording reached 319 m within a permitted 1.72 m displacement
bound. I2C read errors occurred near both excursion onsets despite a 100 Hz
average. A repeat after reseating the connections stopped with a 105.608 ms
MCU acquisition gap. Its maximum displacement before stopping was 0.668 m;
this incomplete repeat does not validate long-motion stability.

The updated firmware records an immediate `I2C_FAULT` and separate status,
sample and timeout counters. An IMU-only check confirmed `fault_trace=1` and
100.1 Hz startup, followed by a sample read returning zero bytes and failed
configuration reads. This is a sensor communication failure, not evidence
that a calibration or estimator setting is its cause. The module is connected
to Nano 5V at VCC_IN. Photos show a separate 3.3V pin and an apparent regulator;
initially the regulator output and idle bus voltage were unverified.
With a temporary firmware build that stops I2C polling, measured VCC_IN was
4.65 V, the 3.3V rail was 3.32 V, and both header SDA/SCL pins were 4.65 V.
A reference GY-87 schematic includes level translation; this is not proof
of the exact HW-290 revision or its sensor-side voltage. The earlier SDA
2.25 V and SCL 2.6 V readings had unspecified bus activity.

The normal diagnostic firmware was restored after measurement. Its short
20-second IMU-only check started at 100 Hz with no reported I2C errors,
sequence gaps or saturation. This confirms restoration and current delivery,
not long-motion stability or a repair of the intermittent communication fault.

Artifacts: `benchmark/results/firmware_diagnostics_20260930/probe/` and
`benchmark/results/hw290_openvins_20260930_122500_npx7go/`.

The rebuilt estimator emits DEBUG `[VIO_UPDATE]` timestamps and candidate and
retained feature counts. A replay verified 141 update lines and count bounds.
These are diagnostic counts, not guaranteed accepted EKF measurement rows.

The next desk test reproduced two partial sample reads: 8 bytes returned from
14 requested, followed by an 84.032 ms acquisition gap. The bridge stopped,
and the launcher terminated the estimator. Its live TF and pose updates then
disappeared. A fixed-frame change cannot restore valid VIO after that failure.
The capture ended at 29.85 seconds, with maximum displacement 0.106 m; the
planned two-minute movement interval was incomplete. The raw stream averaged
100.09 Hz over 553.09 seconds but contained two missing sequence numbers.
Replacing suspect leads or securing reliable contacts and strain relief is
the next physical check; a specific wiring or module cause is still unproven.
Artifacts: `benchmark/results/hw290_openvins_20260930_134122_p84GeF/` and
`benchmark/results/hw290_20260930_openvins_desk_3/`.

### Visual loss and exposure correction, 2026-10-01

The final fixed-exposure run had no I2C errors, sequence gaps or saturated IMU
samples, but reached 39.90 m. Recorded images were nearly black: the interval
around the first departure contained about 98 percent pixels below intensity
10 and as few as 1 and 10 ORB keypoints in the two images. The first 1 m error
appeared at 64.00 seconds and 10 m at 66.83 seconds.

Direct tests found that `autoexposure: true` in usb_cam 0.8.1 did not reliably
control this device. A later clean restart reported UVC mode 3 but stayed at the
old 5 ms value, producing a mean intensity of 0.03/255 and only 10 to 21 ORB
keypoints. Raising gain from 120 to 255 while retaining 5 ms produced about
1,400 keypoints and 81 valid stereo matches in the same view. The launcher now
applies and verifies manual 5 ms exposure, gain 255, neutral brightness and
neutral backlight compensation after camera startup.

Replaying the earlier failure with a 10 ms camera-to-IMU offset worsened the
maximum displacement to 16.32 m. The retained 20 ms offset produced 0.958 m on
the same 5x-gate replay, so the calibrated 20 ms value remains in use. Physical
continuous-motion validation of the exposure correction remains pending.

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

### Estimator IMU delivery failure, 2026-10-02

The rigid tower's first physical calibration trial reached 15.856 m despite
confirmed desk-scale motion. Firmware reported no I2C errors, corrupt samples,
sequence gaps or saturation. The recording contains 6,429 IMU samples at
99.019 Hz with maximum interval 21.523 ms. OpenVINS received gaps up to
680.785 ms and exited after negative covariance entries.

The default ROS 2 SensorDataQoS retains five IMU messages, only about 50 ms
at this rate. Image callbacks share the default callback group and can wait
behind the camera update queue lock. The subscriber now retains 1,000 messages
by default; `imu_queue_depth` can select 5 to 10,000. Best-effort compatibility
is preserved. This absorbs stalls but does not guarantee delivery under
unbounded load or prove that dropped IMU was the sole cause of divergence.

Real-time headless replays of the same failed recording showed:

| Check | Original queue 5 | Queue 1,000 |
|---|---:|---:|
| Received IMU samples | 6,239 | 6,429 |
| Received intervals above 50 ms | 17 | 0 |
| Maximum received interval | 250.033 ms | 21.523 ms |
| Maximum trajectory displacement | 0.501 m | 0.495 m |

Initialization times differed, so these displacement values are not an
accuracy improvement score. Both replays ended near the recording's final
image timestamp. The fixed replay excludes camera drivers, RViz and recording;
a new physical test with those processes still needs to pass.

Evidence: `benchmark/results/hw290_openvins_20261002_144009_LinLH5/`, including
`imu_continuity.json` and `replay_validation/{baseline_queue5,queue1000_installed}`.
The earlier `queue1000` trial loaded the old installed library and was stopped;
it is incomplete and excluded. The valid trial startup explicitly logged
`queue depth 1000`. Build and install the library before validating runtime
changes: `cmake --build build_vio/ov_msckf --target run_subscribe_msckf -j2`,
then `cmake --install build_vio/ov_msckf`.

### Camera callback blocking, 2026-10-02

After the IMU queue change, a two-minute live test received 20,503 IMU samples
with no large gaps or covariance failure. One sharp 1.378 m position step at
74.599 seconds nevertheless reached 2.197 m. The source stereo frames remain
continuous below 36.065 ms near the event, but estimator camera updates skip
two seconds after a 1.429-second processing stall.

The ROS 2 update worker held camera_queue_mtx throughout tracking and
visualization. Image callbacks also need that lock, and share a callback group
with IMU reception. The worker now removes one eligible camera message under
the lock and releases it before processing. Queue ordering and the requirement
for IMU coverage beyond camera time are preserved, with one update worker.

Real-time replay of the full saved movement interval now saves 3,612 poses over
120.299 seconds, maximum displacement 0.86757 m, maximum pose interval 36.150 ms
and maximum position step 0.09030 m. It passes the original jump interval
without another large step. The replay is headless and has no independent
ground truth; fresh physical validation remains necessary.

Evidence: `hw290_openvins_20261002_145045_4UIsx8/` under benchmark/results,
including camera_gap_check.json and replay_validation/camera_lock_release.
