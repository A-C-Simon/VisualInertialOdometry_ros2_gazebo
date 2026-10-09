# HW290 and ELP stereo VIO

The hardware pipeline uses a side by side ELP stereo camera, an HW290 IMU
connected through an Arduino Nano, the OpenVINS estimator and an optional
ORB-SLAM3 estimator. The IMU reports identity 0x98, consistent with an
ICM-20689. Both estimators use the measured October 2 rigid-tower camera/IMU
profile. IMU noise remains provisional. The [selected October 5 ORB profile](ORB_SELECTED_PROFILE.md)
passed the fresh tower movement check.

## Implement OpenVINS on another device

Start with [OPENVINS_IMPLEMENTATION_GUIDE.md](OPENVINS_IMPLEMENTATION_GUIDE.md).
It provides the complete installation and commissioning sequence for the
working OpenVINS pipeline, including its calibration and delivery fixes.
[CALIBRATION_TOOLS.md](CALIBRATION_TOOLS.md) provides the offline tool build
and fitting commands. Another physical assembly needs a new calibration;
the historical troubleshooting sections below are not an acceptance record
for a new device.

## Start here: calibration that stopped the large drift

**Read [DRIFT_FIX.md](DRIFT_FIX.md) before changing this calibration.**
It identifies the current tower profile, the calibration and queue fixes that
removed the large flights, and the method to repeat for a changed mount. The
October 1 Git reference `hw290-openvins-drift-fix-20261001` (`0c9cf22`) records
the previous mount and is retained for historical replay.

## Run

From `open_vins`:

```bash
./hw290_stereo/run_hw290_openvins.sh
./hw290_stereo/run_selected_orb.sh --rviz
```

RViz opens by default for OpenVINS and when requested with `--rviz` for ORB.
Both launchers keep a per-run CPU, memory and estimator summary under
`benchmark/results/`. OpenVINS selects the C++ IMU reader by default;
`HW290_IMU_BACKEND=python` selects the fallback. Build the ROS package with
the following command if the executable is missing:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ov_hw290 --build-base build_vio \
  --install-base install_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Keep the camera and IMU rigidly attached. Hold the rig still for initialization,
then move slowly through a static, textured scene with objects about 0.5 to 2 m
away. Do not assess accuracy from a stationary cumulative path length. Use a
measured motion and return test, and compare against ground truth when available.

## Earlier hardware status, 2026-09-30

Earlier, a 20 cm out and back test reached 21.1 cm estimated displacement and
returned within 2.46 cm. Later failures showed the IMU rate dropping to about
14 Hz instead of 100 Hz. OpenVINS then diverged and ORB-SLAM3 lost tracking and
reset. The firmware and both IMU readers now verify the acquisition and
delivery rates and stop VIO on a rate fault. After reconnecting USB, the source
reported about 100 Hz for more than three minutes with no packet gaps, checksum
errors, saturation or I2C read errors. A movement test after this change
was performed on 30 September. Small free movement stayed bounded, but a desk
test reproduced two excursions reaching 319 m within a permitted 1.72 m
displacement bound. Firmware I2C read errors were reported near both excursion
onsets despite about 100 Hz average acquisition. This correlation is being
investigated; trajectory recovery has not been established.

A fresh stationary reset recovered the earlier gyro baseline without packet
gaps or I2C errors. Diagnostic RGB recording also exposed multi-second host
delivery stalls and dropped bag messages. Source acquisition and subscriber
delivery are measured separately.

The C++ splitter now publishes `mono8`, matching the grayscale conversion
already performed by both estimators. Rectification runs before conversion.
An integration check found identical pixels and headers for both cameras,
with image payloads reduced by two-thirds. Set the splitter parameter
`monochrome:=false` to obtain RGB output. This is a payload reduction, not a
measured reduction in total pipeline CPU or an accuracy improvement.

## Diagnostics and calibration

See [repair notes](REPAIR_NOTES.md) for the single source of calibration,
firmware and camera timestamp details, known limitations and reproduction
commands. The C++ bridge and optimization reports are stored locally in
`/home/ac/Work_Reports/` as `2026-09-29_CPP_IMU_Bridge.md` and
`2026-09-28_HW290_ORB_Optimization.md`. Work reports are kept outside the
repository; the calibration profile and operating guides remain here.

When the IMU fails startup, the launchers keep RViz available in sensor view
and skip VIO until the sensor is restored. During a run, a detected IMU rate or
connection failure stops the estimator and saves the summary.

For failure diagnosis:

```bash
./hw290_stereo/run_hw290_openvins.sh --diagnostics
```

This starts a sensor bag before the IMU and estimator, saves raw serial records
including sequence numbers and raw temperature, and enables estimator debug
logs and complete state/covariance output. `--sensors-only --diagnostics` skips
the estimator for sensor checks. All files are stored in the saved run directory.
Diagnostic recording and debug costs are included in its summary, so use
ordinary runs for compute comparisons. Close the launcher and bag writer before
inspecting the SQLite database. Check the recorder log for message losses.

The IMU bridge permits brief host catch-up bursts when MCU timestamps verify
the source rate. It retains acquisition, disconnect and sustained slow-delivery
guards. Run summaries also report firmware I2C errors; a 100 Hz average alone
does not verify measurement integrity.

Diagnostic firmware reports `fault_trace=1` at startup and an immediate
`I2C_FAULT` for the first failed read in each health window. `HEALTH` also
separates status reads, sample reads and Wire timeouts. After another connection
check, a short IMU-only run started at 100 Hz, then a sample read returned zero
bytes and configuration reads failed. Power, contact and bus voltage checks
remain necessary; changing VIO settings has not resolved this communication
failure. Board VCC input and I2C signal voltage must be checked separately.

The rebuilt estimator's DEBUG output includes `[VIO_UPDATE]` timestamps and
candidate/retained feature counts. A saved-recording replay verified 141 lines
with increasing timestamps and valid count bounds. Retained counts describe
updater filtering, rather than guaranteed EKF measurement rows.

### Powered idle voltage measurement

Keep VCC, GND, SDA and SCL connected when measuring the assembled circuit.
Stopping ROS alone does not stop the Nano's I2C polling. Compile the same
firmware with `--build-property compiler.cpp.extra_flags=-DHW290_IDLE_VOLTAGE`
only for a temporary powered idle check. This mode clears the bus once at
startup, retains the normal Nano pull-ups, then emits serial heartbeats without
sensor reads or IMU packets. Digital high indications are not voltage readings.
Measure VCC_IN, the module's 3.3V rail, SDA and SCL against module GND.

Upload a normal build without that flag after the measurements, before running
VIO. The saved normal firmware for this diagnostic session is in
`benchmark/results/firmware_diagnostics_20260930/`; restore it with:

```bash
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328old \
  --port /dev/ttyUSB0 --input-dir benchmark/results/firmware_diagnostics_20260930
```

For a bus timing experiment, compile with
`--build-property compiler.cpp.extra_flags=-DHW290_I2C_CLOCK_HZ=50000UL` and
export to a separate output directory. The default remains 100 kHz. This
changes I2C clock speed, rather than the configured 100 Hz IMU sampling rate.
The per-transaction Wire timeout scales from 3 ms at 100 kHz to 6 ms at 50 kHz;
the bridge's rate and acquisition-gap guards remain unchanged. The 50 kHz
variant was uploaded on 30 September and passed a 25-second IMU-only check
at 99.15 Hz, without reported I2C errors or missing sequence numbers. Its
maximum source interval was 20.884 ms. Long-motion validation is still pending.
The normal startup reports `I2C_CLOCK_HZ` in builds containing this option.
Verify actual source intervals, health counters and trajectory during motion
before accepting the alternative clock as a useful correction.

### Exposure correction, 2026-10-01

The fixed 5 ms exposure used in the final 30 September run made much of the
recording nearly black. At the first trajectory departure, one lens had only
one ORB keypoint and the other had ten. The path reached 39.90 m despite clean
IMU delivery, so that run demonstrates visual-observation loss rather than an
IMU communication failure.

On this camera, the advertised UVC automatic mode can remain at the old 5 ms
manual value without adapting. In the current desk view that produced a mean
intensity of 0.03/255 and only 10 to 21 ORB keypoints. A live exposure sweep
showed that 5 ms with gain 255 retained about 1,400 ORB keypoints and 81 valid
stereo matches without increasing exposure time. The launcher now applies and
reads back that manual setting after camera startup. A continuous motion desk
test with this setting still escaped the 1.72 m desk bound after about 64
seconds and reached 7.46 m. The IMU remained near 99 Hz without packet, I2C or
saturation faults, and the images still held about 1,480 features after the
failure. Exposure loss is therefore not the complete cause.

### Calibration correction, 2026-10-01

The latest failure logs match the OpenVINS failure mode in which tracked
features are rejected by the MSCKF innovation test, visual updates collapse,
and unconstrained IMU propagation drives the position away. A replay using
online camera extrinsic and time-offset calibration moved the offset estimate
from 20 ms to about 8.4 to 10.5 ms, but still diverged. Online calibration did
not repair the provisional starting calibration.

I completed the raw stereo and camera/IMU fits using the measured 40 mm screen
target. The fitted rotation differs from the previous transform by about
26.5 degrees, and the cam0 time offset is 13.04 ms. Stereo baseline is 5.945 cm.
The new profile applies the rectification rotations to both camera/IMU transforms
and uses matching rectified intrinsics. It is now the normal OpenVINS default.

On the same recorded movement, the old model reached 18.92 km displacement;
the new model stayed below 0.47 m. Both used dynamic initialization for that
offline comparison because the recording starts in motion. The fresh physical
test used normal stationary initialization and continuous desk motion for
about two minutes. I stayed inside the stated workspace and saw no flights.
Across 868 seconds of saved poses, maximum displacement was 0.852 m, below
the 1.72 m desk diagonal. No trajectory clipping or origin resets were added.

The diagnostic run averaged 14.82 ms per logged tracking/update cycle. Estimator
CPU averaged 56.5% of one core, with 185.1 MiB peak RSS; recording and RViz add
their own costs. This is not a matched ORB compute comparison. The recording
has 92,748 IMU samples at 99.02 Hz, with no interval above 25 ms. The estimator
logged 46 input gaps above 50 ms, so subscriber delivery still needs work.

See the [October 1 profile and evidence](calibration/20261001/README.md).
Long stationary Allan calibration and independent trajectory ground truth are
still pending. The estimated 6.7 cm lever arm also needs a better physical
measurement than the approximate 4 cm camera/IMU separation.

Follow the [HW290 offline calibration procedure](calibration/README.md) for the
printable target, raw recordings, Allan dataset and Kalibr commands.

Record a Kalibr input bag with the raw lens images and standard ROS IMU message:

```bash
./hw290_stereo/run_hw290_openvins.sh \
  --sensors-only --no-rviz --diagnostics --raw-stereo
```

The `--raw-stereo` option is restricted to sensor-only runs because the active
OpenVINS camera model expects rectified images. The resulting `sensors_bag`
inside the saved run directory can be converted from ROS 2 to a ROS 1 bag with
`rosbags-convert`, as described by the official Kalibr ROS 2 guide. Record the
static stereo target sequence and dynamic camera-IMU sequence as separate runs.

## Selected ORB pyramid settings

Use `run_selected_orb.sh` for the chosen 600-feature, scale-1.6/five-level profile.
[Selection and evidence](ORB_SELECTED_PROFILE.md) explain why it was retained.
The general launcher can still pass `ORB_FEATURES`, `ORB_PYRAMID_SCALE` and
`ORB_PYRAMID_LEVELS` to the native calibration exporter. Its defaults remain:
600 features with `--efficient`, scale 1.2 and eight levels. The exporter
retains the measured camera/IMU transform, baseline, noise and time offset;
it validates the requested operating point before starting sensors.

The equivalent explicit command uses the separately built core and 600 features:

```bash
ORB_CORE_DIR="$PWD/benchmark/build_orb_fixed_gaussian_packed" \
ORB_PYRAMID_SCALE=1.6 ORB_PYRAMID_LEVELS=5 \
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz --diagnostics
```

Run this command from `open_vins`. A fresh two-minute tower check passed with
this 600-feature profile: maximum displacement 72.4 cm, no active resets or
LOST transitions, and three recovered RECENTLY_LOST events. Continuous motion
within the desk and height limits was confirmed with no observed flights,
jumps or resets. [Recorded checks](../benchmark/tower_pyramid_validation_20261005.json)
retain resources and delivery diagnostics. Desk bounds do not establish
independent ground-truth accuracy or matched CPU savings.
Public 500-feature trials are not the hardware default:
ORB stereo startup needs more than 500 detected features, so the exporter
continues to require a request of at least 501. The smallest pyramid image
must retain at least 64 pixels per side. Do not change mount calibration to
change feature extraction settings.
