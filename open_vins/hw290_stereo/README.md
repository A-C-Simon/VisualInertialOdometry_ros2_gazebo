# HW290 and ELP stereo VIO

The hardware pipeline uses a side by side ELP stereo camera, an HW290 IMU
connected through an Arduino Nano, the OpenVINS estimator and an optional
ORB-SLAM3 estimator. The IMU reports identity 0x98, consistent with an
ICM-20689. The current camera and IMU mount calibration is provisional.

## Run

From `open_vins`:

```bash
./hw290_stereo/run_hw290_openvins.sh
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
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

## Current hardware status, 2026-09-30

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
commands. See [the C++ bridge report](../benchmark/2026-09-29_CPP_IMU_Bridge.md)
for protocol and CPU measurements, and [the optimization report](../benchmark/2026-09-28_HW290_ORB_Optimization.md)
for the EuRoC comparison and ORB settings.

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
reads back that manual setting after camera startup. A full continuous motion
desk test is still required before accepting the repair.
