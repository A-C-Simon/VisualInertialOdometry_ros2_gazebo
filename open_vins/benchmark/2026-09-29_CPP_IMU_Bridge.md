# C++ HW290 IMU bridge — 29 September 2026

The live IMU bridge is now a C++ ROS 2 executable in `ov_hw290`.
Both hardware launchers select it by default. Python is retained for comparison
and fallback; launch/calibration/report scripts remain unchanged in language.

## Measured improvement

Sequential tests fed the Python and C++ bridges identical 100 Hz IMU2 packets
through a pseudo-terminal, including a micros-counter wrap, partial serial
records and a corrupt packet. A ROS subscriber verified SI units, timestamp
intervals and receipt of every expected post-warmup message.

| Bridge | CPU seconds | Mean CPU, one core | Peak RSS | Received after warmup |
|---|---:|---:|---:|---:|
| Python | 4.355 | 25.51% | 60.27 MiB | 1,400/1,400 |
| C++ | 0.188 | 1.13% | 21.63 MiB | 1,400/1,400 |

Each test supplied 1,500 valid packets over 15 seconds. Startup, discovery and
shutdown are included in bridge resource measurements. The feeder/subscriber
cost is excluded. This is approximately **96% less bridge CPU time**, not a
96% reduction in total VIO cost. It is one controlled synthetic serial test;
physical USB latency, trajectory accuracy and full-pipeline savings are not
established by this measurement.

Results: `benchmark/results/imu_cpp_pty_comparison/comparison.json`.

## Preserved behaviour

- IMU1 and IMU2 checksums, raw integer ranges and firmware-dependent SI scales.
- One-second acquisition-clock warmup, counter wrap handling, bounded phase
  correction, loss counts, and failure on duplicate/reset timestamps.
- Saturation rejection, unknown orientation/covariances, `/imu0`, `imu` frame.
- Exclusive serial access and clear startup/disconnection/silence failures.
- Existing per-run summary integration and RViz availability on sensor failure.

The C++ bridge sleeps in `poll()` until serial data arrives rather than waking
Python/ROS every 2 ms. A 100 ms maximum wait bounds shutdown/watchdog latency;
ROS parameter services are serviced every 250 ms. Serial receipt still has USB
latency uncertainty: this does not add hardware synchronization.

## Build and use

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ov_hw290 --build-base build_vio \
  --install-base install_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
./hw290_stereo/run_hw290_openvins.sh
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
# Explicit fallback / comparison:
HW290_IMU_BACKEND=python ./hw290_stereo/run_hw290_openvins.sh --no-rviz
```

No firmware flash or calibration change is required.

## Verification

The package builds successfully. A separate C++ protocol probe matched the
existing Python implementation on 653 cases, including counter wraps, jitter,
loss, duplicate packets, corruption, range validation and clock reset.

```bash
c++ -std=c++17 -O2 -Iov_hw290/include \
  ov_hw290/test/protocol_probe.cpp -o /tmp/hw290_protocol_probe
python3 ov_hw290/test/check_protocol_parity.py /tmp/hw290_protocol_probe
ROS_DOMAIN_ID=64 python3 ov_hw290/test/serial_benchmark.py \
  --output benchmark/results/imu_cpp_reproduction
ROS_DOMAIN_ID=64 python3 ov_hw290/test/serial_fault_checks.py
```

The performance-test output directory must be new. Serial exclusivity,
startup timeout, disconnection, saturation rejection, clock reset and stream
silence tests all passed. Test logs are saved with the benchmark artifacts.

A coordinated real-device check received **1,528 IMU messages** during an
18-second run including startup/warmup. It used **0.300 CPU seconds (1.66% of
one core)** and **21.63 MiB** peak RSS, and shut down normally. No corrupt
packets or sequence gaps were reported in its periodic counters. No received
timestamp intervals were nonpositive; observed intervals ranged from
**9.820 to 10.373 ms**. The sensor reported identity 0x98 and IMU2 firmware.
Artifacts: `benchmark/results/imu_cpp_physical/`.

This is a short communication/resource check, not a new calibrated motion
trial. It does not establish that the earlier intermittent electrical/bus
fault is permanently resolved. The recorded acceleration mean had a norm of
10.24 m/s²; no sensor bias/scale calibration was changed to force that value.
No full OpenVINS/ORB trajectory or whole-pipeline cost comparison was rerun.

