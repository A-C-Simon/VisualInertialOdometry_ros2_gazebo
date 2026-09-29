# HW290 repair and VIO optimization — 28 September 2026

## Physical result

Completed a manually measured **20 cm out-and-back** test with the
same rigid mount. OpenVINS reported a maximum displacement of **21.12 cm**,
a final position residual of **2.46 cm**, and initial stationary jitter of
**1.08 mm** about the median. There were 1,348 poses and 4,496 IMU samples in
45 seconds; the largest pose interval was 36.15 ms. No corrupt, saturated or
missing-sequence IMU packets were reported during this run.

This check avoided the previous metres/kilometres failure. Its reference was
manually measured distance, not a time-resolved ground-truth trajectory. It
does not establish accuracy on every motion or scene, nor compare hardware
ORB accuracy against hardware OpenVINS. No further movement was requested.

Artifacts: [summary](results/hw290_motion_2/summary.json),
[displacement plot](results/hw290_motion_2/displacement.png),
[repair details](../hw290_stereo/REPAIR_NOTES.md).

## Changes installed

- Corrected the usb_cam microsecond conversion; built a local camera driver.
- Configured the IMU identified by register value 0x98, including its separate
  accelerometer filter, verified range registers, and recoverable I2C startup.
- Added MCU acquisition timestamps, sequence numbers, checksums and range-aware
  SI conversion. Firmware now uses ±4 g and ±500 degrees/s. Three reconnects
  each yielded 235 valid packets without checksum errors.
- Re-estimated the mount rotation and +20 ms camera/IMU offset from the user's
  motion capture. Applied an approximate lever arm based on the measured 4 cm
  lens distances and photos. Translation still needs a full spatial calibration.
- Fixed an OpenVINS callback lifetime error and made its worker shutdown explicit.
- ORB: one OpenCV thread, viewers off by default, image sharing, separate IMU
  callbacks, bounded stereo buffering until IMU coverage exists, subscriber-aware
  visualization, and a full-rate online IMU-frame pose topic for evaluation.
- ORB hardware default: 800 features, loop closure disabled; local mapping and
  inertial optimization retained. `--efficient` selects 600 features and the
  isolated core with a 12-keyframe local optimization cap. Global inertial
  initialization is retained. These settings were evaluated on EuRoC; their
  accuracy on this physical rig has not been measured.

## Comparison method

Dataset: complete EuRoC **V1_01_easy**, 2,912 stereo pairs at 20 Hz, 29,120 IMU
samples at 200 Hz. Host: Intel N100, four cores. Trials run sequentially at 1×
playback with the same ROS player; GUI disabled. CPU is estimator process user
plus system seconds from `/usr/bin/time -v`, including startup/shutdown. RSS is
peak resident memory. Replay and sensor-driver costs are excluded for both.
The host was not isolated from all background applications; these are measured
process costs on this machine, not a controlled multi-host performance study.
The ORB baseline uses 1,200 features, default OpenCV threading and loop closure
enabled, with the same repaired ROS wrapper as the candidate runs. Savings
reported against it do not separately measure the wrapper's contribution.

Accuracy uses **online IMU-frame poses** for both systems, interpolated GT,
rigid SE(3) alignment without scale fitting, and one-second relative-pose
translation error. Post-initialization comparison starts at dataset camera
start +30 s (`1403715303.262143`). All trajectories use a common end time.
Whole-interval results are also saved; startup is not silently discarded.

ORB's original online output had a roughly 24.6 m coordinate jump during
inertial initialization. Retrospective, corrected map poses do not show the
same history. Mixing retrospective ORB with online OpenVINS would be misleading.
The original ORB whole-interval ATE was about 6.72 m; this is distinct from its
post-initialization error. Initialization behaviour varies between trials.

ORB's existing best-effort image subscription delivered fewer poses than
OpenVINS's reliable subscription. Coverage and gaps must accompany CPU numbers;
these trials compare the actual ROS pipelines, not identical processed frames.

## Completed measurements

ATE and RPE below are post-initialization, in centimetres. CPU is seconds for
the complete replay, not milliseconds per frame. Coverage is approximate
20 Hz pose coverage over the common evaluation interval.

| Profile ----------------------------|CPUs----|PeakMiB|ATE cm|1sRPEcm|Coverage |
|-------------------------------------|---:----|---:---|---:--|---:--|---:---|
| OpenVINS baseline    ---------------| 84.85  | 133.2 | 5.92 | 4.42 | 100.0%|
| OpenVINS fixed calib ---------------| 78.70  | 135.9 | 4.09 | 4.53 | 100.0%|
| ORB baseline------------------------| 235.46 | 830.2 | 3.99 | 4.67 | 92.2% |
| ORB 800, 1 CV thread, no loop clsur | 172.82 | 758.8 | 3.89 | 4.65 | 95.0% |
| ORB 600, 1 CV thread, no loop clsur | 154.63 | 717.5 | 3.89 | 4.72 | 92.4% |
| ORB 600, local BA cap 12 -----------| 133.96 | 709.7 | 3.44 | 4.62 | 93.9% |
| ORB cap 12, resized 640×408 --------| 121.91 | 731.8 | 3.11 | 4.55 | 93.7% |
| Same resized profile, repeat -------| 120.49 | 737.0 | 3.18 | 4.60 | 96.4% |

The two 640×408 runs reduced CPU **48.2–48.8%** against the ORB baseline while
recording more poses and lower post-initialization ATE (**3.11–3.18 cm**). The
hardware camera profile retains its native 640×480 resolution. Resource savings
on EuRoC are not measurements of the hardware pipeline's resource consumption.

The reversed rankings optimization are **not established**: even this ORB run
used more CPU and memory than OpenVINS, and its measured ATE was lower than
OpenVINS's after initialization.

Including initialization in the common available interval changes the ATE:
OpenVINS fixed calibration **5.44 cm**, ORB baseline **671.84 cm**, ORB 600/cap 12
at full resolution **4.38 cm**, and its 640×408 trials **35.59 cm** and
**17.71 cm**. Thus the
resized trial's 3.11 cm figure must not be presented as whole-run accuracy.
Startup map-coordinate changes remain a limitation of the current ORB online
output and its accumulated RViz path.

The full metrics are in [final_comparison.json](results/final_comparison.json).
The plain-text work report, including a four-line supervisor summary, is
[Sept28cdx.txt](Sept28cdx.txt), also saved in `/home/ac/Work_Reports/Sept28cdx.txt`.

## Rejected or limited candidates

- 400 ORB features never initialized: this upstream stereo initializer requires
  more than 500 detected features. Its incomplete run is not a performance result.
- Enlarging OpenVINS's window/features while still estimating calibration cost
  134.35 CPU seconds with negligible ATE improvement and worse relative error.
- Holding the supplied EuRoC calibration fixed improved OpenVINS at lower cost.
  This does not replace the separate HW290 calibration with EuRoC calibration.
- Enlarging the fixed-calibration OpenVINS window also lost: 119.43 CPU seconds,
  4.48 cm post-initialization ATE, versus 78.70 seconds and 4.09 cm for the
  smaller window. The larger profile is retained only as a rejected experiment.
- Local BA is bounded, not disabled. Turning mapping off during initial VIO
  operation would remove functionality needed to establish and refine the map.

## Run the installed hardware pipelines

From `open_vins/`:

```bash
./hw290_stereo/run_hw290_openvins.sh
../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
```

Run one hardware pipeline at a time. Omitting `--efficient` uses the 800-feature
profile and original ORB core. Omitting `--rviz` avoids display cost. The original
`/home/ac/ORB_SLAM3/lib/libORB_SLAM3.so` is unchanged. Rebuild the isolated variant
with `python3 benchmark/build_orb_core.py`; its build manifest records original
source/library hashes and compiler/linker arguments.

## Reproduce the dataset trials

```bash
source /opt/ros/humble/setup.bash
source install_vio/setup.bash
python3 benchmark/run_public_benchmark.py openvins \
  --config benchmark/configs/openvins_fixed_calibration.yaml \
  --output benchmark/results/ov_reproduction
python3 benchmark/run_public_benchmark.py orb_ros \
  --config benchmark/configs/orb_euroc_600_no_loop.yaml --threads 1 \
  --core-library-dir benchmark/build_orb_core --local-ba-window 12 \
  --output benchmark/results/orb_reproduction
python3 benchmark/summarize_trials.py ov_reproduction orb_reproduction \
  --output benchmark/results/reproduction_comparison.json
```

Output directories must be new. Configuration snapshots, invocation, logs,
online trajectories, completion status and process resource measurements are
retained in each new trial directory. Large result/build artifacts are ignored
by Git and remain on this machine.
