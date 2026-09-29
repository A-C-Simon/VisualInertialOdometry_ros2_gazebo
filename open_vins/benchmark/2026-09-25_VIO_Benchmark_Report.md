# OpenVINS and ORB-SLAM3 VIO benchmark

Date: 25 September 2026  
Host: Intel N100, 4 cores, Ubuntu 22.04, Linux 6.8  
Dataset: EuRoC `V1_01_easy`, 752×480 stereo at 20 Hz and IMU at 200 Hz

## Scope

This report covers four work items:

1. Test OpenVINS with a public camera and IMU dataset.
2. Test OpenVINS with the ELP stereo camera and HW-290 MPU6050.
3. Record accuracy, behavior, and computational cost.
4. Run the equivalent ORB-SLAM3 test and compare the estimators.

## Public dataset method

Both estimators used the same complete EuRoC `V1_01_easy` ASL-format sequence:
2,912 synchronized stereo pairs, 29,120 IMU samples, and 28,712 ground-truth
states over approximately 145 seconds.

OpenVINS received the original sensor timestamps through ROS 2 topics using
`euroc_player.py`. ORB-SLAM3 used its standard `stereo_inertial_euroc`
executable with the supplied EuRoC stereo-inertial configuration. Both ran
without a graphical viewer.

Position accuracy was calculated by nearest timestamp association with a
10 ms maximum difference followed by rigid SE(3) alignment. Scale was not
corrected. The metric is translational absolute trajectory error.

## Public dataset results

| Metric | OpenVINS | ORB-SLAM3 stereo inertial |
|---|---:|---:|
| Matched poses | 2,781 | 2,790 |
| ATE RMSE | **0.0491 m** | **0.0389 m** |
| Median position error | 0.0447 m | 0.0358 m |
| Maximum position error | 0.0934 m | 0.0817 m |
| Estimated path length | 57.565 m | 58.168 m |
| Matched ground-truth path | 58.450 m | 58.543 m |
| Peak estimator RSS | **125.0 MiB** | **813.4–822.9 MiB** |
| Mean process CPU | **Invalid interrupted measurement** | **141–155%** |

CPU percentages are process totals on a four-core machine; 100% is one fully
occupied core. ORB-SLAM3 was measured twice with GNU `time`. Its two peak RSS
results were 822.9 and 813.4 MiB. The older OpenVINS monitor run was interrupted at 126.10 seconds of input.
Its 148.53 CPU seconds cannot establish a complete-run CPU comparison. The
previous explanation that the host suspended was not supported by the logs;
the replay contained a negative-timeout bug that could block indefinitely.
The complete historical OpenVINS accuracy run contained 2,800 poses.

These historical accuracy results also differ in trajectory semantics:
OpenVINS poses were saved online, while ORB-SLAM3's exported trajectory was
recomputed from its optimized map. They do not establish an online VIO ranking.
A new runner now records ORB online IMU poses as well as its retrospective export.

A fresh bounded OpenVINS run after fixing replay and worker lifetime completed
all 2,912 camera pairs and 29,120 IMU samples, saved 2,800 poses, and exited
normally. It used 84.85 CPU seconds over 151.70 wall seconds, with 133.15 MiB
maximum RSS and 0.060439 m position ATE (2,781 ground-truth associations).
This is one run, not a distribution; it supersedes the incomplete CPU estimate.

ORB-SLAM3 created one map with 104–105 keyframes and completed visual-inertial
bundle adjustment. Its two wall times were 168.68 and 171.60 seconds, including
vocabulary loading, initialization, real-time pacing, and shutdown.

## OpenVINS simulator result

The ROS 2 Gazebo rover test uses camera plus IMU input and Gazebo model state as
ground truth. Mono plus IMU was the stable default. A corrected live comparison
contained 356 matched pairs over 25 seconds and produced 0.152 m planar RMS
residual with 0.270 m maximum residual. The estimated path span was about
4.24×3.52 m and ground truth was about 3.87×3.39 m.

The simulator result and EuRoC ATE are different metrics and environments, so
their error values should not be compared directly.

## Physical ELP stereo and HW-290 result

The physical pipeline publishes approximately 25–26 Hz camera data and 100 Hz
IMU data. The compiled splitter rectifies the 1280×480 side-by-side stream into
two 640×480 images. A live rectification check found 247–376 descriptor matches
with approximately 1.0–1.2 pixels median vertical mismatch.

OpenVINS initializes and publishes a trajectory. A stationary fixed-scene test
kept position within about 2 cm of its start. Translation tests remain invalid:
one short desk replay reached 4.13 m and another diverged to 91.80 m for a
manually measured 20–30 cm out-and-back movement. No surveyed ground truth was
available.

The hardware trajectory must not be used as odometry yet. The camera-to-IMU
rotation and 0.155 second offset are provisional, camera-to-IMU translation is
assumed, and the camera driver timestamp error changes between process starts.
A rigid mount, full camera-IMU calibration, stable timestamps, and a controlled
textured scene are required for the next validation.

## Earlier ORB-SLAM3 system benchmarks

The existing 90-second benchmarks include complete process groups, so these
figures include drivers or Gazebo and are not directly comparable with the
estimator-only EuRoC measurements above.

| ORB-SLAM3 configuration | Mean CPU | Peak RSS |
|---|---:|---:|
| Gazebo full GUI | 178.94% | 1760.48 MB |
| Gazebo headless full | 132.39% | 1150.97 MB |
| Gazebo headless localization only | 135.45% | 1132.29 MB |
| Gazebo isolated without loop closure | 133.17% | 1135.32 MB |
| ELP full | 230.54% | 1001.12 MB |
| ELP localization only | 177.38% | 732.08 MB |
| ELP isolated without loop closure | 129.52% | 650.71 MB |

Visualization produced the largest immediate system-level cost. On the ELP
camera, the isolated no-loop configuration reduced mean CPU by 43.8% and peak
RSS by 35.0% relative to full ORB-SLAM3.

## Reproduction tools

- `euroc_player.py`: publishes an ASL-format EuRoC sequence to OpenVINS and
  records camera-update poses.
- `monitor_process.py`: samples CPU time and RSS for the estimator PID.
- `evaluate_ate.py`: associates poses, performs rigid alignment without scale,
  and calculates translational ATE.

Example evaluation:

```bash
python3 benchmark/evaluate_ate.py \
  /tmp/openvins_v101.txt \
  /tmp/euroc/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

## Conclusion

Both estimators completed the public stereo-inertial dataset with centimetre
scale ATE. ORB-SLAM3 was more accurate on this single easy sequence, while
OpenVINS had a much smaller bounded memory footprint. The physical OpenVINS
sensor plumbing works, but calibration and timestamp limitations still prevent
a valid hardware trajectory claim. A repeated multi-sequence benchmark and a
surveyed hardware path are required before selecting an estimator for the
physical rover.
