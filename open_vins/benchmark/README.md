# VIO benchmark

The EuRoC runner replays one ASL format sequence to OpenVINS or the ORB ROS 2
wrapper, records online IMU-frame poses, and measures estimator CPU time and
peak resident memory. It saves the command, configuration snapshot, logs and
resource data in a new results directory. The dataset player is measured
separately from the estimator.

## Run a trial

Download or unpack EuRoC `V1_01_easy` at `/tmp/euroc/V1_01_easy`, then source
ROS Humble and the OpenVINS install. Example:

```bash
source /opt/ros/humble/setup.bash
source install_vio/setup.bash
python3 benchmark/run_public_benchmark.py openvins \
  --config benchmark/configs/openvins_fixed_accuracy.yaml \
  --output benchmark/results/openvins_fixed
```

Use `orb_ros` as the method to run the ROS wrapper. Provide the matching
ORB settings file with `--config`. The `orb` method runs the native replay
executable and expects it to be built at `/tmp/orb_online_build/euroc_online`.
For the isolated ORB local-BA experiment, first run
`python3 benchmark/build_orb_core.py` and pass its library directory and the
requested local-BA window to the runner. See `--help` for the complete options.

## Compare trials

`compare_runs.py` compares one or more online trajectories with EuRoC ground
truth over their shared time interval. It reports rigid SE(3) ATE, one second
relative translation error, pose coverage and the largest pose gap. Alignment
does not fit scale. `summarize_trials.py` collects these metrics with CPU and
memory measurements from completed runs and reports both the full interval and
the post-initialization interval. Reports in `/home/ac/Work_Reports/` state dataset, host,
configuration and limits for each recorded comparison.

## Hardware runs

The [October 2 tower replay comparison](tower_comparison_20261002.md) records
sequential real-time runs with the selected measured calibration. OpenVINS
remained bounded; ORB still reset and exceeded the physical workspace.

The HW290 launchers save individual run summaries under `results/`. These
include sensor faults, logged estimator timings, process CPU and peak memory.
Manual out and back distances are diagnostic checks, not time resolved ATE.
See [the HW290 repair notes](../hw290_stereo/REPAIR_NOTES.md) for operating
details. Dated work reports are kept separately in `/home/ac/Work_Reports/`.

## Rejected HW290 initialization experiments, October 2

A longer low-motion grace period reduced resets but produced a 5.34 m flight
on the recorded desk test. A temporal motion-window prototype then completed
both inertial refinements in three replays, with maximum displacements of
0.62 m, 0.69 m and 0.84 m. Its clean repeated replay used 107.7% of one core
and 946.9 MiB, exceeding OpenVINS cost.

The fresh live test invalidated that prototype: maximum displacement reached
10.31 m, including a sudden 9.93 m position jump as VIBA 2 finished, about
60 seconds after the movement cue. Actual movement stayed within the desk
limits. Completing refinement and staying bounded on an earlier recording
were insufficient validation. The underlying optimizer/calibration cause is
still unresolved.

The experimental builder option and patch were reverted. Its generated
libraries were renamed to prevent launch selection. The default core retains
the upstream reset protection. Sensor recordings, online poses and logs remain
in ignored benchmark results for comparison with the unchanged core. Dated
reports remain in `/home/ac/Work_Reports`.

## Experimental inertial map origin preservation

Full inertial BA can move the optimizer's unconstrained global translation.
The origin experiment translates the optimized cameras and included map points
back to the earliest retained camera's pre-BA position. It leaves optimized
rotations, velocities, biases and relative geometry intact. It applies the same
correction to immediate and staged BA outputs; locally anchored BA is unchanged.

Build it explicitly in a separate directory:

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_origin --preserve-inertial-origin \
  --check-translation-invariance ../ORB_SLAM/orb_slam_ros2/config/ELP_640x480_inertial.yaml
```

The builder rejects selecting this option for the default core directory. Its
manifest records the patch and source hashes. `INERTIAL_BA_ORIGIN` reports the
removed translation and largest remaining relative keyframe correction.

The selected tower recording removed an 11.07 m translation during VIBA 2,
while the largest relative correction was 4.4 mm. The separate origin-only
trial remained below 73.3 cm but still reset 51 times, so origin preservation
does not resolve initialization by itself. Those are diagnostic results,
without independent ground truth.

`orb_translation_invariance.cc` checks the change using ORB's actual stereo
and inertial graph edges, with the exported tower extrinsics. Across 180
checks with translations up to 1,000 m, maximum visual residual change was
5.84e-11 pixels and inertial residual change was 1.67e-12. This verifies the
coordinate transformation; it does not validate optimizer convergence or
live accuracy. The default core is unchanged while live validation continues.

The fresh two-minute tower test stayed below 84.8 cm and completed VIBA 2,
but still reset during the first minute, including a 66.5 cm discontinuity.
Actual movement was confirmed within the desk limits. Origin preservation is
retained for investigation; the keyframe-spacing trial has not established
stable initialization. Evidence is in
`results/hw290_orb_20261002_160638_e9Uy92/`.

To reproduce that keyframe-spacing trial, add `--keyframe-interval-s 0.25`
to the experimental build command. This keeps camera tracking at its input
rate, but limits redundant keyframe insertion while tracking has at least 50
inliers. Weak tracking and loss retain urgent insertion. The initial ten-keyframe
bootstrap, motion-reset thresholds and full inertial refinement are unchanged.
The interval defaults to zero and cannot be enabled in the default core directory.

## Experimental motion-gated initialization

The current tower's continuous-movement test still reset despite keyframe
spacing. Its adjacent-keyframe translation fell below the upstream 2 cm reset
threshold. A one-second translation window reduced the resets to three, but
still discarded a map during a quiet interval after the first refinement.

The motion-gated experiment waits through quiet intervals. It advances the
motion clock only when the two translation segments over roughly one second
sum to more than 5 cm. First inertial fitting requires two accumulated motion
seconds; the existing five- and fifteen-second refinement gates remain. Map
age no longer supplies the initial motion clock. Timestamp and tracking-loss
checks remain active. This policy has only been tested with stereo-inertial
input; low translation can leave the system waiting in stereo tracking.

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_motion_gate \
  --preserve-inertial-origin --keyframe-interval-s 0.25 \
  --motion-gated-initialization
```

This option requires a separate output directory and defaults off. The generated
LocalMapping, Tracking and Optimizer sources were byte-identical to the compiled
prototype used for the following sequential, real-time, headless replays:

| Recording | Active resets | Refinements | Max displacement | CPU seconds | Peak MiB |
| --- | ---: | --- | ---: | ---: | ---: |
| Failed live ORB tower test | 0 | Both completed | 0.879 m | 157.01 | 692.13 |
| Final OpenVINS tower test | 0 | Both completed | 0.811 m | 168.77 | 755.50 |

These runs retain 600 features, the 12-keyframe local BA cap and queue depth 64.
Online poses include preliminary stereo tracking. They have initialization
steps of 25.9 and 34.1 cm, respectively. In the first recording, the 25.9 cm
step coincides with a 90.03 degree orientation change; compensating that common
rotation leaves 4.7 mm of position change. This is consistent with world-frame
gravity alignment. It does not establish independent trajectory accuracy.
A fresh physical test and ground-truth verification remain required. ORB is
still more expensive than OpenVINS on the matched recording.

Evidence: `results/tower_live_replay_20261002/orb_motion_gate/` and
`results/tower_matched_20261002/orb_motion_gate/`.


### Initialization-aware online output

The native wrapper now waits for inertial initialization before publishing pose,
body pose, path, TF and the online trajectory. It skips the transition frame and
clears the displayed path when the map changes. Map resets remain diagnostic
failures, even when separate map paths are no longer joined in RViz. Enable
`publish_bootstrap_poses` for raw preliminary stereo poses.

On the failed live recording, the initialized-output replay completed both
refinements with zero resets and zero tracking losses. It produced 3,241 poses
over 107.94 seconds, with maximum step 5.25 cm and maximum pose interval 36.1 ms.
The full input still covered 190.2 seconds and 5,567 tracked stereo pairs. The
shorter pose coverage reflects waiting for initialization; it is not frame
filtering or position smoothing. This replay used 155.18 CPU seconds and
693.48 MiB peak memory. Evidence: `results/tower_live_replay_20261002/orb_motion_gate_display/`.
