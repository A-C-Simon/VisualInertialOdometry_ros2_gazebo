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


### Fresh live check and sensor interruption

The motion-gated core and initialized-output wrapper completed both refinements
with zero active resets during the fresh tower run. The recorded initialized
path lasted 70.13 seconds, stayed below 79.6 cm, and had a maximum position step
of 5.7 cm. Two brief RECENTLY_LOST episodes recovered, with maximum pose gap
168 ms. Actual movement was confirmed within the desk and height limits, with
no observed flights or resets.

This is an incomplete test: IMU measurements stopped approximately 87 seconds
after the movement cue, and the sensor watchdog shut down the pipeline. The
last firmware health record reported zero I2C errors; both USB device nodes
remained present. No kernel USB event identified the cause. This does not
establish a wiring or I2C fault, and does not qualify as a two-minute pass.
The external movement timer initially missed the pipeline shutdown; saved
metadata has been corrected. Resource totals from this shortened live run
are not a valid matched efficiency comparison.

Evidence: `results/hw290_orb_20261002_165009_S4g8Gr/analysis.json`.
Compact replay and live measurements: [tower_initialization_20261002.json](tower_initialization_20261002.json).


The repeat also completed both refinements without resets. It saved 69.70
seconds of initialized poses, maximum displacement 86.6 cm, maximum step
14.0 cm, and maximum pose interval 64 ms. It is also incomplete: the kernel
reported the Nano CH340 USB disconnect at 17:00:26 and re-enumeration at
17:00:28. The serial driver stopped on a disconnect/device error. This confirms
a USB transport interruption for the repeat; its precise physical or electrical
cause remains unverified. The movement monitor detected the failure immediately.
Evidence: `results/hw290_orb_20261002_165802_x5vMbS/`.

The tested experimental library is saved persistently in
`benchmark/build_orb_motion_gate/`. Its SHA and all five prepared source files
were checked against the compiled prototype. It remains separate from the
default core; repeat physical validation after securing the USB connection.
To select it from the OpenVINS directory:

```bash
ORB_CORE_DIR="$PWD/benchmark/build_orb_motion_gate" \
  ../ORB_SLAM/orbslam3_hw290_vio.sh --efficient --rviz
```


## Local BA cap cost check after initialization fixes

Sequential real-time headless replays of the final OpenVINS tower recording
held the motion-gated core, 600 features, queue 64 and initialized-only output
constant. Only the local inertial BA cap changed.

| Local BA cap | CPU seconds | Peak MiB | TrackStereo mean ms | Max position step |
| --- | ---: | ---: | ---: | ---: |
| 12 | 166.13 | 755.74 | 15.66 | 2.78 cm |
| 6 | 165.58 | 770.58 | 16.88 | 3.00 cm |

Both tracked 4,869 pairs, completed both refinements, and had zero resets or
tracking losses. The CPU reduction was only 0.33%, with higher peak memory in
the smaller-window run. One trial each cannot resolve such a small difference,
so the cap remains 12. Neither recording has independent ground truth; this
is a cost and continuity check. See [the measurements](tower_ba_cost_20261002.json).


## Verified local EuRoC cache

`V1_01_easy` is saved under `benchmark/datasets/euroc/V1_01_easy`, with its ZIP
beside it. The dataset directory is ignored by Git. Pass that directory with
`--dataset` to the public benchmark runner; it is independent of `/tmp` cleanup.

The official ETH archive still returned HTTP 429 on October 2. The public
[Hugging Face mirror](https://huggingface.co/datasets/pepijn223/euroc-mirror/tree/main)
provided the sequence. Its 1,149,702,102-byte size and CRC32 `1abe6a0d` matched
the entry in the previously cached official ZIP directory. All inner ZIP
member CRC checks passed; every camera CSV filename exists. The cache has
2,912 frames per camera, 29,120 IMU rows and 28,712 ground-truth rows. SHA256 and retrieval
provenance are recorded in [euroc_dataset_provenance_20261002.json](euroc_dataset_provenance_20261002.json).
CRC matching checks integrity; the recorded SHA256 is a local fingerprint,
not a separately published official SHA256.


## Fresh public-sequence checks, October 2

Fresh runs use the [upstream corrected V1_01 ground truth](https://github.com/rpng/open_vins/blob/master/ov_data/euroc_mav/V1_01_easy.csv).
[OpenVINS documents the original file's orientation issue](https://docs.openvins.com/gs-datasets.html).
`ov_data/euroc_mav/V1_01_easy.csv` is already present in this checkout. Both full
common-interval and camera-start-plus-30-second metrics are retained, with no
scale fitting. These intervals must be labeled separately; later-refinement
accuracy cannot stand in for startup behavior.

Preliminary best-effort ORB runs processed only 2,786 of 2,912 input pairs at
native resolution and 2,867 at 480 x 306. The 480-pixel run reduced estimator
CPU from 129.63 to 84.81 seconds, but post-initialization ATE increased from
2.01 to 5.26 cm and full common-interval error also increased. This is a rejected
quality tradeoff for now, with incomplete image coverage. OpenVINS used
69.89 CPU seconds and had 100% post-initialization pose coverage.
[Preliminary measurements](euroc_preliminary_20261002.json) retain the coverage
and timing limits; they are not the final performance comparison.

The public ROS-wrapper runner now explicitly requests reliable images, queue
64 and initialized-only poses. `--best-effort-images` remains an explicit
diagnostic option. The publisher is reliable in either case. The change aims
to retain input pairs before interpreting cost reductions; completed trials
must still verify actual tracked-pair counts and acquisition continuity.


### Reliable delivery comparison

Sequential 1x headless V1_01_easy trials used corrected ground truth, one CV
thread, fixed calibration and the motion-gated experimental ORB core. Both
ORB profiles tracked all 2,911 eligible stereo pairs, received all 29,120 IMU
samples, completed both inertial refinements and had zero map resets.

| Profile | CPU seconds | Peak MiB | ATE cm after 30 s | 1 s translation RPE cm | Pose coverage |
| --- | ---: | ---: | ---: | ---: | ---: |
| OpenVINS fixed | 69.89 | 121.88 | 3.340 | 0.989 | 100% |
| ORB 752 x 480, 600 features | 146.26 | 702.81 | 1.894 | 1.201 | 100% |
| ORB 480 x 306, 600 features | 87.34 | 689.49 | 5.610 | 2.564 | 100% |

The 480-pixel trial saved 40.3% ORB CPU time but worsened both trajectory
metrics and still used 25.0% more CPU time than OpenVINS. Native resolution
remains selected. The table uses a fixed camera-start-plus-30-second interval;
full common-interval metrics and initialization coverage are retained in
[euroc_reliable_20261002.json](euroc_reliable_20261002.json). CPU totals include
estimator startup and shutdown, excluding the player and viewers. One trial
per profile on one sequence does not establish repeatability or a general
accuracy ranking.


## USB-secured tower movement check

After securing the Nano USB connection, the first recording was mostly
stationary and initialized late. It is retained as a delivery check, not a
continuous-motion pass. A restarted recording completed 121.1 seconds after
the movement cue, with physical motion confirmed inside the 140 x 80 cm desk
area and 60 cm height limit, without observed flights or resets.

The experimental motion-gated core at native resolution completed both
inertial refinements, with zero map resets and no sensor failure. It published
3,251 initialized poses over 109.10 seconds after a 10.22-second startup.
Maximum displacement was 84.19 cm. One RECENTLY_LOST episode recovered in
0.832 seconds; the largest 9.58 cm step spans that gap. This is a successful
desk-scale stability check, with a brief output interruption and no independent
ground truth. It does not establish continuous pose coverage or an accuracy
ranking. The default core remains unchanged.

All 14,639 recorded IMU samples had source gaps below 22 ms, and all 4,573
images per camera had source gaps below 37 ms. IMU receipt still stalled up to
1.03 seconds, so the enlarged bounded queue remains necessary. Firmware
reported no I2C errors and the kernel had no Nano USB disconnect events in the
recording interval. The live estimator including ROS launch used 153.16 CPU
seconds, 710.07 MiB peak RSS, and TrackStereo averaged 19.72 ms; this diagnostic
run also included sensor drivers, recording and RViz, so these totals cannot
replace matched headless cost measurements.

Evidence: `results/hw290_orb_20261002_190440_OTDslo/analysis.json`,
`sensor_delivery.json`, and [tower_initialization_20261002.json](tower_initialization_20261002.json).


### Wider healthy keyframe spacing trial

A native-resolution experiment increased healthy keyframe spacing from
0.25 to 0.5 seconds after the first inertial fit. Both profiles tracked all
2,911 eligible public-sequence stereo pairs and completed both refinements
without resets. Estimator CPU fell from 146.26 to 127.60 seconds (12.8%),
and peak RSS from 702.81 to 664.00 MiB.

Post-30-second ATE changed from 1.894 to 1.864 cm; 1-second translation RPE
changed from 1.201 to 1.281 cm. Full common-interval ATE increased from 3.458
to 6.204 cm, with RPE rising from 1.965 to 3.127 cm. Therefore this candidate
is not selected on late accuracy alone. A follow-up will retain 0.25 seconds
until the second inertial refinement completes, then try 0.5 seconds.
[Measurements and limits](euroc_spacing_20261002.json) retain both intervals;
one trial cannot separate scheduling variation from the policy change.


The follow-up builder option `--refined-keyframe-interval-s 0.5`, with initial
`--keyframe-interval-s 0.25`, changes the interval only after `GetIniertialBA2()`.
It requires an isolated output directory and preserves urgent insertion when
tracking weakens. The first trial used 127.04 CPU seconds and 664.84 MiB, but
post-30-second ATE increased to 2.258 cm and translation RPE to 1.308 cm. Full
common-interval ATE increased to 4.971 cm. Both refinements completed with no
resets and all eligible inputs retained. This variant remains unselected.
[Follow-up measurements](euroc_refined_spacing_20261002.json) record the limits.
To reproduce the experimental library:

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_motion_gate_refined05 \
  --preserve-inertial-origin --motion-gated-initialization \
  --keyframe-interval-s 0.25 --refined-keyframe-interval-s 0.5
```


## Exact stereo patch arithmetic trial, October 3

`--fast-stereo-patches` is an isolated, unselected experiment. It reduces row
table reservation, stores 11 correlation distances on the stack, and computes
the original grayscale L1 sum directly. The shared native helper matched
OpenCV on 330,004 random and extreme comparisons with odd row strides.
The arithmetic is unchanged: the maximum 11 x 11 uint8 sum is 30,855, which
is represented exactly by both int and float. Build the native check with:

```bash
c++ -O3 -march=native -std=c++14 -I/usr/include/opencv4 -Ibenchmark \
  benchmark/stereo_patch_cost_check.cc -lopencv_core -o /tmp/stereo_patch_cost_check
/tmp/stereo_patch_cost_check
```

A same-day headless EuRoC reference used 139.64 CPU seconds versus 136.70 for
the experiment (2.1% reduction), with nearly equal peak memory near 695 MiB.
Both retained all 2,911 eligible stereo pairs and completed refinements without
resets. Post-30-second ATE was 1.893 versus 1.771 cm; translation RPE 1.191
versus 1.220 cm. Full common-interval ATE was 1.971 versus 7.825 cm. This
startup-quality difference prevents selecting the optimization on the basis
of late accuracy. A separate constructor-baseline investigation is underway.
[Full evidence](euroc_stereo_cost_20261003.json) preserves both intervals and
the single-trial limit. The approximately fivefold isolated patch speedup is
not a whole-pipeline speedup.


## Stereo baseline initialization defect

The stereo `Frame` constructor called `ComputeStereoMatches()` before assigning
`mb`. Matching uses `mb` to limit disparity search. The isolated builder now
initializes it from `bf / K(0,0)` in the constructor's initializer list, before
matching reads it. This uses current-frame intrinsics and does not rely on the
static camera parameters assigned later in the body.

A native constructor fixture supplies identical synthetic stereo images and
seeds only the raw baseline storage differently. The old library returned 141
valid depths for one seed and 69 for the other. The corrected library returned
141 for both. The fixture uses the core's native compiler flags and correctly
aligned `Frame` storage. This establishes a real initialization defect; full
replay must still check whether it explains the observed startup variation.
[Native evidence and commands](stereo_baseline_initialization_20261003.json)
keep this fix separate from the unselected patch-cost experiment.


The first complete fixed-baseline public replay retained all eligible inputs
and completed both refinements without resets or tracking losses. It used
140.04 CPU seconds and 694.07 MiB. Post-30-second ATE was 2.061 cm and
translation RPE 1.237 cm; full common-interval ATE was 4.039 cm. This removes
the confirmed constructor read but does not establish elimination of all
startup variation or a compute saving. The exact patch-cost variant will be
compared against this corrected constructor.


## Corrected stereo-cost comparison reviewed October 5

The completed October 3 trial combines the initialized-baseline constructor
with `--fast-stereo-patches`. Against the corrected reference, CPU fell from
140.04 to 129.54 seconds (7.5%), with peak RSS 694.07 versus 692.11 MiB.
Both processed all 2,911 eligible stereo pairs, completed both inertial
refinements and had no resets or tracking-loss transitions. Post-30-second
ATE was 2.061 versus 1.791 cm; full common-interval ATE was 4.039 versus
3.815 cm. Each trial used reliable images, queue 64, one OpenCV thread,
native resolution, 600 features and a local BA window of 12. CPU includes
estimator startup and shutdown; drivers, recording and RViz are excluded.

The first pair alone did not establish repeatability. The October 5 repeat
and saved tower replay below retain the candidate as experimental. Earlier
unchanged reference runs showed startup variation. OpenVINS still used less CPU and
memory in the matched public protocol: 69.89 seconds and 121.88 MiB.
[Protocol, core hashes and both accuracy intervals](euroc_fixed_stereo_cost_20261005.json)
retain these limitations. Reports remain outside this repository.


## Saved tower replay, October 5

The corrected reference and patch-cost variant both completed both inertial
refinements with no resets on the successful October 2 recording. Maximum
displacement was 82.96 versus 83.66 cm. Both had two RECENTLY_LOST episodes
and a 0.768-second maximum output gap. Their aligned positions agreed with
1.01 cm RMSE on 3,302 matching poses; this is agreement without independent
ground truth, not an accuracy measurement.

The variant used 154.29 CPU seconds versus 147.42 for the reference, 4.7%
more, with peak RSS 721.16 versus 715.54 MiB. However, tracked pairs differed
(4,434 versus 4,435), as did received IMU counts (14,619 versus 14,634 of
14,639 recorded). This is a diagnostic stability comparison with unequal
delivery, so it establishes no tower cost saving. The original recording
starts its IMU after the cameras and retains bursty receipt timing. The
public protocol repeat below checks the earlier single-pair saving.
The cost variant remains experimental.
[Input counts, gaps, resource measurements and agreement](tower_stereo_cost_20261005.json).


### Repeated corrected public comparison

A second sequential pair used 140.42 versus 133.07 CPU seconds, a 5.2% saving,
with peak RSS 698.95 versus 694.92 MiB. All four ORB trials retained 2,911
eligible stereo pairs and 29,120 IMU samples, completed both refinements, and
had no map resets. Both profiles kept complete pose coverage after startup.

| Pair | Reference CPU s | Candidate CPU s | Saving | Full ATE cm, reference / candidate | Post-30 s ATE cm, reference / candidate |
| --- | ---: | ---: | ---: | ---: | ---: |
| October 3 | 140.04 | 129.54 | 7.5% | 4.039 / 3.815 | 2.061 / 1.791 |
| October 5 | 140.42 | 133.07 | 5.2% | 5.310 / 4.936 | 2.104 / 2.131 |

In the second pair, post-30-second translation RPE was 1.221 versus 1.222 cm;
full-interval RPE was 2.735 versus 2.539 cm. The repeated candidate retains
comparable trajectory quality on this sequence and saves estimator CPU in
both pairs. Startup error still varies, and the original tower replay does
not establish a hardware saving. The working hardware profile remains
selected. [Both pairs, coverage and accuracy intervals](euroc_fixed_stereo_cost_20261005.json)
keep the repeat separate from a general performance or accuracy claim.


## CPU stage profiling

When Linux performance events are unavailable, `--profile-cpu` builds an
isolated diagnostic core with inclusive thread CPU timers. Set
`ORB_PROFILE_OUTPUT` to an absolute CSV path before running a replay. It
measures feature-extraction workers, tracking stages, local mapping and
optimization separately. Sleeping and work on other threads are excluded
from each scope. Parent and child scopes overlap, so do not sum every row.
The public runner records whether profiling was enabled. Instrumented trials
identify bottlenecks; use an uninstrumented core for final cost comparisons.

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_cpu_profile \
  --preserve-inertial-origin --motion-gated-initialization \
  --keyframe-interval-s 0.25 --fast-stereo-patches --profile-cpu
```

A native check spanning two translation units and two concurrent workers
reported both 20 ms CPU loops together and excluded an 80 ms sleep. Output
is written at normal process shutdown; forced termination can omit it.

The first complete CPU profile used 130.20 estimator CPU seconds. The two
extraction workers together used 61.58 seconds (47.3%), the tracking thread
20.14 seconds (15.5%) and local mapping 22.68 seconds (17.4%). Remaining
startup, rectification, ROS and output work is not separated in this profile.
Per-frame inertial pose optimization accounts for approximately 8.05 seconds
inside tracking, so changing solver iteration counts alone cannot close the
CPU gap to OpenVINS. Feature extraction is the next measured target.
[Profile counters, input checks and research references](orb_cpu_profile_20261005.json).

Research consulted includes the [original ORB-SLAM3 paper](https://arxiv.org/abs/2007.11898),
[descriptor-free optical-flow tracking](https://arxiv.org/abs/2008.09870), and
[accuracy-preserving GPU feature extraction](https://arxiv.org/abs/2608.17874).
These describe approaches to investigate; their speedups do not establish a
result on this host. No GPU device is accessible here. OpenVINS' existing
KLT tracker reuses image pyramids and persistent tracks while replenishing
features, which suggests a larger front-end redesign if equivalent arithmetic
optimizations are insufficient. Such a change requires complete trajectory
validation. [Official OpenCV guidance](https://docs.opencv.org/4.13.0/da/d54/group__imgproc__transform.html)
also describes faster fixed-point rectification maps; pixel equality and
local cost must be checked before adopting them.

An AVX2/FMA descriptor prototype matched 100,000 descriptors byte for byte on
an odd-stride image ROI. Its isolated kernel saved only about 11% in two
microbenchmarks, so it was not integrated. The unused prototype is retained
under `/tmp`, and its measurements are in the CPU-profile evidence above.

A second complete profile separates extraction further: 129.46 CPU seconds
with 61.50 in extraction, 29.78 in the outer stereo tracking call and 22.55
in mapping. Within extraction, pyramid construction used 5.02 seconds,
blur 16.78, descriptors 3.24 and orientation 1.60. Detection and cell vector
work used approximately 31.03 seconds after subtracting octree distribution
and orientation; that remainder is not a FAST-only timer. All 2,911 eligible
stereo pairs and 29,120 IMU samples were retained with both refinements and
no resets. These measurements prioritize detection and blur over descriptors.

## Equivalent descriptor blur experiment

`--fast-gaussian` selects a separate-output fixed-point 7x7 sigma-2 blur in
an isolated core. The OpenCV 4.5.4 Q8 coefficients are even, allowing exact
Q7 arithmetic and AVX2 multiply-add without intermediate saturation. The
image boundary remains independently reflected, as in the original clone.
Other OpenCV versions use the ordinary isolated OpenCV operation. No feature
count, image size, pyramid level, detection or matching policy is changed.

The native check matched 62,890,306 pixels across 600 random, constant and
checkerboard images with odd strides and small dimensions. A complete
extractor comparison on 160 EuRoC images matched 96,867 keypoints and
descriptors, including repeated input size changes. The 400-image blur
microbenchmark took 0.326 seconds versus 0.160 with AVX2; a scalar build was
correct but slower. A whole-estimator replay is required before selecting
this change. [Native checks and replay status](orb_gaussian_cost_20261005.json).

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_fixed_gaussian \
  --preserve-inertial-origin --motion-gated-initialization \
  --keyframe-interval-s 0.25 --fast-stereo-patches --fast-gaussian
```

The first full blur replay used 118.57 CPU seconds and 692.3 MiB peak RSS,
compared with the preceding stereo-optimized run's 133.07 seconds and
694.9 MiB. All 2,911 eligible pairs and 29,120 IMU samples were retained,
with both inertial refinements, no resets and complete initialized coverage.
Late ATE was 2.063 versus 2.131 cm; full common-interval ATE was 2.333
versus 4.936 cm. The change alters scheduling despite identical extraction
results. A fresh paired repeat is running, and the hardware cost/quality
gate remains open.

## Pyramid buffer reuse experiment

`--reuse-pyramid` retains the previous bordered pyramid allocation when
its dimensions, type, parent dimensions and ROI offset match. Input size
changes allocate fresh storage. Resize and reflected borders remain exactly
the same, and no extractor class layout changes are introduced. This is an allocation experiment inspired by OpenVINS' reuse of saved
pyramids across detection and matching. OpenVINS constructs each current
pyramid in a fresh vector; it does not implement this allocation cache. The complete
extractor check matched all pyramid pixels, keypoints and descriptors on
160 native/resized EuRoC inputs. The complete replay measured no additional CPU saving.
[Reuse checks](orb_pyramid_reuse_20261005.json).

A fresh reference and blur repeat used 127.78 and 119.08 CPU seconds (6.81%
lower), with 694.1 and 695.9 MiB peak RSS. Both retained complete inputs and
pose coverage, completed both refinements and had no resets. Late ATE was
1.731/1.857 cm and RPE 1.198/1.193 cm; full common ATE was 5.939/3.164 cm.
Initialization and mapping are asynchronous, so feature equality does not
imply identical paths. The two blur candidates used 118.57 and 119.08 CPU
seconds. Tower selection remains pending.

Two detector prototypes were rejected before pipeline integration: a native
cell detector matched 1,160,740 corners but gave no microbenchmark saving,
and full-level detection with exact cell-boundary suppression matched
836,536 corners but cost more. Their evidence is in the profiling JSON;
unused implementations remain under `/tmp`.

The buffer-reuse replay used 119.12 CPU seconds versus the blur reference's
119.08, with late ATE 1.977 versus 1.857 cm and similar RPE. Inputs and
initialized coverage were complete, with both refinements and no resets.
The experiment is not selected because no estimator CPU saving was measured.
The parameter trial changes pyramid spacing from 1.2/eight levels to
1.4/six levels while retaining native resolution and a requested 600
features. It changes feature scales and requires ground-truth validation;
it is not covered by the exact-feature arithmetic checks.

## Stop playback after an estimator failure

The public runner monitors the estimator while playing a dataset. Unexpected
estimator exit or playback timeout stops the owned player and writes an
explicit failure reason in `completion.json`. A known crashing configuration
returned estimator status 139 and was stopped in 8.77 seconds; the earlier
runner kept replaying for 156.39 seconds. Failed runs remain excluded from
trajectory and compute comparisons.
[Failure handling check](early_estimator_failure_20261005.json).

## Disabled place-recognition queue safety

GDB reproduced a SIGSEGV in `LoopClosing::Run` with `loopClosing: 0`. The
upstream disabled detector returns before consuming queued keyframes. The
worker then copies its uninitialized current keyframe pointer into the last
pointer and dereferences it on the next iteration. The isolated builder now
initializes that pointer and prevents queuing disabled place-recognition
work. Active-loop handling is unchanged. The formerly crashing wider-pyramid
configuration is replaying with the correction. This diagnosis does not
establish the cause of historical hardware resets.
[Diagnosis and validation](orb_disabled_loop_queue_20261005.json).

The corrected wider-pyramid replay completed without the former crash and
retained all 2,911 eligible pairs and 29,120 IMU samples, both refinements
and complete pose coverage. It used 105.35 CPU seconds, with late ATE
1.805 cm and RPE 1.214 cm. A matching reference with the same loop-worker
fix is running; the previous 119.08-second blur reference used the old
loop worker. Do not attribute the whole difference to spacing yet.
[Spacing trials](orb_pyramid_spacing_20261005.json).

The matching corrected reference used 119.14 CPU seconds, versus 105.35
with 1.4 spacing (11.57% lower). Late ATE was 1.955/1.805 cm and RPE
1.174/1.214 cm, with complete coverage in both. The small RPE increase
and repeatability still need checking before selecting the spacing change.
A 1.6/five-level trial is next, using the same corrected core.

The 1.6/five-level run used 100.44 CPU seconds, late ATE 1.699 cm and RPE
1.203 cm. It retained every eligible pair and IMU sample, both refinements
and complete pose coverage, with no resets. Against the same corrected
reference, CPU fell 15.70%, ATE improved and RPE rose slightly (1.174 to
1.203 cm). This remains an experimental single-sequence result. A 500-feature
trial is measuring a smaller budget at the same native resolution and
pyramid spacing; it changes feature selection and requires trajectory checks.

Fixed-point rectification maps matched all 115,507,200 pixels across 320
rectifications, but 800 local remaps took 1.213 seconds versus 1.166 with
float maps on the installed OpenCV 4.5.4. The suggested map conversion is
not integrated because it did not reduce local cost.

At the same 1.6/five-level spacing, the 500-feature trial used 92.86 CPU
seconds and 685.0 MiB peak RSS versus 100.44 seconds and 699.1 MiB at 600
features. Late ATE improved from 1.699 to 1.592 cm and RPE from 1.203 to
1.181 cm. Both retained complete inputs and pose coverage, both refinements
and no resets. This is a first operating-point trial on one sequence; no
hardware profile is changed and repeat/live validation remains necessary.

## Packed vocabulary loading experiment

`--packed-vocabulary` specializes the existing text loader without changing
the vocabulary class layout. It removes repeated descriptor stringstreams
and stores the valid descriptor bytes in shared row storage. The ordinary
loader's extra invalid node after the final newline is omitted.

All 1,082,073 valid nodes matched their IDs, parent links, weights, descriptor
bytes and valid children. The same 124,315 random and dataset descriptor
queries produced identical word IDs. Native loader CPU fell from 4.941 to
0.567 seconds and check-executable peak RSS from 457.3 to 258.9 MiB. These
are vocabulary-check measurements. The first complete estimator pair used
92.86 versus 88.57 CPU seconds and 685.0 versus 486.4 MiB peak RSS.
Both retained all 2,911 eligible stereo pairs and 29,120 IMU samples,
completed both inertial refinements and had no resets. Late ATE was
1.592 versus 1.742 cm; one-second RPE was 1.181 versus 1.214 cm.
Full common-interval ATE was 2.415 versus 1.777 cm. These are experimental
single-sequence measurements at 500 requested features, scale 1.6/five
levels, native resolution and BA cap 12. The repeat used 88.87 CPU seconds,
487.8 MiB peak RSS, 1.737 cm late ATE and 1.177 cm late RPE, with complete
input and initialized pose coverage, both refinements and no resets.
Hardware validation remains. Saved legacy atlas compatibility is untested.
[Loader checks and limits](orb_packed_vocabulary_20261005.json).


## Fresh tower check of packed vocabulary and wider pyramid

The explicit 600-feature, scale-1.6/five-level live profile completed the
120.60-second movement check on the unchanged measured tower. Maximum
initialized displacement was 72.4 cm, maximum pose step 7.52 cm and maximum
pose gap 368 ms. Both inertial refinements completed, with zero map resets
and zero LOST transitions. Three RECENTLY_LOST events recovered. Continuous
motion inside the desk/height limits and no observed flights, jumps or resets
were confirmed.

The estimator including ROS launch used 133.31 CPU seconds over 165.01 seconds
(80.8% of one core), with 518.2 MiB peak RSS. Tracking averaged 11.67 ms over
4,892 frames. The diagnostic full pipeline used 264.03 CPU seconds over
183.21 seconds and includes camera, IMU, splitter, recording and RViz. These
are descriptive live measurements, not a matched comparison to older runs
or proof of trajectory accuracy against independent ground truth. The public
500-feature profile still needs separate hardware validation.
[Live evidence and limitations](tower_pyramid_validation_20261005.json).


## Refined keyframe spacing with the corrected packed core

A fresh test repeats the 0.5-second spacing policy after VIBA2 with the
corrected baseline, disabled-loop queue, exact stereo/blur arithmetic and
packed vocabulary. CPU fell 6.9% from the two-reference mean of 88.72 to
82.56 seconds, but late ATE rose 22.4% from 1.740 to 2.129 cm and late RPE
rose 17.6% from 1.196 to 1.406 cm. Full common-interval ATE also rose 41.0%.
All 2,911 eligible frames and 29,120 IMU samples were retained, both refinements
completed and there were no active resets. This policy remains unselected:
the cost reduction comes with worse error on both evaluation intervals.
[Comparison and percentages](orb_packed_refined_spacing_20261005.json).


## Smaller local BA window with the packed core

Keeping keyframe spacing at 0.25 seconds and reducing the BA cap from 12 to
10 used 87.03 CPU seconds, 1.9% below the two-reference mean of 88.72 seconds.
Late ATE fell 5.2% from 1.740 to 1.649 cm and RPE fell 3.0% from 1.196 to
1.159 cm. However, full common-interval ATE rose 81.1% from 1.857 to 3.364 cm;
full RPE rose 41.4%. This one-run result remains unselected. Initialization
scheduling can change error, but a larger full-interval error cannot be
ignored when selecting an optimization that must preserve trajectory quality.
[Both intervals and percentage changes](orb_packed_ba10_20261005.json).
