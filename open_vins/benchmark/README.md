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

The HW290 launchers save individual run summaries under `results/`. These
include sensor faults, logged estimator timings, process CPU and peak memory.
Manual out and back distances are diagnostic checks, not time resolved ATE.
See [the HW290 repair notes](../hw290_stereo/REPAIR_NOTES.md) for operating
details. Dated work reports are kept separately in `/home/ac/Work_Reports/`.

## Experimental temporal initialization motion window

The default isolated core retains the upstream motion-reset condition. A separate
prototype evaluates the same 2 cm and 5 cm thresholds using keyframes roughly
0.5 seconds apart, spanning about one second. Adjacent keyframes can be only
0.06 seconds apart during initialization, making their travel small even while
the rig is moving. This changes both the reset decision and accumulated motion
time. It does not disable the reset or alter trajectory output.

Build it in a separate directory; the builder refuses to overwrite the default
core with this experimental option:

```bash
python3 benchmark/build_orb_core.py --temporal-init-motion \
  --output benchmark/results/orb_temporal_core
```

Select that directory explicitly with `ORB_CORE_DIR` when launching the ORB
hardware script. The default launcher continues to use `benchmark/build_orb_core`.
The build manifest records the option and source hashes; the incremental patch
is `patches/orb_temporal_init_motion.patch`.

Two replays of the October 2 movement recording completed both inertial
refinements and stayed within 0.62 m and 0.69 m maximum displacement, with 10
and 8 map resets. Both included stationary startup. A clean repeat used 107.7%
of one core and 946.9 MiB peak estimator RSS, so this has not achieved lower
cost than OpenVINS. Bag completion does not guarantee identical delivered frame
counts; the runs processed 4237 and 4168 frames. The first run overlapped a
diagnostic build and its CPU result is excluded from ranking.

These are desk-bound checks without independent ground truth. A second recording
and fresh live movement test are needed before selecting this core for normal
use. Sampling over a longer interval can miss motion that returns to the same
position between samples, so this remains an experiment.
