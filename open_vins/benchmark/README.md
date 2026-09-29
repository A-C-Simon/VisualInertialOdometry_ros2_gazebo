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
the post-initialization interval. The included reports state dataset, host,
configuration and limits for each recorded comparison.

## Hardware runs

The HW290 launchers save individual run summaries under `results/`. These
include sensor faults, logged estimator timings, process CPU and peak memory.
Manual out and back distances are diagnostic checks, not time resolved ATE.
See [the HW290 repair notes](../hw290_stereo/REPAIR_NOTES.md) and the dated
reports linked from [the OpenVINS README](../README.md).
