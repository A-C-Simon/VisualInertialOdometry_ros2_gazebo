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
