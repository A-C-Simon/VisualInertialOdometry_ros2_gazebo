# VIO benchmark inputs

`euroc_player.py` replays an ASL-format EuRoC sequence as the three ROS 2
topics consumed by OpenVINS:

- `/cam0/image_raw`
- `/cam1/image_raw`
- `/imu0`

It also records `/ov_msckf/poseimu` in TUM trajectory format. The player keeps
the original sensor timestamps and interleaves the 200 Hz IMU with synchronized
stereo pairs. It reads images on demand so the dataset is not duplicated in the
repository.

Example using the short public sample already installed with Kimera-VIO:

```bash
source /opt/ros/humble/setup.bash
source install_vio/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav rviz_enable:=false

# Run after the estimator is ready.
python3 benchmark/euroc_player.py \
  /home/ac/ros2_ws/src/Kimera-VIO/tests/data/MicroEurocDataset \
  --trajectory /tmp/openvins_micro_euroc.txt
```

The MicroEuroc sample has only 95 synchronized stereo pairs spanning 4.7
seconds. It is suitable for checking topic wiring and startup, but it is too
short for a defensible accuracy or steady-state compute comparison. Use a full
EuRoC sequence such as `MH_01_easy` for reported ATE and resource results.

`monitor_process.py` samples Linux process CPU time and resident memory without
including the dataset player. `evaluate_ate.py` timestamp-associates an output
trajectory with EuRoC ground truth, performs a rigid SE(3) alignment without
scale correction, and reports translational absolute trajectory error.
