# OpenVINS rover simulator

This package starts the Gazebo rover, camera and IMU sensors, RViz2, optional
automatic motion, and the ground truth path publisher used by the OpenVINS
comparison.

The true rover pose comes from `/gazebo/model_states`, filtered for the
`ov_rover` model by `scripts/ground_path.py`. It is published as
`/ov_msckf/posegt` and `/ov_msckf/pathgt` in the `world` frame. This avoids
using wheel odometry as ground truth.

Build from the OpenVINS directory:

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
source /opt/ros/humble/setup.bash
colcon --log-base log_vio build --base-paths . --build-base build_vio \
  --install-base install_vio --symlink-install \
  --packages-select ov_core ov_data ov_eval ov_init ov_msckf ov_rover_sim
```

Run the complete test from `/home/ac/VisualInertialOdometry_ros2_gazebo/open_vins`
with `./run_vio_gazebo.sh`.
