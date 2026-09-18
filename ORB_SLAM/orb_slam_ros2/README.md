# orb_slam_ros2

ROS 2 Humble stereo wrapper around the tested ORB-SLAM3 build in
`~/ORB_SLAM3`. It preserves the known-good direct ELP side-by-side capture and
also accepts synchronized ROS image topics.

## Build

From this repository root:

```bash
source /opt/ros/humble/setup.bash
export ORB_SLAM3_ROOT="$HOME/ORB_SLAM3"
colcon build --packages-select orb_slam_ros2 --symlink-install \
  --cmake-args -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"
source install/setup.bash
```

ORB-SLAM3 must already be built, including `lib/libORB_SLAM3.so`, DBoW2, g2o,
and `Vocabulary/ORBvoc.txt`.

## Run the tested ELP camera directly

```bash
ros2 launch orb_slam_ros2 stereo_live.launch.py device_id:=0
```

The defaults request MJPG at 3200x1200 and split each frame into two
1600x1200 images, exactly like the standalone tested program. Override paths
if needed:

```bash
ros2 launch orb_slam_ros2 stereo_live.launch.py \
  vocabulary_path:=/path/to/ORBvoc.txt \
  settings_path:=/path/to/stereo.yaml \
  viewer:=false
```

## Run from ROS image topics

```bash
ros2 launch orb_slam_ros2 stereo_topics.launch.py \
  left_topic:=/camera/left/image_raw \
  right_topic:=/camera/right/image_raw
```

The image headers must contain monotonic timestamps. Approximate-time
synchronization defaults to a 20 ms maximum separation.

The `config/rover_gazebo_stereo.yaml` settings are provided for the
`ov_rover_sim` 640x400, 20 Hz, 0.11 m-baseline cameras. They select pure
stereo SLAM (`System::STEREO`); no simulated IMU topic is consumed.

## ROS interfaces

All relative names are under the node namespace if one is supplied.

| Interface | Type | Meaning |
|---|---|---|
| `pose` | `geometry_msgs/msg/PoseStamped` | Current camera optical-frame pose in `map` |
| `path` | `nav_msgs/msg/Path` | Camera trajectory |
| `tracked_map_points` | `sensor_msgs/msg/PointCloud2` | Map points observed in the current frame |
| `tracking_image` | `sensor_msgs/msg/Image` | Left image with tracked points in green |
| `tracking_state` | `std_msgs/msg/Int32` | ORB state: 0 no image, 1 initializing, 2 OK, 3 recently lost, 4 lost, 5 OK_KLT |
| `map -> camera_optical_frame` | TF | Published only while tracking is valid |
| `reset` | `std_srvs/srv/Trigger` | Reset the atlas |
| `activate_localization_mode` | `std_srvs/srv/Trigger` | Stop map growth |
| `activate_slam_mode` | `std_srvs/srv/Trigger` | Resume map growth |

ORB-SLAM3 uses the camera optical convention (x right, y down, z forward).
Consequently the `map` axes inherit the initial optical-camera orientation.
Use a static/dynamic transform downstream if a REP-103 body frame is needed.

On clean shutdown the EuRoC-format frame and keyframe trajectories are saved
to `trajectory_path` and `keyframe_trajectory_path`. Set either parameter to an
empty string to disable that file.
