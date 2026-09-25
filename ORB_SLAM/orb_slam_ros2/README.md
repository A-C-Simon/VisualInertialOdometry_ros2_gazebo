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

## Stereo-inertial (HW-290 and ELP)

`stereo_imu_node` runs `System::IMU_STEREO` on rectified `/cam0/image_raw`
and `/cam1/image_raw` (for example from the `ov_hw290` splitter at 640x480)
plus `/imu0` (HW-290 at 100 Hz). Images and IMU share ROS header time as
the timestamp base.

```bash
ros2 launch orb_slam_ros2 stereo_inertial_topics.launch.py \
  namespace:=orbslam_vio viewer:=true
```

Topics under the namespace are `pose`, `path`, `tracked_map_points`,
`tracking_image`, and `tracking_state`, plus TF `map` to
`camera_optical_frame` (published only while tracking is valid).
Parameters are `vocabulary_path`, `settings_path` (default
`config/ELP_640x480_inertial.yaml`), `left_topic`, `right_topic`,
`imu_topic`, `use_viewer`, `publish_tf`, `trajectory_path`,
`keyframe_trajectory_path`, and `timing_path` (per-frame TrackStereo
timings). Frames without enough spanning IMU samples are skipped; see the
parent README for the reason.

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

To reduce CPU use during a stereo run, set `localization_only:=true`. The node
will allow stereo initialization to create the initial map, then stop local
mapping. With no new keyframes, loop closing and global bundle adjustment will
also remain inactive:

```bash
ros2 launch orb_slam_ros2 stereo_topics.launch.py localization_only:=true
```

For the camera-only Gazebo test, use:

```bash
./orbslam3_gazebo.sh --auto --localization-only
```

ORB-SLAM3 uses the camera optical convention (x right, y down, z forward).
Consequently the `map` axes inherit the initial optical-camera orientation.
Use a static/dynamic transform downstream if a REP-103 body frame is needed.

On clean shutdown the EuRoC-format frame and keyframe trajectories are saved
to `trajectory_path` and `keyframe_trajectory_path`. Set either parameter to an
empty string to disable that file.
