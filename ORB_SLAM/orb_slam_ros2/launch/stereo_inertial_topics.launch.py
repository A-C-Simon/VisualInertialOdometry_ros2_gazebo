import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("orb_slam_ros2")
    orb_slam3_root = os.environ.get("ORB_SLAM3_ROOT", os.path.expanduser("~/ORB_SLAM3"))

    arguments = [
        DeclareLaunchArgument(
            "vocabulary_path",
            default_value=os.path.join(orb_slam3_root, "Vocabulary", "ORBvoc.txt"),
        ),
        DeclareLaunchArgument(
            "settings_path",
            default_value=os.path.join(package_share, "config", "ELP_640x480_inertial.yaml"),
        ),
        DeclareLaunchArgument("left_topic", default_value="/cam0/image_raw"),
        DeclareLaunchArgument("right_topic", default_value="/cam1/image_raw"),
        DeclareLaunchArgument("imu_topic", default_value="/imu0"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("trajectory_path", default_value="vio_session.txt"),
        DeclareLaunchArgument(
            "keyframe_trajectory_path", default_value="kf_vio_session.txt"
        ),
        DeclareLaunchArgument("timing_path", default_value="vio_timing.txt"),
        DeclareLaunchArgument("namespace", default_value="orbslam_vio"),
    ]

    node = Node(
        package="orb_slam_ros2",
        executable="stereo_imu_node",
        name="orb_slam3_stereo_imu",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "vocabulary_path": LaunchConfiguration("vocabulary_path"),
                "settings_path": LaunchConfiguration("settings_path"),
                "left_topic": LaunchConfiguration("left_topic"),
                "right_topic": LaunchConfiguration("right_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "use_viewer": LaunchConfiguration("viewer"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "trajectory_path": LaunchConfiguration("trajectory_path"),
                "keyframe_trajectory_path": LaunchConfiguration(
                    "keyframe_trajectory_path"
                ),
                "timing_path": LaunchConfiguration("timing_path"),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
