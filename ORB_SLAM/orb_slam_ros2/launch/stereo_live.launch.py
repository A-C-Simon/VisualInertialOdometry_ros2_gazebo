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
            default_value=os.path.join(package_share, "config", "ELP_1600x1200.yaml"),
        ),
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("trajectory_path", default_value="live_session.txt"),
        DeclareLaunchArgument(
            "keyframe_trajectory_path", default_value="kf_live_session.txt"
        ),
    ]

    node = Node(
        package="orb_slam_ros2",
        executable="stereo_node",
        name="orb_slam3_stereo",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "input_mode": "device",
                "vocabulary_path": LaunchConfiguration("vocabulary_path"),
                "settings_path": LaunchConfiguration("settings_path"),
                "device_id": LaunchConfiguration("device_id"),
                "use_viewer": LaunchConfiguration("viewer"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "trajectory_path": LaunchConfiguration("trajectory_path"),
                "keyframe_trajectory_path": LaunchConfiguration(
                    "keyframe_trajectory_path"
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
