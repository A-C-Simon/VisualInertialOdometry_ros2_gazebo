import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configured_node(context, package_share):
    settings_path = LaunchConfiguration("settings_path").perform(context)
    settings = Path(settings_path).read_text()
    offset_arg = LaunchConfiguration("camera_imu_offset").perform(context)
    if offset_arg == "auto":
        match = re.search(r"^HW290\.CameraImuOffset:\s*([^#\s]+)", settings, re.MULTILINE)
        offset = float(match.group(1)) if match else 0.0
    else:
        offset = float(offset_arg)
    extra_env = {}
    if re.search(r'Camera\.type:\s*["\']?Rectified', settings):
        candidates = [p / "open_vins/benchmark/build_orb_core"
                      for p in Path(package_share).resolve().parents]
        core_dir = os.environ.get("ORB_CORE_DIR")
        if not core_dir:
            core_dir = next((str(p) for p in candidates if (p / "libORB_SLAM3.so").is_file()), "")
        if not core_dir or not (Path(core_dir) / "libORB_SLAM3.so").is_file():
            raise RuntimeError("Rectified HW290 input requires the isolated ORB core. Run open_vins/benchmark/build_orb_core.py.")
        extra_env["LD_LIBRARY_PATH"] = core_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    return [make_node(offset, extra_env)]


def make_node(offset, extra_env):
    return Node(
        package="orb_slam_ros2",
        executable="stereo_imu_node",
        name="orb_slam3_stereo_imu",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        additional_env=extra_env,
        parameters=[{
            "vocabulary_path": LaunchConfiguration("vocabulary_path"),
            "settings_path": LaunchConfiguration("settings_path"),
            "left_topic": LaunchConfiguration("left_topic"),
            "right_topic": LaunchConfiguration("right_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "use_viewer": LaunchConfiguration("viewer"),
            "reliable_images": LaunchConfiguration("reliable_images"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "publish_bootstrap_poses": LaunchConfiguration("publish_bootstrap_poses"),
            "opencv_threads": LaunchConfiguration("opencv_threads"),
            "visualization_hz": LaunchConfiguration("visualization_hz"),
            "camera_imu_offset": offset,
            "trajectory_path": LaunchConfiguration("trajectory_path"),
            "keyframe_trajectory_path": LaunchConfiguration("keyframe_trajectory_path"),
            "timing_path": LaunchConfiguration("timing_path"),
            "online_trajectory_path": LaunchConfiguration("online_trajectory_path"),
        }],
    )


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
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("reliable_images", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("publish_bootstrap_poses", default_value="false"),
        DeclareLaunchArgument("opencv_threads", default_value="1"),
        DeclareLaunchArgument("visualization_hz", default_value="5.0"),
        DeclareLaunchArgument("camera_imu_offset", default_value="auto"),
        DeclareLaunchArgument("trajectory_path", default_value="vio_session.txt"),
        DeclareLaunchArgument(
            "keyframe_trajectory_path", default_value="kf_vio_session.txt"
        ),
        DeclareLaunchArgument("timing_path", default_value="vio_timing.txt"),
        DeclareLaunchArgument("online_trajectory_path", default_value=""),
        DeclareLaunchArgument("namespace", default_value="orbslam_vio"),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=configured_node, args=[package_share])])
