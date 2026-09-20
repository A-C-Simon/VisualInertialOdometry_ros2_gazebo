"""Launch Gazebo Classic, the stereo rover, and camera-only ORB-SLAM3."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('orbslam3_rover_sim')
    world = os.path.join(pkg, 'worlds', 'small_room.world')
    urdf = os.path.join(pkg, 'urdf', 'rover.urdf')
    model_sdf = os.path.join(pkg, 'models', 'orb_rover', 'model.sdf')
    orb_share = get_package_share_directory('orb_slam_ros2')
    default_vocabulary = os.path.expanduser('~/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    default_orb_settings = os.path.join(
        orb_share, 'config', 'rover_gazebo_stereo.yaml')

    with open(urdf) as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world,
                          'verbose': 'false',
                          'gui': LaunchConfiguration('gui')}.items())

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        # z=0.01: wheels touch gently (0.1 drops/bounces the rover into a spin)
        arguments=['-file', model_sdf, '-entity', 'orb_rover', '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen')

    # Everything simulated runs on sim time so behavior is identical at any
    # real-time factor (steady headless slow-motion would otherwise shrink
    # every auto phase and stamp TFs with the wrong clock).
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc},
                           {'use_sim_time': True}],
               output='screen')

    auto = Node(package='orbslam3_rover_sim', executable='auto_loop.py',
                parameters=[{'enabled': LaunchConfiguration('auto')},
                            {'mode': LaunchConfiguration('mode')},
                            {'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('auto')),
                output='screen')

    # Pure stereo ORB-SLAM3: only the two image topics are consumed. The
    # Gazebo model has no IMU sensor and no inertial SLAM mode is selected.
    orb_slam = Node(
        package='orb_slam_ros2', executable='stereo_node',
        namespace='orbslam3', name='stereo', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'input_mode': 'topics',
            'left_topic': '/cam0/image_raw',
            'right_topic': '/cam1/image_raw',
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'settings_path': LaunchConfiguration('orb_settings_path'),
            'use_viewer': LaunchConfiguration('orb_viewer'),
            'publish_tf': True,
            'map_frame': 'map',
            'camera_frame': 'orb_camera_optical',
            'trajectory_path': LaunchConfiguration('trajectory_path'),
            'keyframe_trajectory_path': LaunchConfiguration('keyframe_trajectory_path'),
        }],
        condition=IfCondition(LaunchConfiguration('orb_slam')))

    # The ORB world starts at the first left optical-camera pose. This fixed
    # transform places Gazebo's odom axes/origin in that optical map frame so
    # RViz can show the truth robot and ORB outputs in one TF tree.
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='orb_map_to_odom',
        arguments=['--x', '0.055', '--y', '0.28', '--z', '-0.22',
                   '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=IfCondition(LaunchConfiguration('orb_slam')))

    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='true', description='drive automatically'),
        DeclareLaunchArgument('mode', default_value='circle', description='auto drive mode: circle or square'),
        DeclareLaunchArgument('rviz', default_value='false', description='open rviz'),
        DeclareLaunchArgument('gui', default_value='true', description='gazebo GUI (false=headless)'),
        DeclareLaunchArgument('orb_slam', default_value='true', description='run stereo ORB-SLAM3'),
        DeclareLaunchArgument('orb_viewer', default_value='true', description='open ORB-SLAM3 Pangolin viewer'),
        DeclareLaunchArgument('vocabulary_path', default_value=default_vocabulary),
        DeclareLaunchArgument('orb_settings_path', default_value=default_orb_settings),
        DeclareLaunchArgument('trajectory_path', default_value='/tmp/orbslam3_gazebo_trajectory.txt'),
        DeclareLaunchArgument('keyframe_trajectory_path', default_value='/tmp/orbslam3_gazebo_keyframes.txt'),
        gazebo, spawn, rsp, auto, orb_slam, map_to_odom,
        # manual drive: ros2 run orbslam3_rover_sim key_teleop.py
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'rover.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
