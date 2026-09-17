"""Launch the Gazebo rover, stereo-inertial ORB-SLAM3, and optional RViz."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('orbslam3_rover_sim')
    orb = get_package_share_directory('orbslam3')
    gazebo_share = get_package_share_directory('gazebo_ros')
    world = os.path.join(sim, 'worlds', 'small_room.world')
    model = os.path.join(sim, 'models', 'ov_rover', 'model.sdf')
    urdf = os.path.join(sim, 'urdf', 'rover.urdf')
    settings = os.path.join(sim, 'config', 'rover_stereo_inertial.yaml')
    vocabulary = os.path.join(orb, 'vocabulary', 'ORBvoc.txt')
    with open(urdf, encoding='utf-8') as stream:
        robot_description = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto', default_value='true'),
        DeclareLaunchArgument('mode', default_value='circle'),
        DeclareLaunchArgument('orb_viewer', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': LaunchConfiguration('gui')}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-file', model, '-entity', 'orbslam3_rover', '-z', '0.01']),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='orbslam3_rover_sim', executable='auto_loop.py', output='screen',
             condition=IfCondition(LaunchConfiguration('auto')),
             parameters=[{'enabled': True, 'mode': LaunchConfiguration('mode'), 'use_sim_time': True}]),
        Node(package='orbslam3', executable='stereo-inertial', name='orbslam3', output='screen',
             arguments=[vocabulary, settings, 'false', 'false', LaunchConfiguration('orb_viewer')],
             remappings=[('camera/left', '/cam0/image_raw'),
                         ('camera/right', '/cam1/image_raw'), ('imu', '/imu0')],
             parameters=[{'use_sim_time': True, 'map_frame': 'orb_map'}]),
        Node(package='orbslam3_rover_sim', executable='align_frames.py', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(sim, 'rviz', 'orbslam3_rover.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
