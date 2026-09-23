#!/usr/bin/env python3
"""Publish the true Gazebo model pose as the comparison ground truth path.

OpenVINS publishes ``pathimu`` in its ``global`` frame. The ROS subscription
mode does not have OpenVINS' internal simulator object, so its built-in
``pathgt`` topic is empty. This node republishes Gazebo's exact model pose as
``/ov_msckf/pathgt`` in the ``world`` frame. The align_frames node publishes
the measured ``global`` to ``world`` transform for RViz2.
"""

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class GroundPath(Node):
    def __init__(self):
        super().__init__('ground_path')
        self.declare_parameter('max_poses', 20000)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.path = Path()
        self.path.header.frame_id = 'world'
        self.path_pub = self.create_publisher(Path, '/ov_msckf/pathgt', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ov_msckf/posegt', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)

    def on_models(self, msg: ModelStates):
        try:
            i = msg.name.index('ov_rover')
        except ValueError:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose = msg.pose[i]

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            del self.path.poses[:-self.max_poses]

        self.pose_pub.publish(pose)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = GroundPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
