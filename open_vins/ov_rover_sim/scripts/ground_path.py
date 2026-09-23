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
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import math


class GroundPath(Node):
    def __init__(self):
        super().__init__('ground_path')
        self.declare_parameter('max_poses', 20000)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.path = Path()
        self.path.header.frame_id = 'world'
        self.path_pub = self.create_publisher(Path, '/ov_msckf/pathgt', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ov_msckf/posegt', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.model_pose = None
        self.odom_pose = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_models(self, msg: ModelStates):
        try:
            i = msg.name.index('ov_rover')
        except ValueError:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose = msg.pose[i]
        self.model_pose = pose.pose
        self.publish_world_odom(pose.header.stamp)

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            del self.path.poses[:-self.max_poses]

        self.pose_pub.publish(pose)
        self.path_pub.publish(self.path)

    def on_odom(self, msg: Odometry):
        self.odom_pose = msg.pose.pose
        self.publish_world_odom(msg.header.stamp)

    def publish_world_odom(self, stamp):
        """Bridge Gazebo's world pose to the diff-drive odom TF tree.

        Gazebo publishes odom -> base_footprint, while model_states gives
        world -> base_footprint. Publishing world -> odom makes the robot
        model reachable from RViz's global -> world fixed-frame chain.
        """
        if self.model_pose is None or self.odom_pose is None:
            return
        g = self.model_pose
        o = self.odom_pose
        gy = math.atan2(2.0 * (g.orientation.w * g.orientation.z +
                               g.orientation.x * g.orientation.y),
                        1.0 - 2.0 * (g.orientation.y * g.orientation.y +
                                      g.orientation.z * g.orientation.z))
        oy = math.atan2(2.0 * (o.orientation.w * o.orientation.z +
                               o.orientation.x * o.orientation.y),
                        1.0 - 2.0 * (o.orientation.y * o.orientation.y +
                                      o.orientation.z * o.orientation.z))
        yaw = gy - oy
        c, s = math.cos(yaw), math.sin(yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = g.position.x - (c * o.position.x - s * o.position.y)
        t.transform.translation.y = g.position.y - (s * o.position.x + c * o.position.y)
        t.transform.translation.z = g.position.z - o.position.z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)


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
