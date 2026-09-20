#!/usr/bin/env python3
"""Project the camera-only ORB-SLAM path onto the Gazebo ground plane.

The raw stereo estimate remains available on /orbslam3/path. This display path
is transformed into odom, then roll, pitch, and height are removed because the
camera-only system has no gravity sensor to make those components observable.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class GroundPath(Node):
    def __init__(self):
        super().__init__('ground_path')
        self.declare_parameter('input_topic', '/orbslam3/pose')
        self.declare_parameter('output_topic', '/orbslam3/ground_path')
        self.declare_parameter('ground_frame', 'odom')
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('max_path_length', 10000)

        self.ground_frame = self.get_parameter('ground_frame').value
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.max_path_length = max(1, int(self.get_parameter('max_path_length').value))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.path = Path()
        self.pub = self.create_publisher(
            Path, self.get_parameter('output_topic').value, 10)
        self.sub = self.create_subscription(
            PoseStamped, self.get_parameter('input_topic').value,
            self.pose_callback, 10)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def yaw_quaternion(yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def pose_callback(self, msg):
        try:
            # The static map-to-odom transform is valid for all timestamps,
            # but simulated clocks can briefly publish a pose before the TF
            # cache has received that transform at the exact pose timestamp.
            # Try the stamped lookup first, then use the latest transform.
            try:
                transform = self.buffer.lookup_transform(
                    self.ground_frame, msg.header.frame_id, msg.header.stamp,
                    timeout=Duration(seconds=0.05))
            except TransformException:
                transform = self.buffer.lookup_transform(
                    self.ground_frame, msg.header.frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            pose = do_transform_pose_stamped(msg, transform)
        except TransformException as error:
            self.get_logger().debug('Waiting for path transform: %s', error)
            return

        yaw = self.yaw_from_quaternion(pose.pose.orientation)
        pose.header.frame_id = self.ground_frame
        pose.pose.position.z = self.ground_z
        pose.pose.orientation = self.yaw_quaternion(yaw)
        self.path.header = pose.header
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = GroundPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
