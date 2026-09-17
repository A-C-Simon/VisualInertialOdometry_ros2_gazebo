#!/usr/bin/env python3
"""Align Gazebo odometry with ORB-SLAM's arbitrary world yaw for RViz only."""
import math
from collections import deque
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FrameAligner(Node):
    def __init__(self):
        super().__init__('orbslam3_frame_aligner')
        self.declare_parameter('window_start_m', 1.0)
        self.declare_parameter('window_end_m', 2.5)
        self.distance = 0.0
        self.previous_odom = None
        self.start = None
        self.end = None
        self.origin = None
        self.odom_history = deque(maxlen=400)
        self.transform = (0.0, 0.0, 0.0, 0.0)
        self.aligned = False
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseStamped, '/orbslam3/pose', self.on_slam, 20)
        self.create_timer(0.05, self.publish_transform)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def add_pair(self, odom, slam):
        if self.origin is None:
            self.origin = (odom, slam)
            self.previous_odom = odom
        self.distance += math.hypot(
            odom[0] - self.previous_odom[0], odom[1] - self.previous_odom[1])
        self.previous_odom = odom
        pair = (odom, slam)
        if self.start is None and self.distance >= self.get_parameter('window_start_m').value:
            self.start = pair
        if self.end is None and self.distance >= self.get_parameter('window_end_m').value:
            self.end = pair
        self.try_align()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_history.append((self.stamp_ns(msg.header.stamp), (p.x, p.y, p.z)))

    def on_slam(self, msg):
        if not self.odom_history:
            return
        p = msg.pose.position
        stamp = self.stamp_ns(msg.header.stamp)
        odom_stamp, odom = min(self.odom_history, key=lambda item: abs(item[0] - stamp))
        if abs(odom_stamp - stamp) <= 100000000:  # synchronized within 100 ms
            self.add_pair(odom, (p.x, p.y, p.z))

    def try_align(self):
        if self.aligned or self.end is None:
            return
        odom_start, slam_start = self.start
        odom_end, slam_end = self.end
        odom_heading = math.atan2(odom_end[1] - odom_start[1], odom_end[0] - odom_start[0])
        slam_heading = math.atan2(slam_end[1] - slam_start[1], slam_end[0] - slam_start[0])
        yaw = math.atan2(math.sin(slam_heading - odom_heading),
                         math.cos(slam_heading - odom_heading))
        c, s = math.cos(yaw), math.sin(yaw)
        odom0, slam0 = self.origin
        tx = slam0[0] - (c * odom0[0] - s * odom0[1])
        ty = slam0[1] - (s * odom0[0] + c * odom0[1])
        self.transform = (tx, ty, slam0[2] - odom0[2], yaw)
        self.aligned = True
        self.get_logger().info('Aligned orb_map->odom: yaw=%.2f deg' % math.degrees(yaw))

    def publish_transform(self):
        tx, ty, tz, yaw = self.transform
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'orb_map'
        msg.child_frame_id = 'odom'
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = FrameAligner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
