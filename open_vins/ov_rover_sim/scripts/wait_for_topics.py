#!/usr/bin/env python3
"""Wait for the complete Gazebo rover sensor set before starting OpenVINS."""

import argparse
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu


class TopicWaiter(Node):
    def __init__(self, stereo):
        super().__init__('ov_rover_topic_waiter')
        self.seen = {
            '/gazebo/model_states': False,
            '/odom': False,
            '/imu0': False,
            '/cam0/image_raw': False,
        }
        if stereo:
            self.seen['/cam1/image_raw'] = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._topic_subscriptions = [
            self.create_subscription(ModelStates, '/gazebo/model_states',
                                     lambda _: self.mark('/gazebo/model_states'), qos),
            self.create_subscription(Odometry, '/odom', lambda _: self.mark('/odom'), qos),
            self.create_subscription(Imu, '/imu0', lambda _: self.mark('/imu0'), qos),
            self.create_subscription(Image, '/cam0/image_raw',
                                     lambda _: self.mark('/cam0/image_raw'), qos),
        ]
        if stereo:
            self._topic_subscriptions.append(
                self.create_subscription(Image, '/cam1/image_raw',
                                         lambda _: self.mark('/cam1/image_raw'), qos))

    def mark(self, topic):
        self.seen[topic] = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stereo', action='store_true')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = TopicWaiter(args.stereo)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(node.seen.values()):
                print('Gazebo rover and required sensor topics are ready.', flush=True)
                return 0
        missing = ', '.join(topic for topic, seen in node.seen.items() if not seen)
        print('Timed out waiting for: ' + missing, flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
