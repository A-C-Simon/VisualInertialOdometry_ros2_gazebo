#!/usr/bin/env python3
"""Replay an EuRoC ASL-format sequence into ROS 2 and record OpenVINS poses."""

import argparse
import csv
import signal
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu


def read_rows(path: Path):
    with path.open(newline="") as stream:
        return [row for row in csv.reader(line for line in stream if not line.startswith("#")) if row]


def stamp_message(message, timestamp_ns: int, frame_id: str):
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
    message.header.frame_id = frame_id


class EurocPlayer(Node):
    def __init__(self, trajectory_path: Path):
        super().__init__("euroc_player")
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.cam0 = self.create_publisher(Image, "/cam0/image_raw", reliable)
        self.cam1 = self.create_publisher(Image, "/cam1/image_raw", reliable)
        self.imu = self.create_publisher(Imu, "/imu0", sensor)
        self.create_subscription(
            PoseWithCovarianceStamped, "/ov_msckf/poseimu", self.pose_callback, reliable
        )
        trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        self.trajectory = trajectory_path.open("w")
        self.trajectory.write("# timestamp x y z q_x q_y q_z q_w\n")
        self.bridge = CvBridge()
        self.pose_count = 0

    def pose_callback(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = message.pose.pose
        self.trajectory.write(
            f"{stamp:.9f} {pose.position.x:.9f} {pose.position.y:.9f} "
            f"{pose.position.z:.9f} {pose.orientation.x:.9f} "
            f"{pose.orientation.y:.9f} {pose.orientation.z:.9f} "
            f"{pose.orientation.w:.9f}\n"
        )
        self.trajectory.flush()
        self.pose_count += 1

    def publish_imu(self, row):
        message = Imu()
        stamp_message(message, int(row[0]), "imu0")
        message.angular_velocity.x = float(row[1])
        message.angular_velocity.y = float(row[2])
        message.angular_velocity.z = float(row[3])
        message.linear_acceleration.x = float(row[4])
        message.linear_acceleration.y = float(row[5])
        message.linear_acceleration.z = float(row[6])
        self.imu.publish(message)

    def publish_stereo(self, timestamp_ns: int, left_path: Path, right_path: Path):
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise RuntimeError(f"Failed to read stereo pair at {timestamp_ns}")
        left_message = self.bridge.cv2_to_imgmsg(left, encoding="mono8")
        right_message = self.bridge.cv2_to_imgmsg(right, encoding="mono8")
        stamp_message(left_message, timestamp_ns, "cam0")
        stamp_message(right_message, timestamp_ns, "cam1")
        self.cam0.publish(left_message)
        self.cam1.publish(right_message)

    def close(self):
        self.trajectory.close()


def wait_until(node: Node, deadline: float):
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.005, deadline - time.monotonic()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path, help="EuRoC sequence directory containing mav0")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    parser.add_argument("--trajectory", type=Path, default=Path("/tmp/openvins_euroc.txt"))
    parser.add_argument("--startup-delay", type=float, default=2.0)
    parser.add_argument("--post-roll", type=float, default=2.0)
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be positive")

    mav0 = args.dataset / "mav0"
    imu_rows = read_rows(mav0 / "imu0" / "data.csv")
    left_rows = read_rows(mav0 / "cam0" / "data.csv")
    right_files = {int(row[0]): row[1] for row in read_rows(mav0 / "cam1" / "data.csv")}
    stereo_rows = [row for row in left_rows if int(row[0]) in right_files]
    if not imu_rows or not stereo_rows:
        raise RuntimeError("Dataset has no IMU data or synchronized stereo pairs")

    last_stereo_stamp = int(stereo_rows[-1][0])
    # Some small test fixtures retain the full IMU stream but only a short
    # image excerpt. Stop with the image data instead of replaying IMU alone.
    imu_rows = [row for row in imu_rows if int(row[0]) <= last_stereo_stamp + 100_000_000]
    events = [(int(row[0]), 0, row) for row in imu_rows]
    events.extend((int(row[0]), 1, row) for row in stereo_rows)
    events.sort(key=lambda event: (event[0], event[1]))

    rclpy.init()
    node = EurocPlayer(args.trajectory)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    try:
        wait_until(node, time.monotonic() + args.startup_delay)
        first_stamp = events[0][0]
        start = time.monotonic()
        imu_count = 0
        stereo_count = 0
        for timestamp_ns, kind, row in events:
            if not rclpy.ok():
                break
            target = start + (timestamp_ns - first_stamp) * 1e-9 / args.speed
            wait_until(node, target)
            if kind == 0:
                node.publish_imu(row)
                imu_count += 1
            else:
                left_path = mav0 / "cam0" / "data" / row[1]
                right_path = mav0 / "cam1" / "data" / right_files[timestamp_ns]
                node.publish_stereo(timestamp_ns, left_path, right_path)
                stereo_count += 1
            rclpy.spin_once(node, timeout_sec=0)
        wait_until(node, time.monotonic() + args.post_roll)
        print(
            f"Published {imu_count} IMU samples and {stereo_count} stereo pairs; "
            f"recorded {node.pose_count} OpenVINS poses"
        )
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
