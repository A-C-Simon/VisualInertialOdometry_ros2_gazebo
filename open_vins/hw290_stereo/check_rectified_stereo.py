#!/usr/bin/env python3
"""Check vertical epipolar error in one pair of rectified stereo images."""
import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CheckStereo(Node):
    def __init__(self, save_left=None):
        super().__init__('check_rectified_stereo')
        self.save_left = save_left
        self.bridge = CvBridge()
        self.frames = {}
        self.result = None
        self.create_subscription(Image, '/cam0/image_raw',
                                 lambda msg: self.receive(0, msg), qos_profile_sensor_data)
        self.create_subscription(Image, '/cam1/image_raw',
                                 lambda msg: self.receive(1, msg), qos_profile_sensor_data)

    def receive(self, camera, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.frames[(camera, stamp)] = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='mono8')
        other = self.frames.get((1 - camera, stamp))
        if other is None:
            if len(self.frames) > 20:
                self.frames.pop(next(iter(self.frames)))
            return
        left = self.frames[(0, stamp)]
        right = self.frames[(1, stamp)]
        if self.save_left:
            cv2.imwrite(self.save_left, left)
        detector = cv2.ORB_create(nfeatures=1500, fastThreshold=10)
        k0, d0 = detector.detectAndCompute(left, None)
        k1, d1 = detector.detectAndCompute(right, None)
        if d0 is None or d1 is None:
            self.result = 'No ORB features detected in both images'
            return
        matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
        good = [pair[0] for pair in matches if len(pair) == 2 and
                pair[0].distance < 0.7 * pair[1].distance]
        dy, disparities = [], []
        for match in good:
            x0, y0 = k0[match.queryIdx].pt
            x1, y1 = k1[match.trainIdx].pt
            if x0 > x1 and x0 - x1 < 200:
                dy.append(abs(y0 - y1))
                disparities.append(x0 - x1)
        if not dy:
            self.result = f'No positive-disparity matches among {len(good)} descriptor pairs'
            return
        self.result = (f'{len(dy)} positive-disparity matches; '
                       f'median |dy|={np.median(dy):.2f}px; '
                       f'90th percentile |dy|={np.percentile(dy, 90):.2f}px; '
                       f'median disparity={np.median(disparities):.2f}px; '
                       f'left brightness={left.mean():.0f}/255; '
                       f'left Laplacian variance={cv2.Laplacian(left, cv2.CV_64F).var():.1f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-left')
    args = parser.parse_args()
    rclpy.init()
    node = CheckStereo(args.save_left)
    start = time.monotonic()
    while node.result is None and time.monotonic() - start < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(node.result or 'No synchronized stereo pair received within 12 seconds')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
