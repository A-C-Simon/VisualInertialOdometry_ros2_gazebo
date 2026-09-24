#!/usr/bin/env python3
"""Estimate the rigid camera-to-IMU rotation from hand motion.

Run with camera and IMU ROS topics active. Rotate the fixed pair about all
three axes in a textured scene. This estimates rotation only. Translation and
noise still require a full camera-IMU calibration for accurate VIO.
"""
import argparse
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, Imu


class Calibrator(Node):
    def __init__(self, offset_min=-0.4, offset_max=0.4):
        super().__init__('hw290_rotation_calibrator')
        self.offset_min = offset_min
        self.offset_max = offset_max
        self.bridge = CvBridge()
        self.imu = deque(maxlen=12000)
        self.pairs = []
        self.last = None
        self.orb = cv2.ORB_create(nfeatures=900, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.k = np.array([[423.9725148, 0, 318.7361917],
                           [0, 424.0607865, 246.9560589],
                           [0, 0, 1]], dtype=np.float64)
        self.d = np.array([-0.4169720272, 0.2386866981,
                           0.0001763182, -0.0001649462, -0.0935433880])
        self.create_subscription(Imu, '/imu0', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/cam0/image_raw', self.on_image, qos_profile_sensor_data)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_imu(self, msg):
        self.imu.append((self.stamp(msg), np.array([
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z], dtype=np.float64)))

    def on_image(self, msg):
        t = self.stamp(msg)
        if self.last is not None and t - self.last[0] < 0.13:
            return
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        current = (t, keypoints, descriptors)
        old = self.last
        self.last = current
        if old is None or old[2] is None or descriptors is None:
            return
        matches = self.matcher.knnMatch(old[2], descriptors, k=2)
        good = [pair[0] for pair in matches
                if len(pair) == 2 and pair[0].distance < 0.7 * pair[1].distance]
        if len(good) < 40:
            return
        p0 = np.float32([old[1][m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        p1 = np.float32([keypoints[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        u0 = cv2.undistortPoints(p0, self.k, self.d).reshape(-1, 2)
        u1 = cv2.undistortPoints(p1, self.k, self.d).reshape(-1, 2)
        # A pure or near-pure rig rotation makes the essential matrix
        # degenerate. Fit a ray homography and project it onto SO(3).
        h, mask = cv2.findHomography(u0, u1, cv2.RANSAC, 0.003)
        if h is None or mask is None:
            return
        valid = int(np.count_nonzero(mask))
        h = h / np.cbrt(np.linalg.det(h))
        u, _, vt = np.linalg.svd(h)
        r = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
        vec = Rotation.from_matrix(r).as_rotvec()
        angle = np.linalg.norm(vec)
        if valid >= 25 and 0.015 < angle < 0.5:
            self.pairs.append((old[0], t, vec, valid))

    def integrate(self, times, gyro, start, end):
        if start < times[0] or end > times[-1]:
            return None
        interior = (times > start) & (times < end)
        t = np.concatenate(([start], times[interior], [end]))
        w = np.vstack((np.array([np.interp(start, times, gyro[:, j])
                                 for j in range(3)]), gyro[interior],
                       np.array([np.interp(end, times, gyro[:, j])
                                 for j in range(3)])))
        return -np.trapz(w, t, axis=0)

    def solve(self):
        if len(self.pairs) < 12 or len(self.imu) < 100:
            print(f'Insufficient motion: {len(self.pairs)} useful visual rotations. '
                  'Rotate the rigid pair about three axes in a textured scene.')
            return
        sample = list(self.imu)
        times = np.array([x[0] for x in sample])
        gyro = np.array([x[1] for x in sample])
        best = None
        for offset in np.arange(self.offset_min, self.offset_max + 0.001, 0.005):
            camera, inertial = [], []
            for t0, t1, vector, _ in self.pairs:
                imu_vector = self.integrate(times, gyro, t0 + offset, t1 + offset)
                if imu_vector is None or np.linalg.norm(imu_vector) < 0.01:
                    continue
                camera.append(vector)
                inertial.append(imu_vector)
            if len(camera) < 12:
                continue
            camera = np.array(camera)
            inertial = np.array(inertial)
            for sign in (1, -1):
                try:
                    fit, _ = Rotation.align_vectors(camera, sign * inertial)
                except ValueError:
                    continue
                residual = np.linalg.norm(
                    camera - fit.apply(sign * inertial), axis=1)
                score = float(np.median(residual))
                if best is None or score < best[0]:
                    best = (score, offset, sign, fit, camera, inertial)
        if best is None:
            print('No synchronized camera and IMU rotations could be paired.')
            return
        score, offset, sign, fit, camera, inertial = best
        singular = np.linalg.svd(inertial, compute_uv=False)
        print(f'Usable homography rotation pairs: {len(camera)}')
        print(f'Median rotational residual: {np.rad2deg(score):.2f} degrees')
        print(f'IMU motion axis spread: {singular[-1]/singular[0]:.3f}')
        print(f'Camera-to-IMU time shift: {offset:+.3f} seconds')
        if offset - self.offset_min < 0.005 or self.offset_max - offset < 0.005:
            print('Time shift is at the search boundary; do not use it as a calibration.')
        print(f'Gyro sign used: {sign:+d}')
        print('Estimated R_imu_to_cam:')
        print(np.array2string(fit.as_matrix(), precision=5))
        print('Estimated R_cam_to_imu for T_imu_cam:')
        print(np.array2string(fit.as_matrix().T, precision=5))
        if np.rad2deg(score) > 5 or singular[-1]/singular[0] < 0.08:
            print('Calibration quality is insufficient for an automatic VIO update.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--save', default='/tmp/hw290_rotation_capture.npz')
    parser.add_argument('--load')
    parser.add_argument('--offset-min', type=float, default=-0.4)
    parser.add_argument('--offset-max', type=float, default=0.4)
    args = parser.parse_args()
    if args.offset_min >= args.offset_max:
        parser.error('--offset-min must be below --offset-max')
    rclpy.init()
    node = Calibrator(args.offset_min, args.offset_max)
    if args.load:
        capture = np.load(args.load)
        node.pairs = [(float(t0), float(t1), vector, int(valid))
                      for t0, t1, vector, valid in zip(
                          capture['pair_t0'], capture['pair_t1'],
                          capture['pair_vec'], capture['pair_valid'])]
        node.imu.extend((float(t), vector) for t, vector in zip(
            capture['imu_time'], capture['imu_gyro']))
    else:
        start = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        if node.pairs and node.imu:
            pair_t0, pair_t1, pair_vec, pair_valid = zip(*node.pairs)
            imu_time, imu_gyro = zip(*node.imu)
            np.savez(args.save, pair_t0=pair_t0, pair_t1=pair_t1,
                     pair_vec=pair_vec, pair_valid=pair_valid,
                     imu_time=imu_time, imu_gyro=imu_gyro)
            print(f'Saved capture to {args.save}')
    node.solve()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
