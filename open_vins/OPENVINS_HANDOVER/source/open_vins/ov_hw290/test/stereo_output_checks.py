#!/usr/bin/env python3
"""Compare native mono output with cv_bridge conversion of RGB stereo output."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

root = Path(__file__).resolve().parents[2]
exe = root / 'install_vio/ov_hw290/lib/ov_hw290/stereo_splitter'
calibration = root.parent / 'calibration/elp_3dgs1200p01/calib/calibration_opencv.yaml'
output = Path(tempfile.mkdtemp(prefix='hw290_stereo_output_'))
rclpy.init()
node = rclpy.create_node('stereo_output_check')
bridge = CvBridge()
received = {}
children = []
logs = []
subscriptions = []

def capture(mode, camera, message):
    key = (message.header.stamp.sec, message.header.stamp.nanosec)
    received.setdefault(key, {})[(mode, camera)] = message

try:
    publisher = node.create_publisher(Image, '/splitter_check/source', 5)
    for mode in ('rgb', 'mono'):
        args = [str(exe), '--ros-args', '-r', f'__node:=splitter_{mode}',
                '-r', '/image_raw:=/splitter_check/source',
                '-p', f'calibration_file:={calibration}',
                '-p', 'auto_timestamp_correction:=false',
                '-p', f'monochrome:={str(mode == "mono").lower()}']
        for camera in (0, 1):
            topic = f'/splitter_check/{mode}/cam{camera}'
            args += ['-r', f'/cam{camera}/image_raw:={topic}',
                     '-r', f'/cam{camera}/camera_info:={topic}_info']
            subscriptions.append(node.create_subscription(
                Image, topic, lambda m, md=mode, c=camera: capture(md, c, m), 5))
        log = (output / f'{mode}.log').open('w')
        logs.append(log)
        children.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
    deadline = time.monotonic() + 15
    while publisher.get_subscription_count() < 2:
        rclpy.spin_once(node, timeout_sec=.05)
        assert time.monotonic() < deadline, 'Splitter discovery timed out'
        assert all(p.poll() is None for p in children), 'Splitter exited'
    rng = np.random.default_rng(290)
    differences = []
    for index in range(6):
        image = rng.integers(0, 256, (480, 1280, 3), dtype=np.uint8)
        message = bridge.cv2_to_imgmsg(image, encoding='rgb8')
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'camera_link'
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        publisher.publish(message)
        deadline = time.monotonic() + 5
        while len(received.get(key, {})) != 4:
            rclpy.spin_once(node, timeout_sec=.02)
            assert time.monotonic() < deadline, 'Stereo output timed out'
        for camera in (0, 1):
            rgb = received[key][('rgb', camera)]
            mono = received[key][('mono', camera)]
            expected = bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            actual = bridge.imgmsg_to_cv2(mono, desired_encoding='mono8')
            delta = int(np.max(np.abs(expected.astype(np.int16)-actual.astype(np.int16))))
            assert delta == 0, (index, camera, delta)
            assert mono.header == rgb.header and mono.encoding == 'mono8'
            assert len(mono.data) * 3 == len(rgb.data)
            differences.append(delta)
        del received[key]
    result = {'stereo_pairs': 6, 'max_pixel_difference': max(differences),
              'identical_headers': True, 'image_payload_reduction_percent': 100 * 2 / 3}
    (output / 'results.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    print('Artifacts:', output, flush=True)
finally:
    for child in children:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=5)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
