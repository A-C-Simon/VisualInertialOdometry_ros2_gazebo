#!/usr/bin/env python3
"""One-time global->world alignment so RViz shows truth and VIO overlaid.

Gazebo model truth lives in the `world` frame, OpenVINS in its own `global`
frame. VIO global yaw is unobservable, so the two frames differ by a fixed
yaw and translation.
This node measures that offset from the first meters both travel and
then publishes it as a latched static transform. Residual separation
after that is real estimator drift, which is exactly what you want to
see when judging VIO competence.

Method: match VIO poses and Gazebo odometry by their message timestamps
and use the measured pose orientations to determine the yaw offset. The
translation is computed for every matched pose and the median is used.
This avoids the old displacement-window method, which became wrong when
VIO scale or heading drifted during the 3 to 5 metre window. The resulting
transform is only a rigid frame alignment. Any later separation is estimator
error, not hidden by the alignment.
Publishes once via a static broadcaster, logs it, and exits.
If VINS never shows up (e.g. --no-vins) it publishes identity after
timeout_s so RViz still has a complete TF tree.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Aligner(Node):
    def __init__(self):
        super().__init__('align_frames')
        self.declare_parameter('alignment_duration_s', 3.0)
        self.declare_parameter('max_time_delta_s', 0.08)
        self.declare_parameter('timeout_s', 180.0)
        self.odom_samples = []
        self.vio_samples = []
        self.first_vio_stamp = None
        self.done = False
        self.t_start = self.get_clock().now()
        self.br = StaticTransformBroadcaster(self)
        # Provisional identity so the TF tree (and RViz) is complete from
        # second zero; replaced by the measured alignment once available.
        self.publish_tf(0.0, 0.0, 0.0, 0.0, 'provisional identity until measured')
        self.create_subscription(PoseStamped, '/ov_msckf/posegt', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/ov_msckf/poseimu', self.on_vio, 20)
        self.create_timer(0.5, self.tick)

    def on_odom(self, msg):
        p = msg.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_samples.append((stamp, p.x, p.y, p.z,
                                  yaw_from_quaternion(msg.pose.orientation)))
        self.odom_samples = self.odom_samples[-500:]

    def on_vio(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_vio_stamp is None:
            self.first_vio_stamp = stamp
        if stamp - self.first_vio_stamp > self.get_parameter('alignment_duration_s').value:
            return
        if not self.odom_samples:
            return
        odom = min(self.odom_samples, key=lambda x: abs(x[0] - stamp))
        if abs(odom[0] - stamp) > self.get_parameter('max_time_delta_s').value:
            return
        self.vio_samples.append((stamp, p.x, p.y, p.z,
                                 yaw_from_quaternion(msg.pose.pose.orientation), odom))

    def publish_tf(self, tx, ty, tz, yaw, why):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = 'world'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(t)
        self.get_logger().info('global->world: t=(%.2f,%.2f,%.2f) yaw=%.1fdeg (%s)'
                               % (tx, ty, tz, math.degrees(yaw), why))

    def tick(self):
        if self.done:
            return
        now = self.get_clock().now()
        if (now - self.t_start).nanoseconds / 1e9 > self.get_parameter('timeout_s').value:
            self.publish_tf(0.0, 0.0, 0.0, 0.0, 'timeout fallback, VINS unseen')
            self.finish()
            return
        if len(self.vio_samples) < 20:
            return
        yaw_diffs = [norm_angle(v[4] - v[5][4]) for v in self.vio_samples]
        yaw_off = math.atan2(sum(math.sin(x) for x in yaw_diffs),
                             sum(math.cos(x) for x in yaw_diffs))
        c, s = math.cos(yaw_off), math.sin(yaw_off)
        txs, tys, tzs = [], [], []
        for v in self.vio_samples:
            o = v[5]
            txs.append(v[1] - (c * o[1] - s * o[2]))
            tys.append(v[2] - (s * o[1] + c * o[2]))
            tzs.append(v[3] - o[3])
        tx = sorted(txs)[len(txs) // 2]
        ty = sorted(tys)[len(tys) // 2]
        tz = sorted(tzs)[len(tzs) // 2]
        self.publish_tf(tx, ty, tz, yaw_off, 'initial %.0f-%.0fs' % (
            0.0, self.get_parameter('alignment_duration_s').value))
        self.finish()

    def finish(self):
        self.done = True
        # latched static transform stays alive; give it a moment on the wire
        import time
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(Aligner())
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
