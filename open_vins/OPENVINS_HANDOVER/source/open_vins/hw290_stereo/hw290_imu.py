#!/usr/bin/env python3
"""Publish SI measurements using acquisition timestamps from HW290 firmware."""
import math
import time
import rclpy
import serial
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from imu_protocol import DeviceClock, parse_sample


class Hw290Imu(Node):
    def __init__(self):
        super().__init__('hw290_imu')
        for name, value in (('port', '/dev/ttyUSB0'), ('baud', 115200), ('frame_id', 'imu'), ('raw_log_path', '')):
            self.declare_parameter(name, value)
        self.frame = self.get_parameter('frame_id').value
        self.baud = int(self.get_parameter('baud').value)
        self.pub = self.create_publisher(Imu, '/imu0', 100)
        self.rx = serial.Serial(self.get_parameter('port').value, self.baud, timeout=0, exclusive=True)
        self.buffer = bytearray()
        self.clock = DeviceClock()
        self.realtime_offset = self.get_clock().now().nanoseconds - time.monotonic_ns()
        raw_path = self.get_parameter('raw_log_path').value
        self.raw_log = open(raw_path, 'w') if raw_path else None
        if self.raw_log:
            self.raw_log.write('# host_receipt_unix_ns serial_record (includes sequence, MCU clock and raw temperature)\n')
        self.bad = self.saturated = self.published = 0
        self.started = time.monotonic()
        self.last_valid = None
        self.previous_device_us = None
        self.rate_start = None
        self.rate_device_start = self.rate_count = 0
        self.rate_ready = False
        self.slow_delivery_windows = 0
        self.timer = self.create_timer(0.002, self.poll)
        self.create_timer(5.0, self.diagnostics)
        self.get_logger().info('Awaiting IMU1/IMU2/IMU3 records; one-second device-clock warmup')

    def poll(self):
        chunk = self.rx.read(min(self.rx.in_waiting, 8192))
        if not chunk:
            return
        receipt = time.monotonic_ns()
        self.buffer.extend(chunk)
        if len(self.buffer) > 16384:
            raise RuntimeError('serial buffer overflow; restart sensor pipeline')
        while b'\n' in self.buffer:
            line, _, remaining = self.buffer.partition(b'\n')
            self.buffer = bytearray(remaining)
            if self.raw_log:
                self.raw_log.write(f'{receipt+self.realtime_offset} {line.decode("ascii", errors="replace")}\n')
            if not line.startswith((b'IMU1,', b'IMU2,', b'IMU3,')):
                if line.startswith(b'MPU a/g:'):
                    raise RuntimeError('Legacy firmware: flash firmware/hw290_openvins for acquisition timestamps')
                self.get_logger().info('Firmware: ' + line.decode('ascii', errors='replace'))
                if b'ERROR IMU' in line:
                    raise RuntimeError('IMU firmware reported failure: ' + line.decode('ascii', errors='replace'))
                continue
            try:
                sample = parse_sample(bytes(line))
            except (ValueError, UnicodeError):
                self.bad += 1
                continue
            if self.previous_device_us is not None:
                dt = (sample.micros - self.previous_device_us) & 0xffffffff
                if 50000 < dt < 2000000:
                    raise RuntimeError('IMU unhealthy: MCU sample gap >50 ms; source rate too low for VIO')
            self.previous_device_us = sample.micros
            if self.rate_start is None:
                self.rate_start, self.rate_device_start, self.rate_count = receipt, sample.micros, 0
            self.rate_count += 1
            if receipt - self.rate_start >= 1_000_000_000:
                host_rate = (self.rate_count-1)*1e9/(receipt-self.rate_start)
                device_elapsed = (sample.micros-self.rate_device_start) & 0xffffffff
                source_rate = (self.rate_count-1)*1e6/device_elapsed if device_elapsed else 0
                source_bad = not (80 <= source_rate <= 120)
                host_bad = not (80 <= host_rate <= 120)
                if source_bad or (not self.rate_ready and host_bad):
                    raise RuntimeError(f'IMU unhealthy: host/source rate outside 80-120 Hz ({host_rate:.1f}/{source_rate:.1f})')
                self.slow_delivery_windows = self.slow_delivery_windows+1 if host_rate < 80 else 0
                if self.slow_delivery_windows >= 3:
                    raise RuntimeError('IMU unhealthy: delivery below 80 Hz for three consecutive windows')
                if self.rate_ready and host_bad:
                    self.get_logger().warning(f'IMU delivery jitter: host {host_rate:.1f} Hz, verified source {source_rate:.1f} Hz')
                if not self.rate_ready:
                    self.get_logger().info(f'IMU_READY: verified host/source rate {host_rate:.1f}/{source_rate:.1f} Hz')
                    self.rate_ready = True
                self.rate_start, self.rate_device_start, self.rate_count = receipt, sample.micros, 1
            observed = receipt - int((len(line) + 1 + len(remaining)) * 10e9 / self.baud)
            stamp = self.clock.update(sample, observed)
            self.last_valid = time.monotonic()
            if stamp is None:
                continue
            if any(abs(v) >= 32760 for v in sample.accel + sample.gyro):
                self.saturated += 1
                self.get_logger().error('IMU range saturated; measurement rejected')
                continue
            msg = Imu()
            ns = stamp + self.realtime_offset
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            msg.header.frame_id = self.frame
            a = [v * 9.80665 / sample.accel_lsb_per_g for v in sample.accel]
            g = [v * math.pi / (180.0 * sample.gyro_lsb_per_dps) for v in sample.gyro]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
            msg.orientation_covariance[0] = -1.0
            # Unknown per-sample covariance: OpenVINS uses calibrated noise YAML.
            self.pub.publish(msg)
            self.published += 1

    def diagnostics(self):
        if self.last_valid is not None and time.monotonic() - self.last_valid > 3:
            raise RuntimeError('IMU stream stopped for over 3 seconds; check power and I2C wiring')
        if not self.published and time.monotonic() - self.started > 5:
            self.get_logger().error('No timestamped IMU output; check firmware and wiring')
        if not self.published and time.monotonic() - self.started > 12:
            raise RuntimeError('IMU startup timed out; restart after checking firmware diagnostics')
        self.get_logger().info(f'published={self.published} corrupt={self.bad} '
                               f'sequence_gaps={self.clock.dropped} saturated={self.saturated}')

    def destroy_node(self):
        if self.raw_log:
            self.raw_log.close()
        self.rx.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Hw290Imu()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
