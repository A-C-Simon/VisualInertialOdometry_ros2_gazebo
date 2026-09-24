#!/usr/bin/env python3
import re, serial, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
class Hw290Imu(Node):
    def __init__(self):
        super().__init__('hw290_imu'); self.declare_parameter('port','/dev/ttyUSB0'); self.declare_parameter('baud',115200); self.declare_parameter('accel_lsb_per_g',16384.0); self.declare_parameter('gyro_lsb_per_dps',131.0); self.declare_parameter('frame_id','imu')
        self.acc_scale=9.80665/float(self.get_parameter('accel_lsb_per_g').value); self.gyr_scale=3.141592653589793/180.0/float(self.get_parameter('gyro_lsb_per_dps').value); self.frame=self.get_parameter('frame_id').value; self.pub=self.create_publisher(Imu,'/imu0',50)
        self.rx=serial.Serial(self.get_parameter('port').value,int(self.get_parameter('baud').value),timeout=0.05); self.pattern=re.compile(r'MPU a/g:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\|\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)'); self.timer=self.create_timer(0.002,self.poll); self.get_logger().info('HW-290 IMU bridge started')
    def poll(self):
        while self.rx.in_waiting:
            m=self.pattern.search(self.rx.readline().decode('ascii',errors='ignore'))
            if not m: continue
            a=[int(m.group(i))*self.acc_scale for i in range(1,4)]; g=[int(m.group(i))*self.gyr_scale for i in range(4,7)]; msg=Imu(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id=self.frame; msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=a; msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=g; msg.orientation_covariance[0]=-1.0; msg.linear_acceleration_covariance[0]=0.02**2; msg.angular_velocity_covariance[0]=0.005**2; self.pub.publish(msg)
    def destroy_node(self):
        try: self.rx.close()
        except Exception: pass
        super().destroy_node()
def main():
    rclpy.init(); node=Hw290Imu()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
