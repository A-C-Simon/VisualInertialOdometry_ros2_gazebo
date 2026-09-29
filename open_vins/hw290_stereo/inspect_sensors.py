#!/usr/bin/env python3
"""Record sensor timing and stationary statistics; no estimator or bias subtraction."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

p=argparse.ArgumentParser()
p.add_argument('--seconds',type=float,default=20)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
rclpy.init(); node=Node('hw290_sensor_inspector')
imu=[]; camera=[]
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def on_imu(m):
    imu.append([stamp(m),node.get_clock().now().nanoseconds*1e-9,
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,
        m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
def on_image(m): camera.append([stamp(m),node.get_clock().now().nanoseconds*1e-9])
node.create_subscription(Imu,'/imu0',on_imu,qos_profile_sensor_data)
node.create_subscription(Image,'/cam0/image_raw',on_image,qos_profile_sensor_data)
start=time.monotonic()
try:
    while rclpy.ok() and time.monotonic()-start<a.seconds: rclpy.spin_once(node,timeout_sec=.05)
finally:
    node.destroy_node();rclpy.shutdown()
a.output.parent.mkdir(parents=True,exist_ok=True)
np.savez(str(a.output)+'.npz',imu=np.asarray(imu),camera=np.asarray(camera))
result={}
for name,rows in [('imu',imu),('camera',camera)]:
    x=np.asarray(rows)
    result[name]={'samples':len(rows)}
    if len(x)>2:
        dt=np.diff(x[:,0]);age=x[:,1]-x[:,0]
        result[name].update(rate_hz=float(1/np.median(dt)),
            dt_ms_percentiles=np.percentile(dt*1000,[0,5,50,95,100]).tolist(),
            age_ms_percentiles=np.percentile(age*1000,[0,5,50,95,100]).tolist(),
            nonpositive_intervals=int(np.count_nonzero(dt<=0)))
        if name=='imu':
            x=x[min(100,len(x)//4):]
            result[name].update(mean_accel=x[:,2:5].mean(0).tolist(),std_accel=x[:,2:5].std(0).tolist(),
                mean_gyro=x[:,5:8].mean(0).tolist(),std_gyro=x[:,5:8].std(0).tolist(),
                gravity_norm=float(np.linalg.norm(x[:,2:5].mean(0))))
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
