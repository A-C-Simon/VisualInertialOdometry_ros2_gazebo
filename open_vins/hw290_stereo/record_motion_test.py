#!/usr/bin/env python3
"""Record online IMU poses for a manually measured out-and-back test.
This produces displacement/return statistics, not time-resolved ground-truth ATE.
"""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, Imu

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--seconds',type=float,default=45);p.add_argument('--cue',type=float,default=8)
p.add_argument('--distance-cm',type=float,default=20)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();node=Node('measured_motion_recorder');bridge=CvBridge();poses=[];imu=[]
start=time.monotonic();image_saved=False
stream=(a.output/'online.txt').open('w');stream.write('# timestamp x y z qx qy qz qw\n')
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def pose(m):
 v=m.pose.pose;row=[stamp(m),v.position.x,v.position.y,v.position.z,v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w]
 poses.append([time.monotonic()-start]+row);stream.write(' '.join(f'{x:.9f}' for x in row)+'\n');stream.flush()
def inertial(m):
 imu.append([stamp(m),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
def camera(m):
 global image_saved
 if not image_saved:
  cv2.imwrite(str(a.output/'scene.jpg'),bridge.imgmsg_to_cv2(m,desired_encoding='bgr8'));image_saved=True
node.create_subscription(PoseWithCovarianceStamped,'/ov_msckf/poseimu',pose,10)
node.create_subscription(Imu,'/imu0',inertial,qos_profile_sensor_data)
node.create_subscription(Image,'/cam0/image_raw',camera,qos_profile_sensor_data)
print('Recording stationary start; wait for movement cue.',flush=True);announced=False
try:
 while rclpy.ok() and time.monotonic()-start<a.seconds:
  rclpy.spin_once(node,timeout_sec=.02)
  if not announced and time.monotonic()-start>a.cue:
   if len(poses) < 10:
    raise RuntimeError('No initialized trajectory; movement test aborted before cue')
   print(f'MOVE NOW: {a.distance_cm:g} cm out, hold 5 seconds, return, then stay still. poses={len(poses)}',flush=True)
   announced=True
finally:
 stream.close();node.destroy_node()
 if rclpy.ok(): rclpy.shutdown()
np.savez(a.output/'capture.npz',poses=np.asarray(poses),imu=np.asarray(imu))
result={'poses':len(poses),'imu_samples':len(imu),'reference':f'user-measured {a.distance_cm/100:g} m out-and-back; no timed ground truth'}
if len(poses)>10:
 x=np.asarray(poses);pre=x[x[:,0]<a.cue];post=x[x[:,0]>a.seconds-5]
 if len(pre) and len(post):
  initial=np.median(pre[:,2:5],axis=0);final=np.median(post[:,2:5],axis=0)
  displacement=np.linalg.norm(x[:,2:5]-initial,axis=1)
  result.update(max_displacement_m=float(displacement.max()),return_error_m=float(np.linalg.norm(final-initial)),
                initial_position_m=initial.tolist(),final_position_m=final.tolist(),
                initial_jitter_m=float(np.max(np.linalg.norm(pre[:,2:5]-initial,axis=1))),
                max_pose_gap_s=float(np.diff(x[:,1]).max()))
(a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
