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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, Imu

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--seconds',type=float,default=45);p.add_argument('--cue',type=float,default=8)
p.add_argument('--distance-cm',type=float,default=20)
p.add_argument('--method',choices=['openvins','orb'],default='openvins')
p.add_argument('--pose-topic',help='Override the selected estimator online IMU pose topic')
mode=p.add_mutually_exclusive_group()
mode.add_argument('--stationary',action='store_true',help='Record rest without a movement cue')
mode.add_argument('--continuous-motion',action='store_true',help='Cue sustained movement instead of a single displacement')
p.add_argument('--motion-seconds',type=float,default=90)
p.add_argument('--desk-size-cm',type=float,nargs=3,metavar=('LENGTH','WIDTH','HEIGHT'),help='Physical movement envelope; flag estimates beyond its diagonal')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
if a.desk_size_cm and any(v<=0 for v in a.desk_size_cm):p.error('Desk dimensions must be positive')
desk_bound=float(np.linalg.norm(np.asarray(a.desk_size_cm)/100)) if a.desk_size_cm else None
rclpy.init();node=Node('measured_motion_recorder');bridge=CvBridge();poses=[];imu=[]
start=time.monotonic();image_saved=False;watch_origin=None;bound_exceeded_live=False
stream=(a.output/'online.txt').open('w');stream.write('# timestamp x y z qx qy qz qw\n')
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def pose(m):
 global watch_origin,bound_exceeded_live
 v=m.pose.pose if a.method=='openvins' else m.pose
 row=[stamp(m),v.position.x,v.position.y,v.position.z,v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w]
 poses.append([time.monotonic()-start]+row);stream.write(' '.join(f'{x:.9f}' for x in row)+'\n');stream.flush()
 if desk_bound is not None and len(poses)>=10:
  if watch_origin is None:watch_origin=np.median(np.asarray(poses[:10])[:,2:5],axis=0)
  distance=float(np.linalg.norm(np.asarray(row[1:4])-watch_origin))
  if distance>desk_bound and not bound_exceeded_live:
   print(f'BOUND EXCEEDED: estimated {distance:.3f} m exceeds desk diagonal {desk_bound:.3f} m. Recording continues without resetting VIO.',flush=True)
   bound_exceeded_live=True
def inertial(m):
 imu.append([stamp(m),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
def camera(m):
 global image_saved
 if not image_saved:
  cv2.imwrite(str(a.output/'scene.jpg'),bridge.imgmsg_to_cv2(m,desired_encoding='bgr8'));image_saved=True
pose_type=PoseWithCovarianceStamped if a.method=='openvins' else PoseStamped
pose_topic=a.pose_topic or ('/ov_msckf/poseimu' if a.method=='openvins' else '/orbslam_vio/pose_imu')
node.create_subscription(pose_type,pose_topic,pose,10)
node.create_subscription(Imu,'/imu0',inertial,qos_profile_sensor_data)
node.create_subscription(Image,'/cam0/image_raw',camera,qos_profile_sensor_data)
print('Recording stationary check.' if a.stationary else 'Recording stationary start; wait for movement cue.',flush=True);announced=False;return_announced=False
capture_error=None;interrupted=False
try:
 while rclpy.ok() and time.monotonic()-start<a.seconds:
  rclpy.spin_once(node,timeout_sec=.02)
  if not a.stationary and not announced and time.monotonic()-start>a.cue:
   if len(poses) < 10:
    raise RuntimeError('No initialized trajectory; movement test aborted before cue')
   instruction=(f'keep moving gently for {a.motion_seconds:g} seconds, then return to the starting mark and rest.'
                if a.continuous_motion else f'{a.distance_cm:g} cm out, hold 5 seconds, return, then stay still.')
   print(f'MOVE NOW: {instruction} poses={len(poses)}',flush=True)
   announced=True
  if a.continuous_motion and not return_announced and time.monotonic()-start>a.cue+a.motion_seconds:
   print('RETURN NOW: return to the starting mark and stay still.',flush=True)
   return_announced=True
except KeyboardInterrupt:
 interrupted=True
except Exception as error:
 capture_error=error
finally:
 stream.close();node.destroy_node()
 if rclpy.ok(): rclpy.shutdown()
 # Retain received data even when initialization or the movement cue fails.
 np.savez(a.output/'capture.npz',poses=np.asarray(poses),imu=np.asarray(imu))
reference=('stationary check; no timed ground truth' if a.stationary else
           f'{a.motion_seconds:g} seconds of continuous manual motion and return; no timed ground truth' if a.continuous_motion else
           f'user-measured {a.distance_cm/100:g} m out-and-back; no timed ground truth')
result={'method':a.method,'pose_topic':pose_topic,'poses':len(poses),'imu_samples':len(imu),'reference':reference,
        'capture_status':'interrupted' if interrupted else 'failed' if capture_error else 'completed' if time.monotonic()-start>=a.seconds else 'shutdown',
        'movement_cue_sent':announced,'return_cue_sent':return_announced}
if capture_error:result['capture_error']=str(capture_error)
if desk_bound is not None:
 result.update(desk_size_cm=a.desk_size_cm,desk_diagonal_bound_m=desk_bound,
               reference='User movement within the stated desk volume; no timed ground truth')
if len(poses)>10:
 x=np.asarray(poses);pre=x[x[:,0]<a.cue];post=x[x[:,0]>=x[-1,0]-5]
 result.update(last_pose_elapsed_s=float(x[-1,0]),pose_stream_ended_early=bool(x[-1,0]<a.seconds-5))
 if len(pre) and len(post):
  initial=np.median(pre[:,2:5],axis=0);final=np.median(post[:,2:5],axis=0)
  displacement=np.linalg.norm(x[:,2:5]-initial,axis=1)
  result.update(max_displacement_m=float(displacement.max()),
                initial_position_m=initial.tolist(),final_position_m=final.tolist(),
                initial_jitter_m=float(np.max(np.linalg.norm(pre[:,2:5]-initial,axis=1))),
                max_pose_gap_s=float(np.diff(x[:,1]).max()))
  result['final_displacement_m' if a.stationary or a.continuous_motion else 'return_error_m']=float(np.linalg.norm(final-initial))
  if desk_bound is not None:result['exceeded_geometry_bound']=bool(displacement.max()>desk_bound)
(a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
if capture_error:raise capture_error
if interrupted:raise SystemExit(130)
