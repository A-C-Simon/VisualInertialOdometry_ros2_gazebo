#!/usr/bin/env python3
"""Emit static_transform_publisher arguments from the SAME YAML used by VIO."""
import argparse
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('camera',choices=['cam0','cam1','time_offset','orb_imu']);a=p.parse_args()
config=Path(__file__).with_name('kalibr_imucam_chain.yaml')
x=yaml.safe_load(config.read_text().replace('%YAML:1.0',''))
if a.camera == 'time_offset':
 print(float(x['cam0']['timeshift_cam_imu']))
 raise SystemExit(0)
parent,child='imu',a.camera
if a.camera == 'orb_imu':
 # ORB publishes map -> camera_optical_frame. Join the real sensor TF tree
 # using the inverse rigid calibration, without inventing a world pose.
 t=np.linalg.inv(np.asarray(x['cam0']['T_imu_cam']))
 parent,child='camera_optical_frame','imu'
else:
 t=np.asarray(x[a.camera]['T_imu_cam'])
q=Rotation.from_matrix(t[:3,:3]).as_quat()
for name,value in zip(['x','y','z','qx','qy','qz','qw'],list(t[:3,3])+list(q)):
 print('--'+name);print(float(value))
print('--frame-id');print(parent);print('--child-frame-id');print(child)
