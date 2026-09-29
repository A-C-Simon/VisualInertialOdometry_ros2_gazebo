#!/usr/bin/env python3
"""Repeatable PTY/ROS test; CPU measurements concern the bridge, not the feeder."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=15);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();node=Node('hw290_bridge_test');rows=[]
def callback(m):
    rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.linear_acceleration.z,m.angular_velocity.x,m.orientation_covariance[0]])
sub=node.create_subscription(Imu,'/imu0',callback,100)
results={}

def record(i):
    version='IMU2';accel=8192;gyro=131
    # 1 g and 2 degrees/s. Cross micros wrap while publishing at 100 Hz.
    raw=f'{version},{i},{(0xffffff00+i*10000)&0xffffffff},0,0,{accel},{gyro},0,0,42'.encode()
    checksum=0
    for byte in raw: checksum ^= byte
    return raw+f'*{checksum:02X}\r\n'.encode()

try:
 for name,cmd in [('python',['/usr/bin/python3',str(ROOT/'hw290_stereo/hw290_imu.py')]),
                  ('cpp',[str(ROOT/'install_vio/ov_hw290/lib/ov_hw290/hw290_imu')])]:
    master,slave=pty.openpty();port=os.ttyname(slave)
    resources=a.output/f'{name}.resources.json';log=(a.output/f'{name}.log').open('w')
    proc=subprocess.Popen(['/usr/bin/python3',str(ROOT/'hw290_stereo/managed_process.py'),str(resources)]+cmd+['--ros-args','-p','port:='+port],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+8
        while 'Awaiting IMU1' not in (a.output/f'{name}.log').read_text():
            rclpy.spin_once(node,timeout_sec=.02)
            if proc.poll() is not None or time.monotonic()>deadline: raise RuntimeError(f'{name} startup failed')
        # Allow DDS discovery before sending the one-second warmup.
        until=time.monotonic()+1
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        rows.clear();start=time.monotonic();count=int(a.seconds*100)
        for i in range(count):
            packet=record(i)
            # Feed complete lines, partial lines, and a corrupt line preceding a valid one.
            if i==250: os.write(master, packet[:-4]+b'XX\r\n')
            if i%100==50:
                os.write(master,packet[:12]);time.sleep(.001);os.write(master,packet[12:])
            else: os.write(master,packet)
            deadline=start+(i+1)*.01
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=max(0.,min(.002,deadline-time.monotonic())))
        until=time.monotonic()+.4
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        assert len(rows)==count-100,(name,len(rows),count-100)
        assert all(abs(x[1]-9.80665)<1e-8 and abs(x[2]-math.radians(2))<1e-8 and x[3]==-1 for x in rows)
        gaps=[b[0]-a[0] for a,b in zip(rows,rows[1:])]
        assert all(.0098<d<.0102 for d in gaps),(min(gaps),max(gaps))
        received=len(rows)
        proc.send_signal(signal.SIGINT);proc.wait(timeout=14)
        result=json.loads(resources.read_text());result.update(received=received,sent=count,min_interval_s=min(gaps),max_interval_s=max(gaps))
        results[name]=result
        print(name,json.dumps(result),flush=True)
    finally:
        if proc.poll() is None:proc.send_signal(signal.SIGINT);proc.wait(timeout=14)
        os.close(master);os.close(slave);log.close()
 (a.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
