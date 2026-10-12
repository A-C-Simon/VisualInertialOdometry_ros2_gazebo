#!/usr/bin/env python3
"""Verify C++ serial startup, disconnection, saturation and clock-reset failures."""
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2]
exe=ROOT/'install_vio/ov_hw290/lib/ov_hw290/hw290_imu'
out=Path(tempfile.mkdtemp(prefix='hw290_cpp_faults_'))
results={}
def packet(i,saturated=False,micros=None):
    raw=f'IMU2,{i},{i*10000 if micros is None else micros},0,0,{32767 if saturated else 8192},0,0,0,42'.encode()
    checksum=0
    for b in raw: checksum ^= b
    return raw+f'*{checksum:02X}\n'.encode()
for mode,expected in [('startup','IMU startup timed out'),('disconnect','IMU serial disconnected'),('reset','duplicate/out-of-order sample'),('silence','IMU stream stopped')]:
    master,slave=pty.openpty();logpath=out/f'{mode}.log'
    with logpath.open('w') as log:
        proc=subprocess.Popen([str(exe),'--ros-args','-p','port:='+os.ttyname(slave)],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+5
            while 'Awaiting IMU1' not in logpath.read_text():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Startup failed')
                time.sleep(.01)
            # An independent opener must fail while the first process owns the serial device.
            if mode=='startup':
                duplicate=subprocess.run([str(exe),'--ros-args','-p','port:='+os.ttyname(slave)],capture_output=True,text=True,timeout=5)
                assert duplicate.returncode!=0 and 'already in use' in duplicate.stderr
            else:
                for i in range(150):
                    os.write(master,packet(i,saturated=(i==120)));time.sleep(.01)
                if mode=='disconnect':os.close(master);master=None
                if mode=='reset':os.write(master,packet(0))
            rc=proc.wait(timeout=14)
            text=logpath.read_text()
            assert rc!=0 and expected in text,(mode,rc,text)
            if mode!='startup':assert 'IMU range saturated; measurement rejected' in text
            results[mode]='passed';print(mode,'PASS',flush=True)
        finally:
            if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
            if master is not None:os.close(master)
            os.close(slave)
for mode,period,step,expected in [
        ('low_source_rate',.07,70000,'MCU sample gap'),
        ('incorrect_acquisition_clock',.01,20000,'host/source rate outside'),
        ('slow_delivery',.02,10000,'host/source rate outside'),
        ('midrun_rate_collapse',.01,10000,'MCU sample gap')]:
    master,slave=pty.openpty();logpath=out/f'{mode}.log'
    with logpath.open('w') as log:
        proc=subprocess.Popen([str(exe),'--ros-args','-p','port:='+os.ttyname(slave)],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+5
            while 'Awaiting IMU1' not in logpath.read_text():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Startup failed')
                time.sleep(.01)
            start=time.monotonic()
            for i in range(160):
                if proc.poll() is not None:break
                timestamp=i*step+(60000 if mode=='midrun_rate_collapse' and i>=150 else 0)
                os.write(master,packet(i,micros=timestamp))
                time.sleep(max(0.,start+(i+1)*period-time.monotonic()))
            rc=proc.wait(timeout=5);contents=logpath.read_text()
            assert rc!=0 and expected in contents,(mode,rc,contents)
            if mode=='midrun_rate_collapse':assert 'IMU_READY: verified' in contents
            else:assert 'IMU_READY: verified' not in contents
            results[mode]='passed';print(mode,'PASS',flush=True)
        finally:
            if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
            os.close(master);os.close(slave)
master,slave=pty.openpty();logpath=out/'host_catchup.log'
with logpath.open('w') as log:
    proc=subprocess.Popen([str(exe),'--ros-args','-p','port:='+os.ttyname(slave)],stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+5
        while 'Awaiting IMU1' not in logpath.read_text():
            if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Startup failed')
            time.sleep(.01)
        start=time.monotonic()
        for i in range(340):
            if 150<i<=210:continue
            if i==150:
                time.sleep(.6)
                os.write(master,b''.join(packet(j) for j in range(150,211)))
            else:os.write(master,packet(i))
            time.sleep(max(0.,start+(i+1)*.01-time.monotonic()))
        assert proc.poll() is None,logpath.read_text()
        contents=logpath.read_text()
        assert 'IMU_READY: verified' in contents and 'IMU delivery jitter' in contents,contents
        proc.send_signal(signal.SIGINT);assert proc.wait(timeout=5)==0
        results['host_catchup']='passed';print('host_catchup PASS',flush=True)
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
        os.close(master);os.close(slave)
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
print('Artifacts:',out)
