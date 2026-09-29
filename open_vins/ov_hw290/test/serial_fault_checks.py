#!/usr/bin/env python3
"""Verify C++ serial startup, disconnection, saturation and clock-reset failures."""
import json
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2]
exe=ROOT/'install_vio/ov_hw290/lib/ov_hw290/hw290_imu'
out=Path(tempfile.mkdtemp(prefix='hw290_cpp_faults_'))
results={}
def packet(i,saturated=False):
    raw=f'IMU2,{i},{i*10000},0,0,{32767 if saturated else 8192},0,0,0,42'.encode()
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
(out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
print('Artifacts:',out)
