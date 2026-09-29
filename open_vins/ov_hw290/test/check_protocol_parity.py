#!/usr/bin/env python3
"""Compare the C++ protocol/clock with the retained Python implementation."""
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'hw290_stereo'))
from imu_protocol import DeviceClock, parse_sample

def packet(payload):
    payload=payload.encode(); checksum=0
    for byte in payload: checksum ^= byte
    return payload+f'*{checksum:02X}'.encode()

records=[]
for i in range(650):
    if i==350: continue
    seq=(0xffffff00+i)&0xffffffff
    us=(0xffffff00+i*10000)&0xffffffff
    version='IMU1' if i<200 else 'IMU2'
    values=f'{version},{seq},{us},0,8192,-12,131,0,-10,42'
    records.append((10**12+i*10**7+(i%4)*2000000,packet(values)))
    if i==250: records.append((10**12+i*10**7,packet(values))) # duplicate
    if i==400: records.append((10**12+i*10**7,packet(values)[:-2]+b'XX'))
records += [(10**13,packet('IMU2,1,0,0,0,0,0,0,0,0')),
            (10**13,packet('IMU2,1,1,0,32768,0,0,0,0,0'))]
clock=DeviceClock();expected=[]
for receipt, raw in records:
    try:
        s=parse_sample(raw);stamp=clock.update(s,receipt)
        expected.append(f'{stamp if stamp is not None else "warmup"} {clock.dropped} {s.accel_lsb_per_g:g} {s.gyro_lsb_per_dps:g}')
    except ValueError: expected.append('error')
actual=subprocess.check_output([sys.argv[1]],input=''.join(f'{t} {raw.decode()}\n' for t,raw in records),text=True).splitlines()
assert actual==expected, next(((i,a,b) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b), 'length mismatch')
print(f'PASS: {len(records)} C++/Python results match (counter wrap, jitter, loss, duplicate, checksum, ranges, scales and clock reset).')
