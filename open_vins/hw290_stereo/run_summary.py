#!/usr/bin/env python3
"""Summarize per-run process accounting and current-run estimator logs."""
import json
import csv
import math
from pathlib import Path
import re
import statistics
import sys

root = Path(sys.argv[1])
rows = []
print('\n================ run summary ================')
print('CPU: accumulated user+system; 100% means one fully occupied CPU core.')
print(f'{"Process":18s} {"CPU s":>9s} {"Avg CPU %":>10s} {"Peak MiB":>10s}')
for path in sorted(root.glob('*.resources.json')):
    row = json.loads(path.read_text()); row['role'] = path.name.split('.')[0]
    rows.append(row)
    if 'cpu_seconds' in row:
        print(f'{row["role"]:18s} {row["cpu_seconds"]:9.2f} {row["cpu_percent_one_core"]:10.1f} {row["peak_rss_mib"]:10.1f}')
    else: print(row)
    if row.get('forced_shutdown'): print('  Shutdown required escalation; inspect this process log.')
started = float((root/'started.txt').read_text())
import time
wall = time.time()-started
cpu = sum(x.get('cpu_seconds',0) for x in rows)
print(f'Measured child CPU total: {cpu:.2f} s; launch wall time: {wall:.2f} s; average {100*cpu/wall:.1f}% of one core')
print('Per-process peak memory is not simultaneous total memory. Includes viewers when enabled.')
log = (root/'estimator.log').read_text(errors='replace') if (root/'estimator.log').exists() else ''
log = re.sub(r'\x1b\[[0-9;]*m', '', log)
timing = [float(v)*1000 for v in re.findall(r'\[TIME\]:\s*([0-9.eE+\-]+) seconds (?:for )?total', log)]
timing_source = 'estimator debug log'
timing_stamps = []
timing_csv = root/'estimator_timing.csv'
if timing_csv.exists():
    csv_timing = []
    with timing_csv.open() as stream:
        for fields in csv.reader(line for line in stream if line.strip() and not line.startswith('#')):
            try:
                stamp, seconds = float(fields[0]), float(fields[-1])
            except (ValueError, IndexError):
                continue
            if math.isfinite(stamp) and math.isfinite(seconds) and seconds >= 0:
                timing_stamps.append(stamp)
                csv_timing.append(seconds*1000)
    timing = csv_timing
    timing_source = 'estimator stage CSV'
orb_timing = root/'vio_timing.txt'
if orb_timing.exists():
    timing = [float(line.split()[-1]) for line in orb_timing.read_text().splitlines() if line.strip() and not line.startswith('#')]
    timing_source = 'ORB tracking file'
    timing_stamps = []
timing_stats = None
if timing:
    ordered = sorted(timing)
    timing_stats = dict(source=timing_source, count=len(timing), mean_ms=statistics.mean(timing),
                        median_ms=statistics.median(timing), p95_ms=ordered[min(len(ordered)-1,int(.95*len(ordered)))],
                        max_ms=max(timing))
    print(f'Logged tracking/update timings: n={len(timing)}, mean={timing_stats["mean_ms"]:.2f} ms, median={timing_stats["median_ms"]:.2f}, p95={timing_stats["p95_ms"]:.2f}, max={timing_stats["max_ms"]:.2f} ({timing_source})')
    print('These are processing times, not measured output Hz or total estimator CPU.')
    if len(timing_stamps)>1 and all(b>a for a,b in zip(timing_stamps,timing_stamps[1:])):
        timing_stats['completed_update_hz'] = (len(timing_stamps)-1)/(timing_stamps[-1]-timing_stamps[0])
        print(f'Completed update rate over recorded sensor timestamps: {timing_stats["completed_update_hz"]:.2f} Hz')
else: print('No tracking/update timings from this run; estimator may not have initialized.')
positions = [tuple(map(float,x)) for x in re.findall(r'p_IinG = ([\-0-9.]+),([\-0-9.]+),([\-0-9.]+)',log)]
if positions:
    import math
    net = math.dist(positions[0],positions[-1])
    maximum = max(math.dist(positions[0],x) for x in positions)
    print(f'OpenVINS logged poses: {len(positions)}; final displacement={net:.3f} m; max displacement={maximum:.3f} m (rounded log positions, no ground truth)')
imu = (root/'imu.log').read_text(errors='replace') if (root/'imu.log').exists() else ''
counts = re.findall(r'published=(\d+) corrupt=(\d+) sequence_gaps=(\d+) saturated=(\d+)',imu)
if counts: print('Last periodic IMU counters: published={} corrupt={} gaps={} saturated={}'.format(*counts[-1]))
rates = [float(v) for v in re.findall(r'IMU delivery rate=([0-9.]+) Hz', imu)]
if rates: print(f'Periodic IMU delivery rate: min={min(rates):.1f}, max={max(rates):.1f} Hz')
health_errors = [int(v) for v in re.findall(r'HEALTH samples=\d+ elapsed_ms=\d+ read_errors=(\d+)',imu)]
if health_errors and sum(health_errors):
    print(f'SENSOR WARNING: firmware reported {sum(health_errors)} I2C read errors across {sum(v>0 for v in health_errors)} health windows (maximum {max(health_errors)}). Average rate does not establish measurement integrity.')
if (root/'diagnostics_enabled').exists():
    print('Diagnostic run: sensor recording and debug logging costs are included. Close the bag writer before inspecting its database.')
if 'IMU unhealthy' in imu or 'ERROR IMU source' in imu or 'ERROR IMU configuration' in imu:
    print('SENSOR FAILURE: IMU rate/configuration failed validation. VIO stopped; this run cannot establish trajectory quality.')
if 'ERROR IMU setup failed' in imu or 'IMU startup timed out' in imu:
    print('SENSOR FAILURE: IMU did not start. Check power/I2C wiring; no valid VIO comparison from this run.')
if 'IMU stream stopped' in imu:
    print('SENSOR FAILURE: IMU stream stopped during this run. Check connection; pipeline was shut down.')
if any(message in imu for message in ('IMU serial disconnected', 'IMU serial EOF', 'IMU serial read failed')):
    print('SENSOR FAILURE: serial connection failed. VIO stopped; inspect the IMU connection.')
if 'Not enough motion for initializing' in log:
    print('ORB reported insufficient initialization motion; tracking time alone does not establish valid VIO.')
(root/'summary.json').write_text(json.dumps(dict(processes=rows,child_cpu_seconds=cpu,launch_wall_seconds=wall,logged_updates=len(timing),timing=timing_stats,reported_i2c_read_errors=sum(health_errors),diagnostics_enabled=(root/'diagnostics_enabled').exists(),benchmark_enabled=(root/'benchmark_enabled').exists()),indent=2)+'\n')
print('Saved run:',root)
print('=============================================')
