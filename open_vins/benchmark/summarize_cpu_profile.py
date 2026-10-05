#!/usr/bin/env python3
"""Summarize inclusive ORB stage CPU without adding nested scopes."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
invocation = json.loads((a.run / 'invocation.json').read_text())
completion = json.loads((a.run / 'completion.json').read_text())
if completion['player_exit'] != 0 or completion['estimator_exit'] not in (0, 130):
    raise ValueError('Profiling replay did not complete')
if not invocation.get('diagnostic_cpu_profile'):
    raise ValueError('Run did not record enabled CPU profiling')
profile = Path(invocation['cpu_profile_output'])
stages = {}
with profile.open() as file:
    for row in csv.DictReader(file):
        stage = stages.setdefault(row['stage'], {'calls': 0, 'cpu_seconds': 0.0})
        stage['calls'] += int(row['calls'])
        stage['cpu_seconds'] += float(row['inclusive_thread_cpu_seconds'])
raw = (a.run / 'resources.txt').read_text()
def number(label):
    return float(re.search(r'^\s*' + re.escape(label) + r':\s*([\d.]+)', raw, re.M)[1])
cpu = number('User time (seconds)') + number('System time (seconds)')
# Stereo extraction runs on two worker threads. GrabImageStereo runs on the
# tracking thread and LocalMapping::Run on the mapping thread. These three
# scopes do not nest in the measured stereo-inertial implementation.
partition_names = ['Frame::ExtractORB', 'Tracking::GrabImageStereo', 'LocalMapping::Run']
partitions = {name: stages[name]['cpu_seconds'] for name in partition_names}
other = cpu - sum(partitions.values())
if other < -0.05:
    raise ValueError('Stage partition exceeds process CPU; check scope nesting')
data = {
    'scope': 'Diagnostic profiling replay; inclusive thread CPU. Nested stage rows overlap.',
    'run': str(a.run), 'completion': completion, 'invocation': invocation,
    'process_cpu_seconds': cpu,
    'peak_rss_mib': number('Maximum resident set size (kbytes)') / 1024,
    'profile_csv_sha256': hashlib.sha256(profile.read_bytes()).hexdigest(),
    'nonoverlapping_thread_scopes': partitions,
    'other_cpu_seconds': other,
    'stages': stages,
    'limits': 'Instrumentation and scheduling can alter the replay. Final cost and trajectory comparisons require an uninstrumented candidate.',
}
a.output.write_text(json.dumps(data, indent=2) + '\n')
print(f'Process CPU: {cpu:.2f} s; unassigned CPU: {other:.2f} s')
for name, seconds in partitions.items():
    print(f'{name}: {seconds:.2f} s ({100 * seconds / cpu:.1f}%)')
print('Largest inclusive stages (overlap):')
for name, stage in sorted(stages.items(), key=lambda item: item[1]['cpu_seconds'], reverse=True)[:16]:
    print(f"{name}: {stage['cpu_seconds']:.2f} s, {stage['calls']} calls")
