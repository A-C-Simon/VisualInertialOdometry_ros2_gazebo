#!/usr/bin/env python3
"""Summarize completed trials, comparing online IMU poses without scale fitting."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
p = argparse.ArgumentParser()
p.add_argument('trials', nargs='+', help='Directory names within benchmark/results')
p.add_argument('--ground-truth', type=Path, default=Path('/tmp/euroc/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv'))
p.add_argument('--dataset-name', default='EuRoC V1_01_easy',
               help='Dataset label for this comparison; pass the matching start time for other sequences')
p.add_argument('--start-time', type=float, default=1403715303.262143,
               help='EuRoC V1_01 camera start + 30 seconds, after initial inertial BA')
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
directories = [ROOT/'results'/name for name in a.trials]
resources = []
for directory in directories:
    completion = json.loads((directory/'completion.json').read_text())
    if completion['estimator_exit'] not in (0, 130) or completion['player_exit'] != 0:
        raise ValueError(f'Failed trial: {directory}')
    raw = (directory/'resources.txt').read_text()
    def number(label):
        return float(re.search(r'^\s*'+re.escape(label)+r':\s*([\d.]+)', raw, re.M)[1])
    resources.append(dict(trial=directory.name,
        cpu_seconds=number('User time (seconds)')+number('System time (seconds)'),
        peak_rss_mib=number('Maximum resident set size (kbytes)')/1024,
        completion=completion))
command = [sys.executable, str(ROOT/'compare_runs.py'), str(a.ground_truth)]
command += [str(directory/'online.txt') for directory in directories]
steady = json.loads(subprocess.check_output(command+['--start-time',str(a.start_time)],text=True))
full = json.loads(subprocess.check_output(command,text=True))
report = dict(dataset=a.dataset_name, resources=resources,
              post_initialization=steady, full_common_interval=full)
a.output.write_text(json.dumps(report, indent=2)+'\n')
for resource, metric in zip(resources, steady['results']):
    print(f"{resource['trial']}: CPU {resource['cpu_seconds']:.2f} s, "
          f"RSS {resource['peak_rss_mib']:.1f} MiB, "
          f"ATE {metric['ate_rmse_m']*100:.3f} cm, "
          f"RPE {metric['rpe_1s_translation_rmse_m']*100:.3f} cm, "
          f"coverage {metric['approximate_20hz_coverage']*100:.1f}%")
