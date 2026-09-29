#!/usr/bin/env python3
"""Bounded, sequential EuRoC trials. Source ROS Humble before running.
CPU/RSS include estimator startup and shutdown; player cost is separate.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]

def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('method', choices=['openvins', 'orb', 'orb_ros'])
    p.add_argument('--dataset', type=Path, default=Path('/tmp/euroc/V1_01_easy'))
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--threads', type=int)
    p.add_argument('--core-library-dir', type=Path)
    p.add_argument('--local-ba-window', type=int)
    p.add_argument('--orb-executable', type=Path, default=ROOT.parent/'ORB_SLAM/install/orb_slam_ros2/lib/orb_slam_ros2/stereo_imu_node')
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='63', ROS_LOG_DIR=str(out/'ros_logs'))
    if args.threads is not None: env['ORB_CV_THREADS'] = str(args.threads)
    if args.core_library_dir: env['LD_LIBRARY_PATH'] = str(args.core_library_dir.resolve())+':'+env.get('LD_LIBRARY_PATH','')
    if args.local_ba_window is not None: env['ORB_LOCAL_BA_WINDOW']=str(args.local_ba_window)
    dataset, config = args.dataset.resolve(), args.config.resolve()
    (out/'configuration_snapshot.yaml').write_text(config.read_text())
    time_cmd = ['/usr/bin/time', '-v', '-o', str(out/'resources.txt')]
    if args.method == 'openvins':
        command = [str(ROOT/'install_vio/ov_msckf/lib/ov_msckf/run_subscribe_msckf'),
                   str(config), '--ros-args', '-r', '__ns:=/ov_msckf',
                   '-p', 'multi_threading_pubs:=false', '-p', 'verbosity:=WARNING']
        if args.threads is not None: command += ['-p', f'num_opencv_threads:={args.threads}']
    elif args.method == 'orb_ros':
        command = [str(args.orb_executable.resolve()), '--ros-args',
                   '-r', '__ns:=/orbslam_vio', '-p', 'use_viewer:=false',
                   '-p', 'settings_path:='+str(config),
                   '-p', 'vocabulary_path:=/home/ac/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                   '-p', 'trajectory_path:='+str(out/'retrospective.txt'),
                   '-p', 'keyframe_trajectory_path:='+str(out/'keyframes.txt'),
                   '-p', 'timing_path:='+str(out/'tracking_ms.txt')]
        if args.threads is not None: command += ['-p', f'opencv_threads:={args.threads}']
    else:
        times = out/'timestamps.txt'
        times.write_text('\n'.join(line.split(',')[0] for line in
                         (dataset/'mav0/cam0/data.csv').read_text().splitlines()
                         if line and not line.startswith('#'))+'\n')
        command = ['/tmp/orb_online_build/euroc_online', '/home/ac/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                   str(config), str(dataset), str(times), 'trial']
    (out/'invocation.json').write_text(json.dumps({'command':command,'threads':args.threads,
        'dataset':str(dataset),'config':str(config),'core_library_dir':str(args.core_library_dir),'local_ba_window':args.local_ba_window},indent=2))
    start = time.monotonic()
    player_return = None
    with (out/'estimator.log').open('w') as log:
        proc = subprocess.Popen(time_cmd+command,cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if args.method in ('openvins', 'orb_ros'):
                with (out/'player.log').open('w') as player_log:
                    player = subprocess.run(['/usr/bin/python3',str(ROOT/'benchmark/euroc_player.py'),
                        str(dataset),'--trajectory',str(out/'online.txt'),'--post-roll','3'] + (['--orb'] if args.method == 'orb_ros' else []),
                        env=env,stdout=player_log,stderr=subprocess.STDOUT,timeout=200)
                    player_return=player.returncode
                stop(proc)
            else:
                proc.wait(timeout=240)
        finally:
            stop(proc)
    result={'elapsed_s':time.monotonic()-start,'estimator_exit':proc.returncode,'player_exit':player_return}
    (out/'completion.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    if proc.returncode not in (0,130) or player_return not in (None,0): raise SystemExit(1)

if __name__=='__main__': main()
