#!/usr/bin/env python3
"""Compare online IMU trajectories on a shared time interval; no scale fitting."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from evaluate_ate import rigid_align

p=argparse.ArgumentParser()
p.add_argument('ground_truth',type=Path)
p.add_argument('trajectories',type=Path,nargs='+')
p.add_argument('--skip-seconds',type=float,default=0,help='Remove startup time from common interval (report separately)')
p.add_argument('--start-time',type=float,help='Absolute common evaluation start time in seconds')
a=p.parse_args()
g=np.loadtxt(a.ground_truth,delimiter=',',comments='#');g[:,0]*=1e-9
estimates=[]
for path in a.trajectories:
 x=np.atleast_2d(np.loadtxt(path,comments='#'))
 if np.median(x[:,0])>1e12: x[:,0]*=1e-9
 if not np.isfinite(x[:,:8]).all() or np.any(np.diff(x[:,0])<=0): raise ValueError(f'Invalid timestamps/poses: {path}')
 estimates.append(x)
start=max(g[0,0],*(x[0,0] for x in estimates))+a.skip_seconds
end=min(g[-1,0],*(x[-1,0] for x in estimates))
if a.start_time is not None: start=max(start,a.start_time)
if end<=start: raise ValueError('No common interval')
grot=Slerp(g[:,0],Rotation.from_quat(g[:,[5,6,7,4]]))
results=[]
for path,x in zip(a.trajectories,estimates):
 x=x[(x[:,0]>=start)&(x[:,0]<=end)]
 t=x[:,0];truth=np.column_stack([np.interp(t,g[:,0],g[:,j]) for j in range(1,4)])
 errors=np.linalg.norm(rigid_align(x[:,1:4],truth)-truth,axis=1)
 rot=Rotation.from_quat(x[:,4:8]).as_matrix();gtrot=grot(t).as_matrix()
 # Standard relative-pose translation error for pairs approximately 1 s apart.
 relative=[]
 for i,ti in enumerate(t):
  j=int(np.searchsorted(t,ti+1.0))
  if j>=len(t) or abs(t[j]-ti-1.0)>.03: continue
  de=rot[i].T@(x[j,1:4]-x[i,1:4]);dg=gtrot[i].T@(truth[j]-truth[i])
  relative.append(np.linalg.norm(de-dg))
 results.append(dict(trajectory=str(path),poses=len(x),duration_s=float(t[-1]-t[0]),
     approximate_20hz_coverage=min(1.,len(x)/((end-start)*20+1)),
     max_pose_gap_s=float(np.diff(t).max()),ate_rmse_m=float(np.sqrt(np.mean(errors**2))),
     ate_p95_m=float(np.percentile(errors,95)),
     rpe_1s_translation_rmse_m=float(np.sqrt(np.mean(np.square(relative)))) if relative else None))
print(json.dumps(dict(common_start_s=start,common_end_s=end,alignment='SE(3), no scale; GT position interpolated',results=results),indent=2))
