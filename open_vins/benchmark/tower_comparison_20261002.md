# Tower replay comparison, October 2

Both estimators received the same saved final tower test at real time, in
separate sequential runs. The recording contains 16,071 IMU measurements and
5,002 images per camera over 166.5 seconds. Actual movement was confirmed
inside the 140 x 80 cm desk area and 60 cm height limit.

The selected tower calibration is shared. ORB uses 600 features, a 12-keyframe
local inertial BA cap, and disabled loop closure. Its upstream motion reset and
inertial refinement remain enabled. OpenVINS uses the selected fixed profile
with the IMU queue and camera callback fixes.

| Measurement | OpenVINS | ORB efficient |
| --- | ---: | ---: |
| Estimator CPU time, s | 112.08 | 134.01 |
| Average CPU, % of one core | 63.86 | 76.02 |
| Peak resident memory, MiB | 241.97 | 803.55 |
| Logged processing mean, ms | 13.54 | 13.97 |
| Logged processing p95, ms | 18.01 | 16.90 |
| Online poses | 3,849 | 4,566 |
| Online pose span, s | 128.19 | 162.14 |
| Maximum displacement, m | 0.910 | 4.501 |
| Maximum position step, m | 0.021 | 4.432 |
| Maximum pose interval, s | 0.036 | 2.000 |
| Active map resets | 0 | 54 |
| Later inertial refinement completed | Not applicable | Neither stage |

CPU and memory cover the estimator process, including startup and shutdown.
The player, hardware drivers, RViz and recorder are excluded. Both estimators
received 16,069 IMU samples within their subscription lifetimes, with no gap
over 50 ms. ORB processed 4,635 stereo pairs and logged queue overflows.
OpenVINS waits for stationary initialization, explaining its shorter pose span.

Processing timings have different scopes: OpenVINS logs the update total;
ORB logs TrackStereo and excludes background local mapping. CPU time captures
all estimator threads and is the better cost comparison here. One trial per
profile does not establish a precise performance distribution.

ORB used 19.6% more CPU time and 3.32 times the peak memory. Its trajectory
failed the physical bounds, so this is not an accuracy comparison. The desk
recording has no independent ground truth. ORB initialization, frame coverage
and stability must improve before declaring that an optimization preserves VIO
quality. Initial full inertial BA can also move the map's unconstrained
translation origin; this is being investigated separately from motion resets.

The upstream [calibration tutorial](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Calibration_Tutorial.pdf)
describes translation and yaw as choices of world frame. The
[initialization discussion](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/736)
also stresses extrinsic calibration, time alignment and roll/pitch excitation.
These support diagnostic checks; they do not establish a fix for this recording.

See [the machine-readable measurements](tower_comparison_20261002.json).
Full ignored evidence is in `results/tower_matched_20261002/{openvins_fixed,orb_native600}/`.
