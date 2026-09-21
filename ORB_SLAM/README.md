# ORB_SLAM — live stereo ORB-SLAM3 with ELP USB camera

Live stereo visual SLAM on this machine, packaged so everything needed to
reproduce the run lives in `/home/ac/ORB_SLAM`. Tested working 2026-09-18:
Map Viewer + Current Frame windows open, tracker builds keyframes and map
points from the live feed.

## ROS 2 package

The ROS 2 Humble conversion is in [`orb_slam_ros2/`](orb_slam_ros2/README.md).
It retains direct side-by-side `/dev/video0` capture and also supports
synchronized left/right ROS image topics. It publishes pose, path, TF, tracked
map points, a tracking image, and tracking state.

```bash
source /opt/ros/humble/setup.bash
export ORB_SLAM3_ROOT="$HOME/ORB_SLAM3"
colcon build --packages-select orb_slam_ros2 --symlink-install \
  --cmake-args -DORB_SLAM3_ROOT="$ORB_SLAM3_ROOT"
source install/setup.bash
ros2 launch orb_slam_ros2 stereo_live.launch.py device_id:=0
```

## Gazebo ROS 2 simulation

The complete camera-only simulator now lives beside the ROS 2 wrapper in this
workspace:

- `orb_slam_ros2/` — stereo ORB-SLAM3 ROS 2 node and Gazebo calibration.
- `orbslam3_rover_sim/` — rover, world, Gazebo launch, RViz setup, automatic
  driver, and keyboard teleop.
- `orbslam3_gazebo.sh` — local build/launch entry point. It does not source or
  launch the former package under `/home/ac/ros2_ws/src/open_vins`.

Build both packages from `/home/ac/ORB_SLAM`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam_ros2 \
  --symlink-install --cmake-args -DORB_SLAM3_ROOT=/home/ac/ORB_SLAM3
colcon build --packages-select orbslam3_rover_sim --symlink-install
```

Launch the simulation:

```bash
./orbslam3_gazebo.sh             # automatic circle; same as --auto
./orbslam3_gazebo.sh --auto      # explicitly select automatic driving
./orbslam3_gazebo.sh --teleop    # keyboard driving, with auto driver disabled
```

The script builds automatically when the local simulator is not yet installed;
use `--build` to force a rebuild. By default it opens Gazebo, RViz2, and the
ORB-SLAM3 viewer. Optional display flags are `--headless`, `--no-rviz`, and
`--no-orb-viewer`. `--auto` and `--teleop` are mutually exclusive.

Automatic mode uses a gentle 0.05 m/s, 0.15 rad/s inner loop. Four nearby,
asymmetric landmarks keep distinct stereo features inside the calibrated
close-depth range throughout the loop while remaining clear of the rover.

Teleop controls are arrow keys to drive/turn, `+` and `-` to change the speed
limit, Space to stop, and `Q` to quit. The simulator uses only the synchronized
stereo image topics—there is no IMU sensor or inertial SLAM mode:

- Input: `/cam0/image_raw`, `/cam1/image_raw`
- Output: `/orbslam3/pose`, `/orbslam3/path`,
  `/orbslam3/ground_path`,
  `/orbslam3/tracked_map_points`, `/orbslam3/tracking_image`, and
  `/orbslam3/tracking_state`

The launcher uses ROS domain 42 unless `ROS_DOMAIN_ID` is already set. RViz's
fixed frame is `odom`; the static `map` to `odom` transform converts ORB's
optical forward `+Z` into the rover/Gazebo forward `+X`, so the vehicle sits on
the XY ground plane and travels along it. Camera-only stereo has no gravity
measurement, so its raw path can accumulate roll, pitch, or height drift. RViz
therefore displays `/orbslam3/ground_path`, which is transformed into `odom`
and projected to `z=0` while the raw `/orbslam3/path` remains available for
diagnostics. Shutdown writes
`gazebo_trajectory.txt` and `gazebo_keyframes.txt` in this directory.

The launcher explicitly puts `/home/ac/ORB_SLAM/install` ahead of any ROS
workspace already sourced in the terminal. This prevents the older package of
the same name in `/home/ac/ros2_ws/install` from supplying the wrong launch
file.

## Hardware

- Camera: ELP 3DGS1200P01, synchronized global-shutter stereo pair.
- USB: single `/dev/video0` side-by-side MJPG frame **3200x1200**
  = left **1600x1200** | right **1600x1200** (`/dev/video1` exposes no formats).
- Run and calibrate at the resolution you stream at (here 1600x1200 per eye).

## Calibration (copied into `calib/`)

Source: `/home/ac/Documents/Camera_calib` (screen board 9x6, 35.0 mm squares).
From `calib/results.txt`: 26/26 pairs, stereo rms **0.2908 px**,
mean rectified |dy| 0.184 px, baseline **59.896 mm**, rectified f **1082.8 px**.

`calib/` contains the full calibration output, including the 30 MB
`stereo_calibration.npz`. Only `calibration_opencv.yaml` (K1/D1/K2/D2/R/T)
was needed to derive the ORB-SLAM3 settings; the rest is kept for
re-calibration, rectification checks (`rectified_example.png`), and depth
experiments.

## How ORB-SLAM3 works here (30-second version)

`src/stereo_live_elp.cc` grabs each 3200x1200 frame, splits it into left/right
1600x1200, and feeds the raw (distorted) pair to
`ORB_SLAM3::System::STEREO` via `TrackStereo()`. ORB-SLAM3 extracts ~2000 ORB
features per frame, undistorts/rectifies internally using `config/`, matches
stereo to get depth, tracks camera motion frame-to-frame, spawns keyframes and
3D map points, and runs local mapping + loop closing in background threads.
Two windows appear: **Current Frame** (green tracked features) and
**Map Viewer** (red map points, camera frustums). Bottom status shows
`Maps / KFs / MPs / Matches`.

## Layout

```
ORB_SLAM/
  README.md                  this file
  orb_slam_ros2/             ROS 2 stereo wrapper, launch files, and settings
  orbslam3_rover_sim/        local Gazebo rover simulation package
  orbslam3_gazebo.sh         auto/teleop Gazebo ROS 2 launcher
  config/ELP_1600x1200.yaml  ORB-SLAM3 PinHole stereo settings derived from calib/
  calib/                     full copy of Camera_calib/calib/ (see above)
  src/stereo_live_elp.cc     live side-by-side capture node (integrated into ORB_SLAM3)
  scripts/run_live.sh        one-command launcher (uses the ~/ORB_SLAM3 build)
  live_session.txt           trajectory, written here on shutdown (after a run)
  kf_live_session.txt        keyframe trajectory, written here on shutdown
```

Heavy build products stay where they were built and are **not** duplicated:
`~/ORB_SLAM3/lib/libORB_SLAM3.so`, `~/ORB_SLAM3/Examples/Stereo/stereo_live_elp`
(39 KB, RUNPATH points at its libs), `~/ORB_SLAM3/Vocabulary/ORBvoc.txt`
(139 MB). The launcher points at them via absolute paths / `ORB_SLAM3_DIR`.

## Prerequisites (as tested)

- Ubuntu 22.04, OpenCV 4.5.4, Eigen3, Boost 1.74, g++ 11, X11 `DISPLAY=:1`.
- Pangolin 0.9.6 source at `~/Pangolin`, installed to `/usr/local`.
- ORB-SLAM3 checkout at `~/ORB_SLAM3` with vocabulary uncompressed
  (`Vocabulary/ORBvoc.txt`). ROS 2 Humble is used by `orb_slam_ros2` and the
  Gazebo package; `scripts/run_live.sh` remains the standalone alternative.

## Build (already done on this machine; for a fresh setup)

1. Pangolin — must avoid the Linuxbrew OpenEXR 3.4 trap. The prebuilt
   `libpango_image.so` referenced `libOpenEXR-3_4.so.33` from
   `/home/linuxbrew/.linuxbrew`, which breaks system `cmake`/linking
   (brew libs need glibc 2.36/2.38, system has 2.35). Reconfigure without EXR
   (EXR image support is unused by ORB-SLAM3):
   ```
   cd ~/Pangolin/build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_LIBOPENEXR=OFF
   make -j2 && sudo make install && sudo ldconfig
   ldd /usr/local/lib/libpango_image.so | grep -i "not found"  # expect none
   ```
2. ORB-SLAM3 core + examples:
   ```
   cd ~/ORB_SLAM3/build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j2
   ```
3. Live node: `src/stereo_live_elp.cc` is already wired into
   `~/ORB_SLAM3/CMakeLists.txt` as target `stereo_live_elp`
   (binary lands in `~/ORB_SLAM3/Examples/Stereo/`). On a fresh clone, copy
   `src/stereo_live_elp.cc` to `Examples/Stereo/`, add the
   `add_executable(stereo_live_elp ...)` block next to `stereo_tum_vi`,
   and rebuild. Keep a copy of `config/ELP_1600x1200.yaml` as the settings file.

## Run

```
./scripts/run_live.sh [device_id] [trajectory_name]
# defaults: device 0 (/dev/video0), trajectory "live_session"
# example:  ./scripts/run_live.sh 0 desk_run_01
```

What to do: point at a textured, well-lit scene and **move slowly** with both
rotation and translation. Watch features latch in Current Frame and the map
grow in Map Viewer. Stop with Ctrl+C; trajectories are saved in this directory.

## Config notes (`config/ELP_1600x1200.yaml`)

- `Camera.type: PinHole` with k1/k2/p1/p2/**k3** per eye from `calibration_opencv.yaml`.
- `Camera.width/height: 1600/1200`, `Camera.fps: 30` — **must be an integer**;
  `30.0` aborts with `Camera.fps parameter must be an integer number`.
- `Camera.RGB: 0` (OpenCV VideoCapture delivers BGR).
- `Stereo.T_c1_c2`: 4x4 from OpenCV R|T (cam1 -> cam2).
- `Stereo.ThDepth: 40.0` is **baseline-times**: 40 x 0.0599 m ≈ 2.4 m close-point
  range (code: `mThDepth = b * thDepth`, see `src/Tracking.cc:573`).
- ORB: 2000 features, scaleFactor 1.2, 8 levels, FAST 20/7 — suited to 1600x1200.

## Troubleshooting

- `Failed to open /dev/video0` / black feed: camera busy (only one app at a time),
  unplug/replug USB, check `v4l2-ctl --list-devices` and
  `v4l2-ctl --list-formats-ext -d /dev/video0` (expect MJPG 3200x1200).
- `Camera.fps parameter must be an integer`: settings has `30.0` instead of `30`.
- Link errors mentioning `libOpenEXR-3_4.so.33`: Pangolin picked up Linuxbrew
  OpenEXR — redo the Pangolin step above with `BUILD_PANGOLIN_LIBOPENEXR=OFF`.
- `Fail to track local map!` / new maps spawning: normal during fast motion,
  textureless views, or a static camera. Add light + texture, slow down;
  it relocalizes automatically.
- No windows: need X11 (`echo $DISPLAY`, expect `:1`); run on the desktop session,
  not headless SSH without X forwarding.
- Slow tracking at 1600x1200 on 4 cores: reduce load (close browsers), ensure
  Release build; optionally add `Camera.newWidth/newHeight` to downscale
  (scales fx/fy/cx/cy accordingly — see `src/Settings.cc`).

## Provenance

| Packaged copy | Original |
|---|---|
| `calib/*` | `/home/ac/Documents/Camera_calib/calib/*` |
| `config/ELP_1600x1200.yaml` | `~/ORB_SLAM3/Examples/Stereo/ELP_1600x1200.yaml` |
| `src/stereo_live_elp.cc` | `~/ORB_SLAM3/Examples/Stereo/stereo_live_elp.cc` |
| Upstream ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 |
