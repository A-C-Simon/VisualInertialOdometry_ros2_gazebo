# HW290 offline calibration

The active camera to IMU transform, time offset and IMU noise values are
provisional. Do not replace them until the calibration reports pass the checks
below.

## 1. Print the target

Print `aprilgrid_6x6_a4.pdf` at 100 percent or actual size on flat A4 paper.
Measure one black tag edge after printing. It must be 23.0 mm. If the printer
scaled it, enter the measured edge length in metres as `tagSize` in
`aprilgrid_6x6_a4.yaml`. Keep the sheet flat and rigid.

Alternatively, use the calibration screen described below and measure one
complete black tag edge with a ruler. The gap divided by tag edge must be
0.30. Keep display scaling and target size fixed during both recordings.
The screen measurement on October 1 was approximately 40 mm per tag with
12 mm gaps. The printed target YAML still describes the 23 mm printed tags.
For screen recordings, use the saved run's `target.yaml` in both Kalibr commands
below. Keep the screen fixed and move the rig for both recordings. Check
captured images for glare and flicker before collecting calibration data.

## 2. Measure IMU noise

Place the complete rig on a vibration-free surface and do not touch it for at
least three hours:

```bash
./hw290_stereo/run_hw290_openvins.sh --imu-allan
```

The saved run contains `imu_bag` and the raw serial log. Use Allan variance to
replace the four provisional noise and random-walk values before the final
camera to IMU calibration.

The C++ `allan_variance_ros` tool is built locally in the ignored
`.allan_workspace` directory. It uses upstream commit
`1d54b602ee7f2ba0427865d63afe4945d913ed24` from
https://github.com/ori-drs/allan_variance_ros. It was checked on the 8,870-message
IMU smoke bag; that 89.6 second check is too short for bias noise calibration.
The following commands are for a completed stationary recording of at least
three hours. Put only that one converted bag in the analysis directory.

```bash
rosbags-convert --src /path/to/run/imu_bag \
  --dst /path/to/allan_results/imu.bag
cp hw290_stereo/calibration/allan_hw290.yaml /path/to/allan_results/
docker run --rm --cpus=2 -e OMP_NUM_THREADS=2 \
  -v "$PWD/hw290_stereo/calibration/.allan_workspace:/allan_ws" \
  -v /path/to/allan_results:/data \
  --entrypoint /bin/bash kalibr-hw290 -lc \
  'set -e; source /allan_ws/devel/setup.bash; \
   roscore > /data/roscore.log 2>&1 & core_pid=$!; \
   trap '\''kill "$core_pid" 2>/dev/null || true'\'' EXIT; sleep 2; \
   rosrun allan_variance_ros allan_variance /data /data/allan_hw290.yaml \
   > /data/computation.log 2>&1; \
   cd /data; MPLBACKEND=Agg rosrun allan_variance_ros analysis.py \
   --data allan_variance.csv --config allan_hw290.yaml --output imu.yaml'
```

The result includes `acceleration.png`, `gyro.png` and a flat Kalibr `imu.yaml`.
Inspect the curves and fit before using the four noise values. The configuration
uses integer rates because the upstream tool reads `measure_rate` as an integer.
If `.allan_workspace` is absent, rebuild it from the pinned source:

```bash
mkdir -p hw290_stereo/calibration/.allan_workspace/src
git clone https://github.com/ori-drs/allan_variance_ros.git \
  hw290_stereo/calibration/.allan_workspace/src/allan_variance_ros
git -C hw290_stereo/calibration/.allan_workspace/src/allan_variance_ros \
  checkout 1d54b602ee7f2ba0427865d63afe4945d913ed24
docker run --rm \
  -v "$PWD/hw290_stereo/calibration/.allan_workspace:/allan_ws" \
  --entrypoint /bin/bash kalibr-hw290 -lc \
  'source /catkin_ws/devel/setup.bash; cd /allan_ws; \
   catkin init; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release; \
   catkin build allan_variance_ros --no-status --jobs 2'
```

## 3. Record raw stereo calibration data

For a screen target, use the C++ calibration window:

```bash
./hw290_stereo/run_hw290_openvins.sh --calibration-screen
```

It shows the target at full screen with two small camera previews in the top
right, without covering the grid. Measure the tag edge at this display size
before clicking **Prepare**, and enter it in the measured tag edge field.
The field defaults to 40 mm; `CALIBRATION_TAG_SIZE_M` changes that default.
The border stays
red while the window checks fresh stereo frames, at least seven tags per camera
and four matching tags between the cameras,
and 80 to 120 Hz IMU delivery. After the recorder subscribes to both cameras
and IMU, a five second countdown appears. Green means start moving.
The timer stops recording after 90 seconds and returns to a normal window.
Use `CALIBRATION_SECONDS=60` for the dynamic camera to IMU recording.
Esc stops the recorder and closes the window; the launcher then stops sensors.
Each recording saves `sensors_bag`, `target.yaml`, the displayed image and
recorder log in a `screen_calibration_*` result directory. Use its `target.yaml`
with Kalibr. A live preview and tag count confirm visibility; the Kalibr report
must still verify calibration quality. Detection uses Kalibr's two-bit marker
border, with a fallback for dim or distorted previews in OpenCV 4.5. The
fallback changes preview detection only; images saved in the bag remain raw.

Run this command twice, stopping each run with Ctrl-C:

```bash
./hw290_stereo/run_hw290_openvins.sh \
  --sensors-only --no-rviz --diagnostics --raw-stereo
```

For the first run, change the relative pose of rig and target for one to two
minutes. With a printed grid, hold the rig fixed and move the grid. With a
screen, move the rig. Cover the full image, use different distances, and tilt
the rig or grid.

For the second run, fix the grid and move the rig smoothly for 30 to 60
seconds. Keep the full grid visible while rotating around roll, pitch and yaw
and translating in all three directions. Avoid shocks and motion blur.

## 4. Run Kalibr

Convert each ROS 2 `sensors_bag` to a ROS 1 bag using `rosbags-convert`. Run the
static stereo calibration at about 4 Hz with `pinhole-radtan` for both cameras.
Then run `kalibr_calibrate_imu_camera` with its default time calibration using
the stereo camchain and measured IMU noise file. The local Docker image is
`kalibr-hw290`.

Example conversion and static calibration:

```bash
rosbags-convert --src /path/to/static_run/sensors_bag \
  --dst /path/to/static_run/static.bag
docker run --rm -v /path/to/static_run:/data \
  -v "$PWD/hw290_stereo/calibration:/config:ro" \
  --entrypoint /bin/bash kalibr-hw290 -lc \
  'source /catkin_ws/devel/setup.bash; cd /data; rosrun kalibr \
   kalibr_calibrate_cameras --bag static.bag --bag-freq 4.0 \
   --topics /cam0/image_raw /cam1/image_raw \
   --models pinhole-radtan pinhole-radtan \
   --target /config/aprilgrid_6x6_a4.yaml --dont-show-report'
```

Convert the dynamic bag, copy the generated stereo camchain into its directory
as `camchain.yaml`, and place the Allan result there as a flat Kalibr
`imu.yaml`. It must contain the four measured noise values, `/imu0` as the
topic, and `100.0` as the update rate. Then run:

```bash
docker run --rm -v /path/to/dynamic_run:/data \
  -v "$PWD/hw290_stereo/calibration:/config:ro" \
  --entrypoint /bin/bash kalibr-hw290 -lc \
  'source /catkin_ws/devel/setup.bash; cd /data; rosrun kalibr \
   kalibr_calibrate_imu_camera --bag dynamic.bag \
   --cams camchain.yaml --imu imu.yaml \
   --target /config/aprilgrid_6x6_a4.yaml --dont-show-report'
```

Accept the result only when camera reprojection errors are near 0.1 to 0.2
pixels, predicted IMU curves fit the measurements, errors and biases remain
inside their 3-sigma bounds, and the translation agrees with the measured
mount geometry. Finally, repeat the continuous desk test and require the path
to remain inside the 1.72 m workspace bound.
