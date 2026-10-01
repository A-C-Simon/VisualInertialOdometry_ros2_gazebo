# HW290 offline calibration

The active camera to IMU transform, time offset and IMU noise values are
provisional. Do not replace them until the calibration reports pass the checks
below.

## 1. Print the target

Print `aprilgrid_6x6_a4.pdf` at 100 percent or actual size on flat A4 paper.
Measure one black tag edge after printing. It must be 23.0 mm. If the printer
scaled it, enter the measured edge length in metres as `tagSize` in
`aprilgrid_6x6_a4.yaml`. Keep the sheet flat and rigid.

## 2. Measure IMU noise

Place the complete rig on a vibration-free surface and do not touch it for at
least three hours:

```bash
./hw290_stereo/run_hw290_openvins.sh --imu-allan
```

The saved run contains `imu_bag` and the raw serial log. Use Allan variance to
replace the four provisional noise and random-walk values before the final
camera to IMU calibration.

## 3. Record raw stereo calibration data

Run this command twice, stopping each run with Ctrl-C:

```bash
./hw290_stereo/run_hw290_openvins.sh \
  --sensors-only --no-rviz --diagnostics --raw-stereo
```

For the first run, hold the rig fixed and move the AprilGrid for one to two
minutes. Cover the full image, use different distances, and tilt the grid.

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
