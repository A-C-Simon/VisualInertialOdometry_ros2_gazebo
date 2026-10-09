# Offline tools for OpenVINS hardware calibration

Companion to [the implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md).
Run acquisition on the ROS 2 host and offline fitting in ROS 1 Noetic Docker.
No live ROS 1 bridge is required. The historically named `kalibr-hw290` image
is not distributed with this repository and was absent during this documentation
check. Build the tools explicitly rather than assuming that image exists.

The commands below were checked against pinned upstream source and its command
interfaces. A complete fresh Docker build and new sensor fit were **not** run
on 9 October. Keep successful image IDs, build logs and fit reports with the
new device's evidence; upstream apt repositories and base image tags can change.

## Automated installation

From `open_vins`, install the runtime and offline tools together:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

If Docker and runtime dependencies already exist, build just the images:

```bash
./hw290_stereo/build_calibration_tools.sh --jobs 2
```

The helper uses the pinned sources and image names below, runs command-line
smoke checks and records image IDs in `CAL_TOOLS/calibration_image_ids.txt`.
It uses sudo Docker when the account cannot access the daemon. The default
tools directory is `$HOME/vio_calibration_tools`; export `CAL_TOOLS` to choose
another disk. The setup also prepares `rosbags-venv`; source the environment,
then continue with Section 3 to record/analyze noise and fit the new device.
The detailed commands below remain available for manual installation.

## 1. Build Kalibr

Install Docker Engine using the
[official Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/).
Commands below assume the current account can run Docker. Otherwise use
`sudo docker` consistently. No X11 connection is required for PDF generation.

Use Kalibr's [official Docker build method](https://github.com/ethz-asl/kalibr/wiki/installation)
with a pinned source revision and two compilation jobs:

```bash
export VIO_WS="$HOME/VisualInertialOdometry_ros2_gazebo/open_vins"
export CAL_TOOLS="$HOME/vio_calibration_tools"
mkdir -p "$CAL_TOOLS"
git clone https://github.com/ethz-asl/kalibr.git "$CAL_TOOLS/kalibr"
git -C "$CAL_TOOLS/kalibr" checkout 1f60227442d25e36365ef5f72cd80b9666d73467
cd "$CAL_TOOLS/kalibr"
sed 's/catkin build -j$(nproc)/catkin build -j2/' \
  Dockerfile_ros1_20_04 > Dockerfile.two_jobs
docker build -t kalibr-hw290:doc-20261009 -f Dockerfile.two_jobs .
export KALIBR_IMAGE=kalibr-hw290:doc-20261009
docker image inspect "$KALIBR_IMAGE" --format '{{.Id}}'
docker run --rm --entrypoint /bin/bash "$KALIBR_IMAGE" -lc \
  'source /catkin_ws/devel/setup.bash; rosrun kalibr kalibr_calibrate_cameras --help'
cd "$VIO_WS"
```

Do not use the image if its build or smoke check fails. Preserve the error
and resolve it against the pinned source before collecting expensive fits.
Noetic is confined to this offline container; do not source its environment
into the Humble acquisition shell. On another CPU architecture, the base image
and dependency build must also be verified.

## 2. Install ROS bag conversion

Keep conversion tools in a separate virtual environment; do not activate it
in the ROS acquisition launcher shell:

```bash
python3 -m venv "$CAL_TOOLS/rosbags-venv"
"$CAL_TOOLS/rosbags-venv/bin/python" -m pip install rosbags
"$CAL_TOOLS/rosbags-venv/bin/python" -m pip freeze \
  > "$CAL_TOOLS/rosbags-requirements.txt"
"$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" --help
```

The [official conversion interface](https://ternaris.gitlab.io/rosbags/topics/convert.html)
accepts a ROS 2 bag directory as `--src` and a new `.bag` path as `--dst`.
Stop the recorder first. Do not overwrite an existing converted bag or alter
source header timestamps during conversion.

## 3. Build and run Allan variance analysis

The [pinned Allan tool](https://github.com/ori-drs/allan_variance_ros/tree/1d54b602ee7f2ba0427865d63afe4945d913ed24)
requires at least three hours of stationary data. Longer captures may better
resolve slow bias behaviour. Use the intended IMU range, digital filters and
sampling rate; do not calibrate one mode and operate another.

Create an image extending Kalibr with the analysis tool:

```bash
mkdir -p "$CAL_TOOLS/allan_image"
cat > "$CAL_TOOLS/allan_image/Dockerfile" <<'DOCKER'
FROM kalibr-hw290:doc-20261009
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libyaml-cpp-dev python3-yaml python3-numpy python3-scipy python3-matplotlib
RUN mkdir -p /allan_ws/src && \
    git clone https://github.com/ori-drs/allan_variance_ros.git /allan_ws/src/allan_variance_ros && \
    git -C /allan_ws/src/allan_variance_ros checkout 1d54b602ee7f2ba0427865d63afe4945d913ed24
RUN /bin/bash -c 'source /catkin_ws/devel/setup.bash && cd /allan_ws && \
    catkin init && catkin config --extend /catkin_ws/devel \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build allan_variance_ros --no-status --jobs 2'
ENTRYPOINT ["/bin/bash"]
DOCKER
docker build -t kalibr-hw290:allan-20261009 "$CAL_TOOLS/allan_image"
export ALLAN_IMAGE=kalibr-hw290:allan-20261009
```

After recording with `--imu-allan`, select that exact completed run:

```bash
export ALLAN_RUN="$VIO_WS/benchmark/results/REPLACE_WITH_ALLAN_RUN"
export ALLAN_DATA="$CAL_TOOLS/allan_data"
mkdir -p "$ALLAN_DATA"
"$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" \
  --src "$ALLAN_RUN/imu_bag" --dst "$ALLAN_DATA/imu.bag"
cp "$VIO_WS/hw290_stereo/calibration/allan_hw290.yaml" "$ALLAN_DATA/"
docker run --rm --cpus=2 -e OMP_NUM_THREADS=2 -e MPLBACKEND=Agg \
  -v "$ALLAN_DATA:/data" --entrypoint /bin/bash "$ALLAN_IMAGE" -lc '
    set -e
    source /allan_ws/devel/setup.bash
    roscore > /data/roscore.log 2>&1 & core_pid=$!
    trap "kill $core_pid 2>/dev/null || true" EXIT
    sleep 2
    rosrun allan_variance_ros allan_variance /data /data/allan_hw290.yaml \
      > /data/computation.log 2>&1
    cd /data
    rosrun allan_variance_ros analysis.py --data allan_variance.csv \
      --config allan_hw290.yaml --output imu_noise.yaml > analysis.log 2>&1
  '
```

Put only the intended `.bag` in `ALLAN_DATA`. The supplied config uses integer
rates `imu_rate: 100`, `measure_rate: 100` and `sequence_time: 10800`. Adjust
the duration to valid recorded coverage if needed, retaining integer types.
Review acceleration/gyro plots, fitted regions and source integrity. A short
smoke recording cannot establish bias random walk.

`imu_noise.yaml` is a **flat Kalibr noise file**, containing the four
continuous-time coefficients, `rostopic: /imu0` and `update_rate: 100.0`.
Verify those fields and units. It is not the nested OpenVINS IMU model.
Container-created outputs may be root-owned; if needed, change ownership of
this dedicated data directory before editing or copying its files.

## 4. Convert the two calibration captures

Set these paths to the completed stereo and dynamic recordings from the
implementation guide. Both must contain raw, unrectified stereo images:

```bash
export STEREO_RUN="$VIO_WS/benchmark/results/REPLACE_WITH_STEREO_RUN"
export DYNAMIC_RUN="$VIO_WS/benchmark/results/REPLACE_WITH_DYNAMIC_RUN"
export CAL_WORK="$CAL_TOOLS/my_device_01_fit"
mkdir -p "$CAL_WORK"
"$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" \
  --src "$STEREO_RUN/sensors_bag" --dst "$CAL_WORK/stereo.bag"
"$CAL_TOOLS/rosbags-venv/bin/rosbags-convert" \
  --src "$DYNAMIC_RUN/sensors_bag" --dst "$CAL_WORK/dynamic.bag"
cp "$STEREO_RUN/target.yaml" "$CAL_WORK/stereo_target.yaml"
cp "$DYNAMIC_RUN/target.yaml" "$CAL_WORK/dynamic_target.yaml"
cp "$ALLAN_DATA/imu_noise.yaml" "$CAL_WORK/imu_noise.yaml"
```

Manual printed-target runs do not generate `target.yaml`. For those runs,
copy `aprilgrid_6x6_a4.yaml` to each target destination and set `tagSize` to
the actual measured black edge in metres. `tagSpacing` is the gap divided by
the black tag edge, not a distance. Use the right target for each capture.

## 5. Fit stereo and camera/IMU calibration

Run the static stereo fit at four frames per second:

```bash
docker run --rm --cpus=2 -e OMP_NUM_THREADS=2 -e MPLBACKEND=Agg \
  -v "$CAL_WORK:/data" --entrypoint /bin/bash "$KALIBR_IMAGE" -lc '
    set -e
    source /catkin_ws/devel/setup.bash
    cd /data
    rosrun kalibr kalibr_calibrate_cameras --bag stereo.bag --bag-freq 4.0 \
      --topics /cam0/image_raw /cam1/image_raw \
      --models pinhole-radtan pinhole-radtan \
      --target stereo_target.yaml --dont-show-report > stereo_fit.log 2>&1
  '
```

Require `stereo-camchain.yaml`, `stereo-results-cam.txt` and
`stereo-report-cam.pdf`. Review the report before fitting IMU extrinsics:

```bash
docker run --rm --cpus=2 -e OMP_NUM_THREADS=2 -e MPLBACKEND=Agg \
  -v "$CAL_WORK:/data" --entrypoint /bin/bash "$KALIBR_IMAGE" -lc '
    set -e
    source /catkin_ws/devel/setup.bash
    cd /data
    rosrun kalibr kalibr_calibrate_imu_camera --bag dynamic.bag \
      --cams stereo-camchain.yaml --imu imu_noise.yaml \
      --target dynamic_target.yaml --dont-show-report > dynamic_fit.log 2>&1
  '
```

Time calibration is enabled by default. Require
`dynamic-camchain-imucam.yaml`, `dynamic-results-imucam.txt` and
`dynamic-report-imucam.pdf`. Inspect fitted rotation, translation, time shift,
reprojection, IMU residuals and biases. Preserve both logs and reports.

## 6. Prepare the nested OpenVINS IMU model

Use the system Python with PyYAML installed in the implementation guide:

```bash
python3 - <<'PY'
import os
from pathlib import Path
import yaml
class OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)
work = Path(os.environ['CAL_WORK'])
reference = Path(os.environ['VIO_WS']) / \
    'hw290_stereo/calibration/20261002_tower/candidate/imu.yaml'
model = yaml.safe_load(reference.read_text().replace('%YAML:1.0', ''))
noise = yaml.safe_load((work / 'imu_noise.yaml').read_text())
for key in ('accelerometer_noise_density', 'accelerometer_random_walk',
            'gyroscope_noise_density', 'gyroscope_random_walk'):
    value = float(noise[key])
    if not 0 < value < float('inf'):
        raise ValueError(f'Invalid noise coefficient: {key}')
    model['imu0'][key] = value
model['imu0']['rostopic'] = '/imu0'
model['imu0']['update_rate'] = 100.0
(work / 'imu_openvins.yaml').write_text('%YAML:1.0\n---\n' +
    yaml.dump(model, Dumper=OpenCVDumper, sort_keys=False, default_flow_style=None))
PY
```

The dumper retains the sequence indentation required by OpenCV's YAML reader.
This copies the reference identity intrinsic matrices; it does not measure
scale or axis misalignment. Those assumptions must be checked for the new
sensor. Return to Sections 7.5 and 9 of the implementation guide to generate
rectification, select the candidate and run fresh acceptance tests.
