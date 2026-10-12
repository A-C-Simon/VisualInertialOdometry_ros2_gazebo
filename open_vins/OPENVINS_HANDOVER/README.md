# OpenVINS stereo visual inertial odometry

This package implements stereo visual inertial odometry using an ELP USB stereo
camera, HW290 IMU and Arduino Nano. It provides ROS 2 pose, odometry, trajectory
and TF outputs, together with sensor acquisition, calibration and diagnostic tools.

## System requirements

| Component | Requirement |
| --- | --- |
| Operating system | Ubuntu 22.04; reference validation on x86_64 |
| ROS | ROS 2 Humble |
| Camera input | Side-by-side MJPEG, 1280 x 480 at 30 Hz |
| IMU input | HW290 with supported identity, Nano bridge at approximately 100 Hz |
| Mount | Rigid camera/IMU assembly with measured spatial and time calibration |
| Installation | Internet access for system packages and external build tools |

Electrical specifications, firmware configuration and coordinate conventions
are defined in the [implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md).

## Installation

Extract the package to `$HOME/OPENVINS_HANDOVER`. Run in Bash as a normal account:

```bash
cd "$HOME/OPENVINS_HANDOVER"
sha256sum -c SHA256SUMS
cd source/open_vins
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Adjust the path if the package is installed elsewhere. The installer uses sudo
for system dependencies. Omit `--with-calibration-tools` when calibration fitting
is performed on a separate computer.

## Commissioning

Follow Sections 5-9 of the implementation guide to verify wiring, upload firmware,
check sensor delivery, calibrate the assembly and complete physical acceptance.
Activate the resulting profile before normal operation:

```bash
./hw290_stereo/run_hw290_openvins.sh
```

The supplied reference calibration applies to its measured assembly. Each new
camera/IMU assembly requires calibration and validation under its operating conditions.

## Technical documentation

| Document or directory | Contents |
| --- | --- |
| [Implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md) | Architecture, assembly, installation, calibration, operation and acceptance |
| [Calibration tools](CALIBRATION_TOOLS.md) | Kalibr, Allan analysis and bag conversion procedures |
| [Calibration and delivery requirements](DRIFT_FIX.md) | Sensor models, timestamps, buffering and callback lifecycle |
| [Source workspace](source/open_vins/README.md) | Estimator packages, sensor tools, firmware and scripts |
| [Component and dependency index](FILES_AND_SOURCES.md) | Local source paths and dependency versions |
| `reference_files/` | Reference calibration, CAD geometry and target definitions |
| `evidence/calibration/` | Calibration fit reports and measured target data |
| `evidence/desk_test/` | Validation trajectory, analysis and computational cost |
| `manifest.json`, `SHA256SUMS` | Source version, file inventory and integrity checks |
