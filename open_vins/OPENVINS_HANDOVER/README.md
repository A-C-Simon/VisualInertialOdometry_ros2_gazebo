# OpenVINS source and documentation

This folder contains the complete source for the working ELP stereo, HW290 and
Arduino Nano pipeline, its setup scripts, technical documentation and test evidence.
Copy the whole folder. No personal Git repository or account is needed.

## Install

The tested host is Ubuntu 22.04 x86_64 with ROS 2 Humble. Copy or extract this
folder to `$HOME/OPENVINS_HANDOVER`, then open Bash as your normal account:

```bash
cd "$HOME/OPENVINS_HANDOVER"
sha256sum -c SHA256SUMS
cd source/open_vins
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Change the path if the folder is stored elsewhere. The installer requests sudo
for system packages. Internet access is required for Ubuntu/ROS packages,
Arduino tools, the public usb_cam driver and optional Kalibr/Allan builds.
The OpenVINS implementation itself is already supplied locally.

Continue with Sections 5-9 of the [implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md)
for wiring checks, firmware upload, calibration and acceptance. A new assembly
needs its own calibration. The included tower profile applies only to the
unchanged reference assembly. IMU noise remains provisional.

## Contents

| Path | Contents |
| --- | --- |
| [Implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md) | Assembly, installation, calibration, operation and acceptance |
| [Calibration tools](CALIBRATION_TOOLS.md) | Kalibr/Allan build and fitting procedures |
| [Drift fixes](DRIFT_FIX.md) | Calibration and ROS delivery corrections |
| [Source workspace](source/open_vins/README.md) | Estimator, sensor tools, scripts, firmware, configs and licenses |
| `reference_files/` | Selected tower calibration, CAD and target files |
| `evidence/calibration/` | Original fit reports and calibration results |
| `evidence/desk_test/` | State estimates, trajectory plot and resource summaries |
| [Files and sources](FILES_AND_SOURCES.md) | Local source map and public dependencies |
| [Camera research](CAMERA_RESEARCH_100M.md) | Future 100 m aerial camera study |
| `manifest.json`, `SHA256SUMS` | Source version, file inventory and integrity checks |

## Delivery limits

No Git history, compiled binaries, Docker images or daily work reports are included.
Build outputs are generated on the destination device. Software licenses and
required notices remain in the source files.

Historical raw bags are excluded: approximately 1.7 GB for calibration and 2.9 GB
for the desk test. Record new data for a new device. The supplied reports and
state estimates support inspection but cannot replay the original sensor stream.
The desk result demonstrates bounded operation for that test, not independently
measured accuracy. The camera study has no validated 100 m flight result.
