# OpenVINS engineering handover

Start with [OPENVINS_IMPLEMENTATION_GUIDE.md](OPENVINS_IMPLEMENTATION_GUIDE.md).
This folder can be moved or sent as a ZIP. Included-file links are relative;
other source links point to the exact Git revision below.

## What this implements

The working ELP stereo camera, HW290 IMU and Arduino Nano pipeline on ROS 2
Humble. The tested host was Ubuntu 22.04 x86_64. Another physical assembly
requires its own calibration. The supplied tower profile is a reference for
the unchanged October 2 assembly, not a generic calibration.

The Orin NX class / 100 m camera study is in
[CAMERA_RESEARCH_100M.md](CAMERA_RESEARCH_100M.md). It is future design research,
with no claim of validated aerial performance or a ready industrial-camera driver.

## Get the complete implementation

Internet access is required to obtain source and install dependencies.
The Git repository contains the estimator packages, custom C++ sensor tools,
install/build scripts, firmware, configuration, patches and target files.
Use this fork because it contains the required ROS delivery fixes.

Repository: [VisualInertialOdometry_ros2_gazebo](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo)

Pinned revision: [`a46f7f31cb3e1e8fe8e32f800f2f12375432d23a`](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a)

On a new Ubuntu 22.04 host, open Bash and run as a normal account:

```bash
sudo apt update
sudo apt install git
git clone https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo.git \
  "$HOME/VisualInertialOdometry_ros2_gazebo"
cd "$HOME/VisualInertialOdometry_ros2_gazebo/open_vins"
git checkout --detach a46f7f31cb3e1e8fe8e32f800f2f12375432d23a
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

The guide's `openvins-hw290-setup-20261009` tag is the earlier setup release.
The revision above includes that implementation and the later camera research.
Use the revision above to match this delivery. Both references contain the
same hardware runtime; later changes in this snapshot are documentation.

Continue at Section 5 of the guide after setup. Run all guide commands from
the cloned `open_vins` directory, not from this handover folder. Firmware
flashing, electrical checks, new-device calibration and acceptance still need
the actual hardware. Setup asks for sudo where needed.

## Folder contents

| Item | Purpose |
| --- | --- |
| [Implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md) | Assembly through acceptance, including manual installation |
| [Calibration tools](CALIBRATION_TOOLS.md) | Pinned Kalibr/Allan builds and fitting commands |
| [Calibration and delivery fixes](DRIFT_FIX.md) | Why the working result needs both calibration and ROS delivery corrections |
| [Files and sources](FILES_AND_SOURCES.md) | Included files, Git source links and generated inputs/outputs |
| `reference_files/` | Firmware, camera patch, CAD, targets, configs and selected calibration |
| [Calibration report](evidence/calibration/tower-report-cam.pdf) | Original local stereo fit report |
| [Camera/IMU report](evidence/calibration/tower-report-imucam.pdf) | Original local spatial/time fit report |
| [Desk test analysis](evidence/desk_test/analysis.json) | Successful physical run's motion and delivery metrics |
| [Resource summary](evidence/desk_test/summary.txt) | Per-process computational cost from that run |
| [Trajectory plot](evidence/desk_test/trajectory_check.png) | Desk trajectory review |
| `manifest.json`, `SHA256SUMS` | Source provenance, complete source index and integrity hashes |

## Evidence and new-device inputs

The original calibration and live-run reports were local, outside Git;
copies are included here with source paths and hashes in the manifest.
Historical absolute paths in evidence describe the original machine; they are
not required installation paths.

The old raw ROS bags are not included: the calibration bag is about 1.7 GB
and the desk-test bag about 2.9 GB. They are historical replay data, not
required inputs for implementing a new device. The guide's capture commands
produce the new device's raw bags and measured target YAML. The new fit
produces its own reports, calibration and validation evidence. A completed
long stationary Allan measurement was not available for the reference; its
IMU noise remains provisional.

The included state estimates, summaries, calibration reports and fit review
support inspection of the recorded result. They do not replace the raw bags
for sensor replay or independent ground truth for accuracy measurement.
Daily work reports are not part of this folder.

## Check the received files

From inside this folder on Linux:

```bash
sha256sum -c SHA256SUMS
```

The index covers every delivered file except `SHA256SUMS` itself. No compiled
binaries or Docker images are supplied; the documented tools build them for
the destination device. Existing software licenses remain in their sources;
the OpenVINS license is included as [LICENSE](LICENSE).
