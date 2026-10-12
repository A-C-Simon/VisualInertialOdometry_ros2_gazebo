# OpenVINS hardware source

This directory is the build workspace. It contains the four ROS 2 packages,
hardware scripts, firmware, calibration files, configurations and benchmark tools.
The source version is recorded in SOURCE_REVISION. Git metadata is not included.

From this directory on Ubuntu 22.04, run as a normal account:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Read the [implementation guide](../../OPENVINS_IMPLEMENTATION_GUIDE.md) before
flashing firmware or operating a new rig. A new assembly needs calibration.
Internet access is required for system packages and external tools.
ORB-SLAM3 source and historical raw sensor recordings are not included.
