# OpenVINS hardware source

This directory is the ROS 2 build workspace. The source version is recorded in
`SOURCE_REVISION`.

| Directory | Component |
| --- | --- |
| `ov_core/` | Feature tracking, camera models and mathematical types |
| `ov_init/` | State initialization |
| `ov_msckf/` | Estimation, ROS subscriptions and state publication |
| `ov_hw290/` | IMU bridge, stereo splitter and calibration utilities |
| `hw290_stereo/` | Setup, firmware, configuration and launch scripts |
| `config/` | Estimator configuration files |
| `benchmark/` | Trajectory error and resource measurement utilities |

## Build

On Ubuntu 22.04, run as a normal account:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

The installer builds the estimator packages and patched camera driver for the
destination computer. See the [implementation guide](../../OPENVINS_IMPLEMENTATION_GUIDE.md)
for manual installation, hardware commissioning and acceptance criteria.

## Licensing

OpenVINS licensing is defined in `LICENSE`. The AprilTag dependency retains its
license in `ov_hw290/third_party/ethz_apriltag2/LICENSE`.
