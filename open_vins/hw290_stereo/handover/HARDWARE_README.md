# ELP stereo and HW290 acquisition

The hardware pipeline uses an ELP packed stereo stream, an HW290 IMU connected
through a Nano, and the OpenVINS estimator. The launch script starts acquisition,
checks calibration fingerprints and publishes the estimated state through ROS 2.

## Setup and commissioning

Follow the [implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md) for assembly,
dependency installation, firmware upload and physical acceptance.
[CALIBRATION_TOOLS.md](CALIBRATION_TOOLS.md) specifies the calibration toolchain.

Run these commands from the parent `open_vins` directory:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Each physical camera/IMU assembly requires a fitted profile. Follow
[the calibration procedure](calibration/README.md) and
[sensor delivery requirements](DRIFT_FIX.md) before enabling normal estimation.

## Operation

```bash
./hw290_stereo/run_hw290_openvins.sh
./hw290_stereo/run_hw290_openvins.sh --no-rviz
./hw290_stereo/run_hw290_openvins.sh --diagnostics
```

Keep the rig stationary during initialization, then move through a textured,
static scene. Ctrl-C closes the run and prints its resource summary.
Diagnostic results are saved under `benchmark/results/`.

The default IMU backend and stereo splitter are C++. ROS interfaces, device
paths, timestamp conventions and acquisition limits are defined in the guide.
