# Component and dependency index

## Implementation source

The build workspace is `source/open_vins`. `SOURCE_REVISION` records its source
version; `manifest.json` and `SHA256SUMS` record file identities and integrity.

| Component | Local path |
| --- | --- |
| Feature tracking and core types | [ov_core](source/open_vins/ov_core) |
| Initializer | [ov_init](source/open_vins/ov_init) |
| Estimator and ROS interfaces | [ov_msckf](source/open_vins/ov_msckf) |
| Sensor and calibration tools | [ov_hw290](source/open_vins/ov_hw290) |
| Setup, firmware and launch scripts | [hw290_stereo](source/open_vins/hw290_stereo) |
| Configuration examples | [config](source/open_vins/config) |
| Trajectory and resource utilities | [benchmark](source/open_vins/benchmark) |
| OpenVINS license | [LICENSE](source/open_vins/LICENSE) |

The AprilTag detector and its license are in `ov_hw290/third_party/ethz_apriltag2`.

## Build dependencies

| Dependency | Version or source | Installation |
| --- | --- | --- |
| Camera driver | [usb_cam 0.8.1](https://github.com/ros-drivers/usb_cam/tree/0.8.1) | `build_camera_driver.sh` applies the timestamp patch and builds the driver |
| Kalibr | [1f60227442d25e36365ef5f72cd80b9666d73467](https://github.com/ethz-asl/kalibr/tree/1f60227442d25e36365ef5f72cd80b9666d73467) | `build_calibration_tools.sh` builds the calibration image |
| Allan analysis | [1d54b602ee7f2ba0427865d63afe4945d913ed24](https://github.com/ori-drs/allan_variance_ros/tree/1d54b602ee7f2ba0427865d63afe4945d913ed24) | The Allan Docker recipe builds the analysis tool |
| Arduino CLI | [1.5.1](https://github.com/arduino/arduino-cli/tree/v1.5.1), AVR core 1.8.8 | The setup script installs the CLI and AVR core |
| ROS bag conversion | [rosbags](https://gitlab.com/ternaris/rosbags) | The setup script creates an isolated Python environment and records installed versions |
| ROS | [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) | System packages installed through apt |
| Container runtime | [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) | Installed by the calibration-tools setup option when required |

## Calibration and validation records

| Directory | Record |
| --- | --- |
| `reference_files/` | Reference camera/IMU profile, firmware, timestamp patch, CAD and target definitions |
| `evidence/calibration/` | Target measurements, camera and IMU fit results, residual reports and fitting scope |
| `evidence/desk_test/` | Saved state estimates, trajectory analysis, plot and resource measurements |

Use the reference profile to inspect file formats and calibration conventions.
Commission each new physical assembly with its own measurements.

## Generated outputs

| Stage | Output |
| --- | --- |
| Installation | `install_vio/`, `install_camera/`, build directories and software version logs |
| Acquisition | Sensor bags, raw IMU logs and measured target YAML |
| Calibration | Raw and rectified camera chains, IMU model, stereo matrices and fit reports |
| Acceptance | Mount fingerprints, motion checks, trajectory and resource summaries |

Replace `REPLACE_WITH_*` and `/path/to/*` command arguments with the corresponding
paths for the assembly being commissioned.
