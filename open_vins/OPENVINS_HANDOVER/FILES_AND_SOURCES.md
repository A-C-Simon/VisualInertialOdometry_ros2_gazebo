# Files and sources

## Local implementation

All project source is supplied in `source/open_vins`. Its version is recorded
in `source/open_vins/SOURCE_REVISION`. No project clone or Git account is needed.

| Component | Local path |
| --- | --- |
| Core and feature tracking | [ov_core](source/open_vins/ov_core) |
| Initializer | [ov_init](source/open_vins/ov_init) |
| Estimator and ROS delivery fixes | [ov_msckf](source/open_vins/ov_msckf) |
| C++ sensor and calibration tools | [ov_hw290](source/open_vins/ov_hw290) |
| Setup, firmware and launch scripts | [hw290_stereo](source/open_vins/hw290_stereo) |
| Estimator configuration examples | [config](source/open_vins/config) |
| Analysis and benchmark tools | [benchmark](source/open_vins/benchmark) |
| License | [LICENSE](source/open_vins/LICENSE) |

The custom AprilTag source and license are in `ov_hw290/third_party`.
The manifest records the exported source version, original blob hashes and
delivered file checksums. Documentation links and setup paths use the local copy.

## Reference files and evidence

`reference_files/` contains the selected calibration, firmware, timestamp patch,
CAD and target assets. The complete working copies are also in the source workspace.
`evidence/calibration/` contains twelve original fit/capture files and
`evidence/desk_test/` contains eight original validation files. These evidence
files are unchanged. Historical paths in fit outputs describe the original run.

## External source repositories


| Dependency | Repository / pinned version | How it is obtained |
| --- | --- | --- |
| Patched camera driver | [usb_cam 0.8.1](https://github.com/ros-drivers/usb_cam/tree/0.8.1) | `build_camera_driver.sh` clones it and applies the included timestamp patch |
| Kalibr | [1f60227442d25e36365ef5f72cd80b9666d73467](https://github.com/ethz-asl/kalibr/tree/1f60227442d25e36365ef5f72cd80b9666d73467) | `build_calibration_tools.sh` fetches and builds it |
| Allan analysis | [1d54b602ee7f2ba0427865d63afe4945d913ed24](https://github.com/ori-drs/allan_variance_ros/tree/1d54b602ee7f2ba0427865d63afe4945d913ed24) | Included Docker recipe clones and builds it |
| Arduino CLI | [v1.5.1](https://github.com/arduino/arduino-cli/tree/v1.5.1) | Setup obtains the release; AVR core 1.8.8 is installed separately |
| ROS bag conversion | [rosbags](https://gitlab.com/ternaris/rosbags) | Setup installs it in an isolated Python environment and records installed versions in `rosbags-requirements.txt` |
| ROS 2 Humble | [Official Ubuntu installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) | Setup configures apt and installs the required packages |
| Docker Engine | [Official Ubuntu installation](https://docs.docker.com/engine/install/ubuntu/) | Optional setup step for offline calibration tools |

The installer handles these dependencies; manual commands are in the guides.
Ubuntu package repositories and Docker base-image tags remain external inputs.

## Files generated during installation and calibration

The installer creates `install_vio/`, `install_camera/`, build directories and
version logs. Capture creates new sensor bags and measured target metadata.
Kalibr and Allan analysis produce new camera/IMU chains, noise estimates and
fit reports. Acceptance produces trajectory and resource summaries for that rig.

Paths marked `REPLACE_WITH_*` or `/path/to/*` in commands are new-device inputs.
They are not missing files from the supplied source.
