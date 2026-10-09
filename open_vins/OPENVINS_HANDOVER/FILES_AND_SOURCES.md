# Files and sources

Source revision: [`a46f7f31cb3e1e8fe8e32f800f2f12375432d23a`](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a). All source links below use this
revision. `manifest.json` also indexes every tracked file in the hardware
packages, so files named in code examples can be located without this machine.

## Runtime source obtained by cloning

| Component | Git source |
| --- | --- |
| OpenVINS core / tracker | [ov_core](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/ov_core) |
| Initializer | [ov_init](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/ov_init) |
| Estimator and ROS subscriber fixes | [ov_msckf](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/ov_msckf) |
| C++ sensor, calibration and profile tools | [ov_hw290](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/ov_hw290) |
| Hardware scripts and configuration | [hw290_stereo](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/tree/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo) |

The custom AprilTag source and its license are inside `ov_hw290/third_party`.
The setup scripts obtain the remaining external tools below.

## Scripts used by the implementation guide

- [setup_hw290_openvins.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/setup_hw290_openvins.sh)
- [env_hw290.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/env_hw290.sh)
- [build_camera_driver.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/build_camera_driver.sh)
- [build_calibration_tools.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/build_calibration_tools.sh)
- [run_hw290_openvins.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/run_hw290_openvins.sh)
- [process_helpers.sh](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/process_helpers.sh)
- [managed_process.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/managed_process.py)
- [run_summary.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/run_summary.py)
- [static_transform_args.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/static_transform_args.py)
- [inspect_sensors.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/inspect_sensors.py)
- [check_rectified_stereo.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/check_rectified_stereo.py)
- [imu_protocol.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/imu_protocol.py)
- [hw290_imu.py](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/hw290_imu.py)

These scripts run in the complete cloned workspace. Reference files below
retain the original repository-relative layout for inspection; do not run a
partial workspace or copy its old calibration onto a new physical assembly.

## Included reference files

| Included file | Original Git source |
| --- | --- |
| [reference_files/hw290_stereo/calibration/20261002_tower/README.md](reference_files/hw290_stereo/calibration/20261002_tower/README.md) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/README.md) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/README.md](reference_files/hw290_stereo/calibration/20261002_tower/candidate/README.md) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/README.md) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/camchain.yaml](reference_files/hw290_stereo/calibration/20261002_tower/candidate/camchain.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/camchain.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/camchain_raw.yaml](reference_files/hw290_stereo/calibration/20261002_tower/candidate/camchain_raw.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/camchain_raw.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/estimator_config.yaml](reference_files/hw290_stereo/calibration/20261002_tower/candidate/estimator_config.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/estimator_config.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/fit_review.json](reference_files/hw290_stereo/calibration/20261002_tower/candidate/fit_review.json) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/fit_review.json) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/imu.yaml](reference_files/hw290_stereo/calibration/20261002_tower/candidate/imu.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/imu.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/candidate/stereo_opencv.yaml](reference_files/hw290_stereo/calibration/20261002_tower/candidate/stereo_opencv.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/candidate/stereo_opencv.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/geometry.yaml](reference_files/hw290_stereo/calibration/20261002_tower/geometry.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/geometry.yaml) |
| [reference_files/hw290_stereo/calibration/20261002_tower/openscad_echoes.txt](reference_files/hw290_stereo/calibration/20261002_tower/openscad_echoes.txt) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/20261002_tower/openscad_echoes.txt) |
| [reference_files/hw290_stereo/calibration/Dockerfile.allan](reference_files/hw290_stereo/calibration/Dockerfile.allan) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/Dockerfile.allan) |
| [reference_files/hw290_stereo/calibration/allan_hw290.yaml](reference_files/hw290_stereo/calibration/allan_hw290.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/allan_hw290.yaml) |
| [reference_files/hw290_stereo/calibration/aprilgrid_6x6.png](reference_files/hw290_stereo/calibration/aprilgrid_6x6.png) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/aprilgrid_6x6.png) |
| [reference_files/hw290_stereo/calibration/aprilgrid_6x6_a4.pdf](reference_files/hw290_stereo/calibration/aprilgrid_6x6_a4.pdf) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/aprilgrid_6x6_a4.pdf) |
| [reference_files/hw290_stereo/calibration/aprilgrid_6x6_a4.yaml](reference_files/hw290_stereo/calibration/aprilgrid_6x6_a4.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/aprilgrid_6x6_a4.yaml) |
| [reference_files/hw290_stereo/calibration/aprilgrid_6x6_screen.yaml](reference_files/hw290_stereo/calibration/aprilgrid_6x6_screen.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/aprilgrid_6x6_screen.yaml) |
| [reference_files/hw290_stereo/calibration/current_mount.yaml](reference_files/hw290_stereo/calibration/current_mount.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/current_mount.yaml) |
| [reference_files/hw290_stereo/calibration/docker_context/.dockerignore](reference_files/hw290_stereo/calibration/docker_context/.dockerignore) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/calibration/docker_context/.dockerignore) |
| [reference_files/hw290_stereo/estimator_config.yaml](reference_files/hw290_stereo/estimator_config.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/estimator_config.yaml) |
| [reference_files/hw290_stereo/firmware/hw290_openvins/hw290_openvins.ino](reference_files/hw290_stereo/firmware/hw290_openvins/hw290_openvins.ino) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/firmware/hw290_openvins/hw290_openvins.ino) |
| [reference_files/hw290_stereo/patches/usb_cam-0.8.1-timestamps.patch](reference_files/hw290_stereo/patches/usb_cam-0.8.1-timestamps.patch) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/patches/usb_cam-0.8.1-timestamps.patch) |
| [reference_files/hw290_stereo/record_qos.yaml](reference_files/hw290_stereo/record_qos.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/record_qos.yaml) |
| [reference_files/hw290_stereo/rviz_hw290.rviz](reference_files/hw290_stereo/rviz_hw290.rviz) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/rviz_hw290.rviz) |
| [reference_files/hw290_stereo/rviz_hw290_sensors.rviz](reference_files/hw290_stereo/rviz_hw290_sensors.rviz) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/rviz_hw290_sensors.rviz) |
| [reference_files/hw290_stereo/usb_cam_hw290.yaml](reference_files/hw290_stereo/usb_cam_hw290.yaml) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/usb_cam_hw290.yaml) |
| [reference_files/hw290_stereo/vio_rig_stand_simple.scad](reference_files/hw290_stereo/vio_rig_stand_simple.scad) | [Source](https://github.com/A-C-Simon/VisualInertialOdometry_ros2_gazebo/blob/a46f7f31cb3e1e8fe8e32f800f2f12375432d23a/open_vins/hw290_stereo/vio_rig_stand_simple.scad) |

## Included local evidence

These files were not tracked in the original source revision. They are
included verbatim. The manifest records original relative paths and hashes.

- [evidence/calibration/tower-report-cam.pdf](evidence/calibration/tower-report-cam.pdf)
- [evidence/calibration/tower-report-imucam.pdf](evidence/calibration/tower-report-imucam.pdf)
- [evidence/calibration/tower-results-cam.txt](evidence/calibration/tower-results-cam.txt)
- [evidence/calibration/tower-results-imucam.txt](evidence/calibration/tower-results-imucam.txt)
- [evidence/calibration/tower-camchain.yaml](evidence/calibration/tower-camchain.yaml)
- [evidence/calibration/tower-camchain-imucam.yaml](evidence/calibration/tower-camchain-imucam.yaml)
- [evidence/calibration/tower-imu.yaml](evidence/calibration/tower-imu.yaml)
- [evidence/calibration/target.yaml](evidence/calibration/target.yaml)
- [evidence/calibration/confirmed_target.yaml](evidence/calibration/confirmed_target.yaml)
- [evidence/calibration/confirmed_display_geometry.txt](evidence/calibration/confirmed_display_geometry.txt)
- [evidence/calibration/capture_check.json](evidence/calibration/capture_check.json)
- [evidence/calibration/vi_fit/fit_scope.json](evidence/calibration/vi_fit/fit_scope.json)
- [evidence/desk_test/analysis.json](evidence/desk_test/analysis.json)
- [evidence/desk_test/summary.json](evidence/desk_test/summary.json)
- [evidence/desk_test/summary.txt](evidence/desk_test/summary.txt)
- [evidence/desk_test/estimator.resources.json](evidence/desk_test/estimator.resources.json)
- [evidence/desk_test/current_mount.yaml](evidence/desk_test/current_mount.yaml)
- [evidence/desk_test/record_qos.yaml](evidence/desk_test/record_qos.yaml)
- [evidence/desk_test/state_estimate.txt](evidence/desk_test/state_estimate.txt)
- [evidence/desk_test/trajectory_check.png](evidence/desk_test/trajectory_check.png)

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

## Files produced on the destination device

- `install_vio/` and `install_camera/`: generated binaries and environment files.
- `sensors_bag/`, `imu_bag/`, raw serial logs and `target.yaml`: recordings and
  measured target metadata created by the capture modes.
- `stereo.bag`, `dynamic.bag`, `imu.bag`: converted recordings from that device.
- `stereo-camchain.yaml`, `dynamic-camchain-imucam.yaml` and Kalibr PDFs: outputs
  from fitting those recordings.
- `imu_noise.yaml`, `imu_openvins.yaml`: reviewed new-device noise data and model.
- `my_device_01/`, its fingerprints and test summaries: the new device's
  calibration and acceptance record.

`REPLACE_WITH_*` and `/path/to/*` strings in manual procedures denote these
new-device inputs. They are not missing downloads from the original host.
Historical capture paths in the included fit review are provenance only.
