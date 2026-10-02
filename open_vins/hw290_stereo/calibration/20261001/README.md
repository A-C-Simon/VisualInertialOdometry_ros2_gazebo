# Tested HW290 OpenVINS calibration: large drift stopped, 1 October 2026

**This is the profile that passed the replay and physical desk checks after
the earlier meter- and kilometer-scale failures.**
See [what changed and how to repeat the calibration](../../DRIFT_FIX.md).
Git reference: `hw290-openvins-drift-fix-20261001` (`0c9cf22`).

This is the historical profile for the mount tested on October 1. The
[current tower](../20261002_tower/README.md) needs a new camera/IMU fit.
The live launchers reject this profile until the current mount is validated.
The following command applies to the earlier unchanged mount only.
Run from `open_vins`:

```bash
./hw290_stereo/run_hw290_openvins.sh
```

Hold the rig still for initialization, then move through a textured static scene.
Changing the camera/IMU mount requires a new spatial calibration.

## Files and provenance

| File | Purpose |
| --- | --- |
| `stereo_opencv.yaml` | Raw 640x480 camera models and matching stereo rectification |
| `camchain.yaml` | Rectified camera models and camera to IMU transforms used by OpenVINS |
| `camchain_raw.yaml` | Original Kalibr raw camera and IMU fit |
| `imu.yaml` | Existing provisional noise and identity IMU intrinsic model |
| `validation.json` | Compact test results and file fingerprints |

The source capture is
`benchmark/results/screen_calibration_20261001_134827_547/`.
The fullscreen target was measured as 40 mm black tag edges and 12 mm gaps.
The separate ruler confirmation is in
`benchmark/results/screen_target_measurement_20261001_141055_112/`.

Stereo Kalibr used pinhole/radtan cameras with frames selected at 4 Hz.
Camera/IMU fitting used the first 60 seconds, before the long recorded IMU gaps,
with the existing provisional noise. Reports are `calibration-report-cam.pdf`
and `diagnostic_vi_fit/input-report-imucam.pdf` in the capture directory.

The measured baseline is 59.450 mm. Camera reprojection component standard
deviations are about 0.26 to 0.32 pixels. The fitted cam0 offset is
13.04046 ms with the convention `t_imu = t_camera + offset`.
OpenVINS uses cam0's offset for the stereo system.
The mount rotation differs by about 26.5 degrees from the previous profile.

`stereoRectify` produces focal length 367.53325 pixels and principal point
(324.01826, 268.30197). `camchain.yaml` uses these rectified intrinsics and zero
distortion. Its transforms include the corresponding rectification rotations.
The static TF helper reads this same chain through `estimator_config.yaml`.
The earlier `hw290_stereo/kalibr_imucam_chain.yaml` is a legacy profile.
ORB now exports the selected OpenVINS chain; live use requires a matching mount.

## Validation

Replays in the capture's `replay_validation/` directory used the same estimator
settings, dynamic initialization and half-speed playback. The capture has no
stationary startup window. The old geometry was applied by rectifying the raw
monochrome recording offline, which can differ slightly from rectifying RGB
before grayscale conversion in the old live pipeline.

| Check | Result |
| --- | --- |
| Old model replay maximum displacement | 18,919.61 m |
| New model replay maximum displacement | 0.46848 m |
| New model frames with retained visual features | 2,725 of 2,741 |
| Board reference error after 60 seconds | 2.67 cm RMSE, 3.54 cm maximum |
| Physical run maximum displacement | 0.85166 m |
| Physical run pose span | 868.00 seconds |
| Last 30 seconds position range per axis | 2.77, 7.04, 3.69 mm |

The AprilGrid PnP reference shares camera observations and the fitted camera
model. It is not independent ground truth. A rigid alignment without scale
fitting used the first 60 seconds, then evaluated transfer beyond that interval.
The comparison supports consistent motion without measuring absolute accuracy.

The fresh physical run is
`benchmark/results/hw290_openvins_20261001_165958_VkGYYe/`.
It used stationary initialization, the same mount and a two-minute continuous
movement cue within a 1.40 x 0.80 x 0.60 m workspace. I confirmed staying within
the limits and seeing no flights. Later poses include waiting and possible
repositioning; final displacement is not a measured return error.
The 1.72 m desk diagonal is a gross divergence check, not an exact path reference.
`analysis.json` and `trajectory_validation.png` preserve the unclipped results.

## Compute and remaining work

The physical diagnostic run logged mean 14.82 ms and p95 23.70 ms tracking/update
time. Estimator CPU averaged 56.46% of one core with 185.14 MiB peak RSS.
All measured children together averaged 125.59% of one core, including recorder
and RViz. These figures do not establish superiority over ORB-SLAM3.

The bag saved 92,748 IMU samples at 99.02 Hz, maximum interval 22.02 ms, without
nonpositive intervals or intervals above 25 ms. OpenVINS received 91,754 samples
over its shorter lifetime and reported 46 gaps above 50 ms, maximum 280.11 ms.
The count difference includes startup/shutdown; the gaps expose a remaining
estimator delivery issue despite the complete recorded stream.

IMU noise needs a long stationary Allan measurement. The fitted 6.7 cm lever arm
needs a better physical check against the approximate 4 cm separation.
Independent ground truth, more varied motion and lighting, and applying this
calibration consistently to ORB-SLAM3 remain separate work.
