# Start here: the calibration that stopped the large OpenVINS flights

**Use the [tested October 1 profile](calibration/20261001/README.md) for the
unchanged HW290/stereo mount. It is already the normal OpenVINS default.**

Git reference: `hw290-openvins-drift-fix-20261001`, pointing to calibration
commit `0c9cf22`. Calibration files and compact validation results are tracked;
large recordings and Kalibr reports remain in local `benchmark/results/`.

## What changed and what the evidence shows

The correction replaced the camera and camera/IMU models together:

| Part | Tested correction |
| --- | --- |
| Raw stereo camera geometry | Kalibr pinhole/radtan fit from the measured AprilGrid; 59.450 mm baseline |
| Camera-to-IMU alignment | Full Kalibr spatial fit; rotation differs about 26.5 degrees from the earlier profile |
| Camera/IMU timing | Fitted cam0 offset +13.04046 ms, replacing +20 ms |
| Normal VIO image model | New stereo rectification and matching 640x480 intrinsics with zero remaining distortion |
| Camera transforms | Include each rectification rotation; static TF reads the same chain as the estimator |

On the same recorded movement, the previous model reached **18,919.61 m**
displacement. The fitted model stayed below **0.46848 m** and retained visual
features in 2,725 of 2,741 updates. Both replays used the same estimator settings
and dynamic startup because that recording began in motion.

The fresh physical test used normal stationary startup and two minutes of
continuous desk motion. I stayed within the limits and saw no flights.
Maximum displacement across 868 seconds of saved poses was **0.85166 m**, below
the **1.72 m** desk diagonal. No trajectory clipping or origin resets were added.

These checks support correcting the calibration bundle. They do not identify
one parameter as the sole cause or establish accuracy against independent
ground truth. Earlier failed runs lost useful visual corrections while IMU
prediction continued; the new model maintained visual corrections in replay.

## Keep these parts together

The active profile is `hw290_stereo/calibration/20261001/`:

- `stereo_opencv.yaml`: raw camera models and stereo rectification maps.
- `camchain.yaml`: matching rectified camera models and IMU transforms.
- `camchain_raw.yaml`: original raw Kalibr camera/IMU result, for provenance.
- `imu.yaml`: the existing provisional IMU noise and intrinsic settings.
- `validation.json`: measured results and calibration file fingerprints.

`hw290_stereo/estimator_config.yaml` selects the dated camera and IMU chains.
`run_hw290_openvins.sh` selects the dated stereo rectification file.
Keep those selections consistent. The legacy `kalibr_imucam_chain.yaml` is
still used by the existing ORB launcher and is not the tested OpenVINS profile.

The successful run also used the corrected camera acquisition timestamps,
MCU acquisition timestamps in the IMU bridge, approximately 100 Hz IMU delivery,
and 5 ms exposure with gain 255 for the tested lighting. Secured wiring and
usable images are prerequisites. Exposure alone did not stop the later flights.
The larger IMU recording queue protects calibration capture; it does not fix
the estimator's remaining input gaps.

## Repeat this calibration method for a changed mount

1. Check actual IMU delivery and camera texture first. Record raw, unrectified
   stereo images and timestamped IMU measurements with the fullscreen mode:

   ```bash
   ./hw290_stereo/run_hw290_openvins.sh --calibration-screen
   ```

   Measure a complete black tag edge at the actual fullscreen size. The tested
   target had 40 mm edges and 12 mm gaps, but enter the measured size for the
   current display. Keep the screen fixed and move the complete rigid rig,
   covering the view and exciting roll, pitch, yaw and translation.

2. Inspect image coverage and recorded IMU timestamp intervals. Use the saved
   `target.yaml`. Convert the ROS 2 bag to ROS 1 and fit stereo intrinsics and
   relative geometry with Kalibr. Then fit camera/IMU transforms and timing.
   Follow the [calibration procedure](calibration/README.md) for commands.
   Today's IMU fit used the first 60 seconds, before the long recording gaps.
   A new fit should use a clean, sufficiently excited recording.

3. Generate a consistent normal VIO profile. Run stereo rectification with the
   fitted raw camera models. Use the rectified projection intrinsics and zero
   distortion in the OpenVINS chain. For each camera, apply its rectification
   rotation to the raw Kalibr `T_cam_imu`, then invert for `T_imu_cam`.
   Ensure static TF and the estimator load the same selected chain.

4. Save a dated profile and inspect calibration residuals and mount geometry.
   Replay motion beyond the fitting interval, then run a fresh physical desk
   test with stationary startup and at least two minutes of continuous motion.
   Preserve the unclipped trajectory and sensor logs. Make the profile the
   default after these checks pass, keeping its evidence in Git.

Reuse today's numerical profile only while the mount and camera mode stay the
same. A position or orientation change needs a new fit using this method.

## Quick runs

From `/home/ac/VisualInertialOdometry_ros2_gazebo/open_vins`:

```bash
# Normal OpenVINS with RViz and a shutdown benchmark summary
./hw290_stereo/run_hw290_openvins.sh

# Lower visualization cost
./hw290_stereo/run_hw290_openvins.sh --no-rviz

# Save sensor bag, full state and detailed diagnostics
./hw290_stereo/run_hw290_openvins.sh --diagnostics
```

Hold the rig still until VIO initializes, then move through a textured static
scene. Ctrl+C stops the run and prints the summary. Diagnostics include
recording and debug costs, so use ordinary runs for compute comparisons.

If calibration environment overrides were set during experiments, explicitly
select the normal defaults for a quick check:

```bash
env -u HW290_VIO_CONFIG -u HW290_STEREO_CALIBRATION \
  ./hw290_stereo/run_hw290_openvins.sh
```

## Remaining checks

IMU noise is still provisional. The fitted 6.7 cm lever arm needs a better
physical measurement. The complete new bag had no IMU interval above 25 ms,
but the estimator logged 46 gaps above 50 ms. Independent ground truth and
consistent application to ORB-SLAM3 remain pending.
