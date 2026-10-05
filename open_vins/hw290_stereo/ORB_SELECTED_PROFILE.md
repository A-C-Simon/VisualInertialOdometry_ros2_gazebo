# Selected ORB profile, October 5

Use this profile on the unchanged, calibrated rigid tower. It is the best
balance demonstrated by the live checks and public comparisons at the end
of these trials.

```bash
cd /home/ac/VisualInertialOdometry_ros2_gazebo/open_vins
./hw290_stereo/run_selected_orb.sh --rviz
```

Omit `--rviz` for estimator cost measurements. Add `--diagnostics` to save the
sensor/pose recording. `--show-profile` prints the selection without starting
sensors. Existing mount checks and measured calibration export still apply.

| Setting | Selection |
| --- | --- |
| Core | `benchmark/build_orb_fixed_gaussian_packed` |
| Requested features | 600 |
| Pyramid | Scale 1.6, five levels |
| Local inertial BA cap | 12 |
| Healthy initialized keyframe interval | 0.25 s |
| OpenCV threads | 1 |
| Sensor/calibration handling | Current measured tower profile and delivery fixes |

The core includes the stereo baseline and disabled-loop queue fixes, exact
stereo patch costs, exact descriptor blur, packed vocabulary, motion-gated
initialization and preserved inertial origin. The launcher pins the tested
core checksum recorded in [the selection](../benchmark/orb_selected_profile_20261005.json).

## Evidence and limits

The confirmed two-minute tower run had no observed flights, jumps or resets;
maximum displacement was 72.4 cm. Both inertial refinements completed. Three
RECENTLY_LOST events recovered. Tracking averaged 11.67 ms, estimator plus ROS
launcher CPU averaged 80.8% of one core, and peak RSS was 518 MiB.
[Live evidence](../benchmark/tower_pyramid_validation_20261005.json).

On V1_02_medium, 600 versus 500 features with the same packed core used
52.49 versus 50.91 CPU seconds, a 3.1% increase. Late ATE decreased 22.5%
from 3.227 to 2.499 cm; full ATE decreased 24.0% from 8.314 to 6.321 cm.
The faster 500-feature rectification candidate has mixed trajectory results
and no matching live validation, so it is not the selected tower profile.

Desk bounds are not independent ground truth. General equality to OpenVINS
CPU while preserving trajectory quality remains unproven. These are the
limits of the selected result, and further experiments are concluded here.

## Reproducing the core

The recorded source options are:

```bash
python3 benchmark/build_orb_core.py \
  --output benchmark/build_orb_fixed_gaussian_packed \
  --preserve-inertial-origin --motion-gated-initialization \
  --keyframe-interval-s .25 --fast-stereo-patches --fast-gaussian \
  --packed-vocabulary
```

The installed ORB sources and build dependencies are required. A rebuilt
binary can have a different checksum; the selected launcher intentionally
accepts the tested binary. Update its fingerprint only after validating the
rebuilt core. Calibration must be repeated if the physical mount changes.
