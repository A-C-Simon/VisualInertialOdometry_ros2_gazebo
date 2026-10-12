#!/usr/bin/env bash
# Selected October 5 tower operating point. This selects the already tested
# core and settings; it does not change the camera/IMU calibration.
set -euo pipefail
VIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE_DIR="$VIO_ROOT/benchmark/build_orb_fixed_gaussian_packed"
EXPECTED_CORE_SHA=49e8e0ef3b18bafc3924f689e27274a963519ddfbb2c2c883c2d23ffb643fbe9
for arg in "$@"; do
  case "$arg" in
    --help|-h)
      echo "Usage: $0 [--rviz] [--diagnostics] [--show-profile]"
      echo "Selected tower profile: 600 features, scale 1.6, five levels, local BA 12."
      echo "Other sensor/viewer options are passed to orbslam3_hw290_vio.sh."
      exit 0 ;;
    --show-profile)
      printf 'core=%s\ncore_sha256=%s\nfeatures=600\npyramid_scale=1.6\npyramid_levels=5\nlocal_ba_window=12\nopencv_threads=1\n' "$CORE_DIR" "$EXPECTED_CORE_SHA"
      exit 0 ;;
  esac
done
[[ -r "$CORE_DIR/libORB_SLAM3.so" ]] || {
  echo "Selected core is missing. See hw290_stereo/ORB_SELECTED_PROFILE.md." >&2
  exit 2
}
ACTUAL_CORE_SHA="$(sha256sum "$CORE_DIR/libORB_SLAM3.so")"
ACTUAL_CORE_SHA="${ACTUAL_CORE_SHA%% *}"
[[ "$ACTUAL_CORE_SHA" == "$EXPECTED_CORE_SHA" ]] || {
  echo "Selected core differs from the tested October 5 binary. See hw290_stereo/ORB_SELECTED_PROFILE.md." >&2
  exit 2
}
export ORB_CORE_DIR="$CORE_DIR"
export ORB_FEATURES=600 ORB_PYRAMID_SCALE=1.6 ORB_PYRAMID_LEVELS=5
export ORB_LOCAL_BA_WINDOW=12
exec "$VIO_ROOT/../ORB_SLAM/orbslam3_hw290_vio.sh" --efficient "$@"
