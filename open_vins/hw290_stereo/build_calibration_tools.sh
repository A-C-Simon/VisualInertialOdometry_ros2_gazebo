#!/usr/bin/env bash
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CAL_TOOLS="${CAL_TOOLS:-$HOME/vio_calibration_tools}"
KALIBR_REVISION=1f60227442d25e36365ef5f72cd80b9666d73467
KALIBR_IMAGE=kalibr-hw290:doc-20261009
ALLAN_IMAGE=kalibr-hw290:allan-20261009
JOBS=2
CHECK=false
DRY_RUN=false
while (($#)); do
  case "$1" in
    --jobs) (($# >= 2)) || { echo '--jobs requires a number' >&2; exit 2; }; JOBS=$2; shift ;;
    --check) CHECK=true ;;
    --dry-run) DRY_RUN=true ;;
    --help|-h) echo 'Usage: build_calibration_tools.sh [--jobs N] [--check | --dry-run]'; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs requires a positive integer' >&2; exit 2; }
[[ "$CHECK:$DRY_RUN" != true:true ]] || { echo 'Choose --check or --dry-run' >&2; exit 2; }
if [[ "$DRY_RUN" == true ]]; then
  echo "Fetch Kalibr $KALIBR_REVISION into $CAL_TOOLS; build with $JOBS jobs."
  echo "Build and smoke-check $KALIBR_IMAGE and $ALLAN_IMAGE."
  exit 0
fi
DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
  if [[ "$CHECK" == true ]]; then
    echo 'Cannot query Docker as this account. Retry with sudo or check the daemon.' >&2
    exit 1
  fi
  DOCKER=(sudo docker)
  "${DOCKER[@]}" info >/dev/null
fi
if [[ "$CHECK" == true ]]; then
  "${DOCKER[@]}" image inspect "$KALIBR_IMAGE" "$ALLAN_IMAGE" --format '{{.Id}}'
  exit
fi
source_dir="$CAL_TOOLS/kalibr-$KALIBR_REVISION"
mkdir -p "$CAL_TOOLS"
if [[ ! -d "$source_dir/.git" ]]; then
  git init "$source_dir"
  git -C "$source_dir" remote add origin https://github.com/ethz-asl/kalibr.git
fi
if ! git -C "$source_dir" rev-parse --verify HEAD >/dev/null 2>&1; then
  git -C "$source_dir" fetch --depth 1 origin "$KALIBR_REVISION"
  git -C "$source_dir" checkout --detach FETCH_HEAD
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" == "$KALIBR_REVISION" ]] || {
  echo "Unexpected source revision in $source_dir; preserve it and use a clean CAL_TOOLS directory." >&2; exit 1;
}
[[ -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]] || {
  echo "Tracked Kalibr sources changed in $source_dir; review before building." >&2; exit 1;
}
# Use a generated Dockerfile without altering the pinned upstream recipe.
sed 's/catkin build -j$(nproc)/catkin build -j'"$JOBS"'/' \
  "$source_dir/Dockerfile_ros1_20_04" > "$source_dir/Dockerfile.hw290"
"${DOCKER[@]}" build -t "$KALIBR_IMAGE" -f "$source_dir/Dockerfile.hw290" "$source_dir"
"${DOCKER[@]}" build -t "$ALLAN_IMAGE" --build-arg "BUILD_JOBS=$JOBS" \
  -f "$ROOT/hw290_stereo/calibration/Dockerfile.allan" "$ROOT/hw290_stereo/calibration/docker_context"
"${DOCKER[@]}" run --rm --entrypoint /bin/bash "$KALIBR_IMAGE" -lc \
  'set -e; source /catkin_ws/devel/setup.bash; rosrun kalibr kalibr_calibrate_cameras --help >/dev/null; rosrun kalibr kalibr_calibrate_imu_camera --help >/dev/null'
"${DOCKER[@]}" run --rm --entrypoint /bin/bash "$ALLAN_IMAGE" -lc \
  'set -e; source /allan_ws/devel/setup.bash; test -x /allan_ws/devel/lib/allan_variance_ros/allan_variance; rosrun allan_variance_ros analysis.py --help >/dev/null'
"${DOCKER[@]}" image inspect "$KALIBR_IMAGE" "$ALLAN_IMAGE" --format '{{.Id}}' \
  > "$CAL_TOOLS/calibration_image_ids.txt"
echo "Offline calibration tools ready. Image IDs: $CAL_TOOLS/calibration_image_ids.txt"
