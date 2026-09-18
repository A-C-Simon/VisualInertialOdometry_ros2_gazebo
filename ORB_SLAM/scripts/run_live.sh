#!/usr/bin/env bash
# Live stereo ORB-SLAM3 for ELP 3DGS1200P01 (side-by-side 3200x1200 on /dev/video0).
# Package root: /home/ac/ORB_SLAM
set -euo pipefail

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="$PKG_ROOT/config/ELP_1600x1200.yaml"
ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-$HOME/ORB_SLAM3}"
BIN="$ORB_SLAM3_DIR/Examples/Stereo/stereo_live_elp"
VOCAB="$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt"

DEVICE="${1:-0}"
TRAJ="${2:-live_session}"

if [[ ! -x "$BIN" ]]; then
  echo "ERROR: ORB-SLAM3 live binary not found at $BIN" >&2
  echo "Build ~/ORB_SLAM3 first (see README.md), or set ORB_SLAM3_DIR." >&2
  exit 1
fi
if [[ ! -f "$VOCAB" ]]; then
  echo "ERROR: vocabulary not found at $VOCAB" >&2
  exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "ERROR: config not found at $CONFIG" >&2
  exit 1
fi
if [[ ! -e "/dev/video${DEVICE}" ]]; then
  echo "ERROR: /dev/video${DEVICE} missing. Check USB camera connection." >&2
  ls -l /dev/video* >&2 || true
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:1
  echo "DISPLAY was unset, using DISPLAY=:1"
fi

echo "Config : $CONFIG"
echo "Binary : $BIN"
echo "Vocab  : $VOCAB"
echo "Camera : /dev/video${DEVICE} (expect 3200x1200 side-by-side MJPG)"
echo "Output : $PKG_ROOT/${TRAJ}.txt (trajectory), $PKG_ROOT/kf_${TRAJ}.txt (keyframes)"
echo "Stop with Ctrl+C. Move the camera slowly in a textured, well-lit room."
echo

cd "$PKG_ROOT"
exec "$BIN" "$VOCAB" "$CONFIG" "$DEVICE" "$TRAJ"
