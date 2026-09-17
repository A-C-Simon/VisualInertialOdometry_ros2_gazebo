# ELP 3DGS1200P01 stereo calibration

Synchronized dual-lens USB camera (`32e4:2b10`, `/dev/video0`) presenting one
side-by-side MJPG frame `3200x1200` = left `1600x1200` | right `1600x1200`
(OG02B10 global shutter).

## Contents

| File | Purpose |
|---|---|
| `screen_calibrate.py` | Recommended capture: fullscreen board + live preview PiP, one key handler |
| `capture_stereo.py` | Fallback capture-only script |
| `show_checkerboard_screen.py` | Fallback board-only display |
| `stereo_calibrate.py` | Mono + stereo calibration, rectification, ROS/OpenCV export |
| `stereo_depth_demo.py` | Live rectified + SGBM depth check |
| `generate_checkerboard.py` | Regenerates the printable target |
| `checkerboard_9x6_25mm.pdf/.png` | 9x6 inner, 25mm print target |
| `INSTRUCTIONS.txt` | Full run arguments and troubleshooting |
| `calib/` | Achieved calibration (see below) |

## Quick start

```bash
python3 screen_calibrate.py
# SPACE = save when L:OK R:OK, q = quit. Take ~25 varied poses.
python3 stereo_calibrate.py --square 0.035 --out calib
python3 stereo_depth_demo.py --calib calib/stereo_calibration.npz
```

Screen squares on the capture monitor measured 35.0mm — pass the measured
size in meters via `--square`. For print, use `--square 0.025` with the PDF
at 100% scale.

## Achieved calibration (2026-09-16, 26/26 pairs, 1600x1200/eye, 35.0mm screen board)

- stereo rms **0.2908 px** (mono 0.092 / 0.094), mean rectified `|dy|` **0.184 px**
- baseline **59.896 mm**, rectified focal **1082.8 px** (`Z = f*B/disparity`)
- `calib/`: `results.txt`, `left.yaml` / `right.yaml` (ROS camera_info),
  `calibration_opencv.yaml`, `q_matrix.txt`, `rectified_example.png`

Raw pairs (`pairs/`, ~119MB) and the 30MB `stereo_calibration.npz` (rectify
maps, regenerable with `stereo_calibrate.py`) are excluded from git.
