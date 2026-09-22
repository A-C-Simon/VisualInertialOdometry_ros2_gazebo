#!/usr/bin/env python3
"""Stereo calibrate ELP 3DGS1200P01 from pairs/left + pairs/right (1600x1200 each).
Default board: 9x6 inner, 25mm squares.

Usage:
  python3 stereo_calibrate.py [--pairs pairs] [--cols 9] [--rows 6] [--square 0.025] [--out calib]
Outputs in --out/:
  stereo_calibration.npz      (everything: K1,D1,K2,D2,R,T,E,F,Q, ROI, maps, rms, size)
  calibration_opencv.yaml     (OpenCV FileStorage)
  left.yaml / right.yaml      (ROS camera_info format)
  results.txt                 (human-readable: rms, baseline, f, reproj)
  rectified_example_*.png     (visual check, must show horizontal epipolar lines)
  q_matrix.txt                (Q for depth: Z = Q reprojection)

Requires >= ~15 good pairs, 25+ recommended.
"""
import cv2, numpy as np, os, glob, argparse, yaml

ap = argparse.ArgumentParser()
ap.add_argument("--pairs", default="pairs")
ap.add_argument("--cols", type=int, default=9)
ap.add_argument("--rows", type=int, default=6)
ap.add_argument("--square", type=float, default=0.025, help="square size in meters")
ap.add_argument("--out", default="calib")
ap.add_argument("--min-pairs", type=int, default=10)
args = ap.parse_args()

pattern = (args.cols, args.rows)
os.makedirs(args.out, exist_ok=True)

left_files = sorted(glob.glob(f"{args.pairs}/left/*.png") + glob.glob(f"{args.pairs}/left/*.jpg"))
right_files = sorted(glob.glob(f"{args.pairs}/right/*.png") + glob.glob(f"{args.pairs}/right/*.jpg"))
# also support full/ side-by-side fallback
if (not left_files) and os.path.isdir(f"{args.pairs}/full"):
    print("No left/right splits found, trying full/ side-by-side frames...")
    left_files, right_files = [], []
    os.makedirs(f"{args.pairs}/left", exist_ok=True); os.makedirs(f"{args.pairs}/right", exist_ok=True)
    for f in sorted(glob.glob(f"{args.pairs}/full/*")):
        img = cv2.imread(f)
        if img is None: continue
        h, w = img.shape[:2]; hw = w // 2
        ln = f"{args.pairs}/left/{os.path.splitext(os.path.basename(f))[0]}.png"
        rn = f"{args.pairs}/right/{os.path.splitext(os.path.basename(f))[0]}.png"
        cv2.imwrite(ln, img[:, :hw]); cv2.imwrite(rn, img[:, hw:])
        left_files.append(ln); right_files.append(rn)

assert len(left_files) == len(right_files) and len(left_files) > 0, \
    f"pair mismatch: {len(left_files)} left vs {len(right_files)} right in {args.pairs}/"
print(f"Found {len(left_files)} pairs.")

# 3D object points
objp = np.zeros((pattern[0]*pattern[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * args.square

obj_pts, img_ptsL, img_ptsR = [], [], []
img_size = None
good = 0
for lf, rf in zip(left_files, right_files):
    il, ir = cv2.imread(lf), cv2.imread(rf)
    if il is None or ir is None:
        print(f"skip unreadable {lf} {rf}"); continue
    gl = cv2.cvtColor(il, cv2.COLOR_BGR2GRAY); gr = cv2.cvtColor(ir, cv2.COLOR_BGR2GRAY)
    if img_size is None: img_size = gl.shape[::-1]
    assert gl.shape[::-1] == img_size and gr.shape[::-1] == img_size, "all images must share resolution!"
    # robust detection: SB first, then classic
    okL, cL = cv2.findChessboardCornersSB(gl, pattern)
    okR, cR = cv2.findChessboardCornersSB(gr, pattern)
    if not (okL and okR):
        f = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        if not okL: okL, cL = cv2.findChessboardCorners(gl, pattern, f)
        if not okR: okR, cR = cv2.findChessboardCorners(gr, pattern, f)
    if okL and okR:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-4)
        cv2.cornerSubPix(gl, cL, (11, 11), (-1, -1), term)
        cv2.cornerSubPix(gr, cR, (11, 11), (-1, -1), term)
        obj_pts.append(objp); img_ptsL.append(cL); img_ptsR.append(cR)
        good += 1
        print(f"OK {os.path.basename(lf)}")
    else:
        print(f"MISS L={okL} R={okR} {os.path.basename(lf)}")
print(f"Detected {good}/{len(left_files)}")
if good < args.min_pairs:
    raise SystemExit(f"Only {good} good pairs (< {args.min_pairs}). Capture more varied views.")

w, h = img_size
print(f"Calibrating at {w}x{h} per eye ...")
# --- mono inits ---
r1, K1, D1, rvecs1, tvecs1 = cv2.calibrateCamera(obj_pts, img_ptsL, img_size, None, None)
r2, K2, D2, rvecs2, tvecs2 = cv2.calibrateCamera(obj_pts, img_ptsR, img_size, None, None)
print(f"mono rms L={r1:.3f} R={r2:.3f}")
# --- stereo (fix intrinsics from mono: standard, stable) ---
flags = cv2.CALIB_FIX_INTRINSIC
rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
    obj_pts, img_ptsL, img_ptsR, K1, D1, K2, D2, img_size,
    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-5), flags=flags)
print(f"stereo rms={rms:.4f} px")
baseline = float(np.linalg.norm(T))
print(f"baseline={baseline*1000:.2f} mm  T={T.ravel()}")

# --- rectify ---
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T, alpha=0)
map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, cv2.CV_32FC1)

# per-view epipolar check: mean |y_L - y_R| on rectified points
ys = []
for o, pl, pr in zip(obj_pts, img_ptsL, img_ptsR):
    plr = cv2.undistortPoints(pl, K1, D1, R=R1, P=P1).reshape(-1, 2)
    prr = cv2.undistortPoints(pr, K2, D2, R=R2, P=P2).reshape(-1, 2)
    ys.append(np.abs(plr[:, 1] - prr[:, 1]).mean())
print(f"mean rectified |dy| = {np.mean(ys):.3f} px (want < 1px)")

# --- save npz ---
np.savez(f"{args.out}/stereo_calibration.npz", K1=K1, D1=D1, K2=K2, D2=D2,
         R=R, T=T, E=E, F=F, Q=Q, R1=R1, R2=R2, P1=P1, P2=P2,
         roi1=roi1, roi2=roi2, map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y,
         rms=rms, baseline=baseline, img_size=np.array(img_size), square=args.square,
         pattern=np.array(pattern))

# --- OpenCV yaml ---
fs = cv2.FileStorage(f"{args.out}/calibration_opencv.yaml", cv2.FILE_STORAGE_WRITE)
for k, v in [("K1", K1), ("D1", D1), ("K2", K2), ("D2", D2), ("R", R), ("T", T),
             ("E", E), ("F", F), ("Q", Q), ("R1", R1), ("R2", R2), ("P1", P1), ("P2", P2)]:
    fs.write(k, v)
fs.write("image_width", w); fs.write("image_height", h)
fs.write("rms", rms); fs.write("baseline", baseline)
fs.release()

# --- ROS camera_info yamls ---
def ros_yaml(path, K, D, Rr, P, w, h, name):
    d = {"image_width": w, "image_height": h, "camera_name": name,
         "camera_matrix": {"rows": 3, "cols": 3, "data": K.reshape(-1).tolist()},
         "distortion_model": "plumb_bob",
         "distortion_coefficients": {"rows": 1, "cols": len(D.ravel()), "data": D.ravel().tolist()},
         "rectification_matrix": {"rows": 3, "cols": 3, "data": Rr.reshape(-1).tolist()},
         "projection_matrix": {"rows": 3, "cols": 4, "data": P.reshape(-1).tolist()}}
    open(path, "w").write(yaml.safe_dump(d, sort_keys=False))
ros_yaml(f"{args.out}/left.yaml", K1, D1, R1, P1, w, h, "left")
ros_yaml(f"{args.out}/right.yaml", K2, D2, R2, P2, w, h, "right")

# --- results.txt ---
focal = (P1[0, 0] + P2[0, 0]) / 2
txt = f"""ELP 3DGS1200P01 stereo calibration
pairs used: {good}/{len(left_files)}  per-eye: {w}x{h}  board: {pattern[0]}x{pattern[1]} @ {args.square*1000:.1f}mm
stereo rms: {rms:.4f} px   mono rms L/R: {r1:.3f}/{r2:.3f}
mean rectified |dy|: {np.mean(ys):.3f} px (want <1.0)
baseline: {baseline*1000:.3f} mm
T (m): {T.ravel().tolist()}
f_rect (px): {focal:.1f}   depth Z = f*B/disparity ; Q saved in npz/yaml
ROIs: {roi1} {roi2}
files: stereo_calibration.npz, calibration_opencv.yaml, left.yaml, right.yaml
tip: reproject Q with cv2.reprojectImageTo3D(disparity, Q) after SGBM/BM.
"""
open(f"{args.out}/results.txt", "w").write(txt)
open(f"{args.out}/q_matrix.txt", "w").write(str(Q))
print(txt)

# --- rectified example (last good pair) ---
il = cv2.imread(left_files[-1]); ir = cv2.imread(right_files[-1])
rl = cv2.remap(il, map1x, map1y, cv2.INTER_LINEAR)
rr = cv2.remap(ir, map2x, map2y, cv2.INTER_LINEAR)
combo = np.hstack([rl, rr])
for y in range(0, h, h // 12):
    cv2.line(combo, (0, y), (combo.shape[1], y), (0, 255, 0), 1)
cv2.imwrite(f"{args.out}/rectified_example.png", cv2.resize(combo, (1600, 600)))
print(f"saved {args.out}/rectified_example.png - lines must be horizontal & aligned across L|R")
