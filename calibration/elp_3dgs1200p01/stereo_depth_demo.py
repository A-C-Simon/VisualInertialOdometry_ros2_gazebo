#!/usr/bin/env python3
"""Live rectified preview + SGBM depth from ELP 3DGS1200P01.
Needs calib/stereo_calibration.npz from stereo_calibrate.py.
Usage: python3 stereo_depth_demo.py [--calib calib/stereo_calibration.npz] [--device 0]
Keys: q quit, s save snapshot.
"""
import cv2, numpy as np, argparse
ap = argparse.ArgumentParser()
ap.add_argument("--calib", default="calib/stereo_calibration.npz")
ap.add_argument("--device", type=int, default=0)
a = ap.parse_args()
C = np.load(a.calib)
map1x, map1y, map2x, map2y, Q = C["map1x"], C["map1y"], C["map2x"], C["map2y"], C["Q"]
W, H = int(C["img_size"][0])*2, int(C["img_size"][1])
print(f"loaded {a.calib} baseline={float(C['baseline'])*1000:.1f}mm rms={float(C['rms']):.3f}")

cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 30)
sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=128, blockSize=7,
    P1=8*3*7**2, P2=32*3*7**2, disp12MaxDiff=1, uniquenessRatio=10,
    speckleWindowSize=100, speckleRange=2, preFilterCap=63, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
while True:
    ret, frame = cap.read()
    if not ret: continue
    hw = frame.shape[1] // 2
    rl = cv2.remap(frame[:, :hw], map1x, map1y, cv2.INTER_LINEAR)
    rr = cv2.remap(frame[:, hw:], map2x, map2y, cv2.INTER_LINEAR)
    disp = sgbm.compute(cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY),
                        cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16.0
    valid = disp > 0
    vis = np.zeros_like(disp); vis[valid] = cv2.normalize(disp[valid], None, 0, 255, cv2.NORM_MINMAX).ravel()
    heat = cv2.applyColorMap(vis.astype(np.uint8), cv2.COLORMAP_JET)
    combo = np.hstack([cv2.resize(np.hstack([rl, rr]), (1280, 480)),
                       cv2.resize(heat, (1280, 480))])
    cv2.imshow("rectified L|R (top) + depth (bottom->right) [q quit, s save]", combo)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'): break
    if k == ord('s'):
        cv2.imwrite("depth_snapshot.png", combo); print("saved depth_snapshot.png")
cap.release(); cv2.destroyAllWindows()
