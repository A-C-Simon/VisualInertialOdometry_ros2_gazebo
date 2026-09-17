#!/usr/bin/env python3
"""Capture synchronized stereo pairs from ELP 3DGS1200P01 (side-by-side MJPG).
Camera = ONE device /dev/video0, 3200x1200 frame = left(1600x1200) | right(1600x1200).

Usage:
  python3 capture_stereo.py [--out pairs] [--detect-every 4] [--detect-scale 0.5] [--no-detect]
Keys:
  SPACE - save pair (only if both detect, or 'f' to force)
  f     - force-save even without detection
  d     - toggle detection on/off (off = max fps)
  q/ESC - quit

Why v2 is fast: detection runs on half-res grays and only every Nth frame
(old version ran full-res detection on BOTH eyes EVERY frame: ~1.2s/frame).
Capture is still saved at FULL resolution regardless of preview/detection scale.
"""
import cv2, os, argparse, time, glob

ap = argparse.ArgumentParser()
ap.add_argument("--device", type=int, default=0)
ap.add_argument("--width", type=int, default=3200)
ap.add_argument("--height", type=int, default=1200)
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--out", default="pairs")
ap.add_argument("--cols", type=int, default=9)
ap.add_argument("--rows", type=int, default=6)
ap.add_argument("--detect-every", type=int, default=4, help="run detection every Nth frame")
ap.add_argument("--detect-scale", type=float, default=0.5, help="detection downscale (0.5 = 800x600/eye)")
ap.add_argument("--preview-width", type=int, default=1280, help="preview window width")
ap.add_argument("--no-detect", action="store_true", help="start with detection off")
args = ap.parse_args()

os.makedirs(f"{args.out}/left", exist_ok=True)
os.makedirs(f"{args.out}/right", exist_ok=True)
os.makedirs(f"{args.out}/full", exist_ok=True)

cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
cap.set(cv2.CAP_PROP_FPS, args.fps)
try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
except Exception: pass
time.sleep(0.5)
W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Opened {args.device}: {W}x{H} @ {cap.get(cv2.CAP_PROP_FPS):.1f}fps")
if W != args.width or H != args.height:
    print(f"WARNING: got {W}x{H}, expected {args.width}x{args.height}. Calibrate at runtime resolution!")
if not cap.isOpened():
    raise SystemExit("Cannot open camera.")

n = len(sorted(glob.glob(f"{args.out}/left/*.png")))
print(f"Existing pairs: {n}. Saving FULL-RES to {args.out}/left|right|full.")
print("Keys: SPACE=save-if-both-OK  f=force-save  d=toggle-detect  q=quit")

detect_on = not args.no_detect
okL = okR = False
frame_id = 0
t_fps = time.time(); fps = 0.0
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
cv2.namedWindow("stereo capture", cv2.WINDOW_KEEPRATIO)

while True:
    ret, frame = cap.read()
    if not ret:
        print("grab failed"); time.sleep(0.05); continue
    frame_id += 1
    h, w = frame.shape[:2]; hw = w // 2

    if detect_on and (frame_id % args.detect_every == 0):
        dw, dh = int(hw * args.detect_scale), int(h * args.detect_scale)
        gl = cv2.cvtColor(frame[:, :hw], cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(frame[:, hw:], cv2.COLOR_BGR2GRAY)
        if args.detect_scale != 1.0:
            gl = cv2.resize(gl, (dw, dh)); gr = cv2.resize(gr, (dw, dh))
        okL, _ = cv2.findChessboardCorners(gl, (args.cols, args.rows), flags)
        okR, _ = cv2.findChessboardCorners(gr, (args.cols, args.rows), flags)

    # fps meter
    if frame_id % 10 == 0:
        now = time.time(); fps = 10.0 / max(now - t_fps, 1e-6); t_fps = now

    pw = args.preview_width; ph = int(pw * h / w)
    prev = cv2.resize(frame, (pw, ph))
    cv2.line(prev, (pw // 2, 0), (pw // 2, ph), (0, 255, 0), 1)
    det = f"L:{'OK' if okL else '--'} R:{'OK' if okR else '--'}" if detect_on else "detect:OFF(d)"
    status = f"{det} saved:{n} {fps:.1f}fps | SPACE=save f=force d=detect q=quit"
    color = (0, 255, 0) if (okL and okR) else (0, 0, 255)
    cv2.putText(prev, status, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.imshow("stereo capture", prev)
    # small resize keeps window small so it can share screen with board
    try: cv2.resizeWindow("stereo capture", pw, ph)
    except Exception: pass

    k = cv2.waitKey(1) & 0xFF
    if k in (27, ord('q')): break
    if k == ord('d'):
        detect_on = not detect_on; print(f"detection {'ON' if detect_on else 'OFF'}")
    if k == ord(' ') or k == ord('f'):
        left, right = frame[:, :hw].copy(), frame[:, hw:].copy()
        both = bool(okL and okR)
        if k == ord(' ') and detect_on and not both:
            print("Rejected: board not in BOTH views. Press 'f' to force.")
            continue
        tag = f"{n:03d}"
        cv2.imwrite(f"{args.out}/left/left{tag}.png", left)
        cv2.imwrite(f"{args.out}/right/right{tag}.png", right)
        cv2.imwrite(f"{args.out}/full/full{tag}.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        n += 1
        print(f"saved pair {tag} both={both} ({W}x{H} full-res)")

cap.release(); cv2.destroyAllWindows()
print(f"Done. {n} pairs in {args.out}/")
