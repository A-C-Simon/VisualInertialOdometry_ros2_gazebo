#!/usr/bin/env python3
"""Combined screen-board + capture (single process, single fullscreen window).

Fixes the two-script conflict: board goes FULLSCREEN, live stereo preview is
composited INSIDE the same image at top-right (outside the squares), so:
  - no window covers another, no drag/Alt-Tab needed
  - ONE window has focus => keys ALWAYS work, no terminal clicking

Layout (2560x1440): top bar 300px holds preview PiP (top-right) + status
(top-left); 9x6 board (10x7 squares) centered below. PiP never overlaps squares.

Keys (work when this window focused - no terminal clicks):
  SPACE - save pair if board found in BOTH eyes
  f     - force-save even without detection
  d     - toggle detection on/off (off = max fps)
  p     - toggle preview PiP on/off (for a clean board photo if needed)
  q/ESC - quit
Preview shows corner lines when found (scrolling view may lag 1-2 frames
behind since detection runs every --detect-every frames).

All old flags still available:
  board: --sq-px --cols --rows --screen-w --screen-h --pip-width
  camera: --device --width --height --fps --out --detect-every --detect-scale

After: measure displayed square with ruler (avg 5), then:
  python3 stereo_calibrate.py --square <meters>  e.g. --square 0.0352
"""
import cv2, numpy as np, os, argparse, time, glob

ap = argparse.ArgumentParser()
# board
ap.add_argument("--cols", type=int, default=9); ap.add_argument("--rows", type=int, default=6)
ap.add_argument("--sq-px", type=int, default=150)
ap.add_argument("--screen-w", type=int, default=2560); ap.add_argument("--screen-h", type=int, default=1440)
ap.add_argument("--top-bar", type=int, default=300, help="reserved top strip for preview")
ap.add_argument("--pip-width", type=int, default=640, help="preview PiP width (height auto 8:3)")
ap.add_argument("--no-pip", action="store_true", help="start with preview hidden")
# camera
ap.add_argument("--device", type=int, default=0)
ap.add_argument("--width", type=int, default=3200); ap.add_argument("--height", type=int, default=1200)
ap.add_argument("--fps", type=int, default=30); ap.add_argument("--out", default="pairs")
ap.add_argument("--detect-every", type=int, default=4)
ap.add_argument("--detect-scale", type=float, default=0.5)
ap.add_argument("--no-detect", action="store_true")
ap.add_argument("--smoke", type=int, default=0, help="hidden test: run N frames then exit")
a = ap.parse_args()

NX, NY = a.cols + 1, a.rows + 1
SW, SH, TOP = a.screen_w, a.screen_h, a.top_bar

# --- static board canvas at native screen res (1:1 squares, no blur) ---
base = np.ones((SH, SW, 3), np.uint8) * 255
bw, bh = NX * a.sq_px, NY * a.sq_px
assert bw < SW - 40 and bh < SH - TOP - 40, \
    f"board {bw}x{bh} too big for {SW}x{SH-TOP}. Lower --sq-px."
ox = (SW - bw) // 2
oy = TOP + (SH - TOP - bh) // 2
sq = np.ones((a.sq_px, a.sq_px, 3), np.uint8) * 255
blk = np.zeros((a.sq_px, a.sq_px, 3), np.uint8)
for y in range(NY):
    for x in range(NX):
        base[oy+y*a.sq_px:oy+(y+1)*a.sq_px, ox+x*a.sq_px:ox+(x+1)*a.sq_px] = \
            blk if (x + y) % 2 == 0 else sq
cv2.rectangle(base, (ox, oy), (ox + bw, oy + bh), (100, 100, 100), 2)
est_mm = a.sq_px * 600.0 / 2560.0
cv2.putText(base, f"9x{a.cols}x{a.rows} inner  SQ={a.sq_px}px (~{est_mm:.1f}mm, MEASURE!)",
            (ox, oy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

# --- camera ---
os.makedirs(f"{a.out}/left", exist_ok=True)
os.makedirs(f"{a.out}/right", exist_ok=True)
os.makedirs(f"{a.out}/full", exist_ok=True)
cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
cap.set(cv2.CAP_PROP_FPS, a.fps)
try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
except Exception: pass
time.sleep(0.5)
W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
if not cap.isOpened(): raise SystemExit("Cannot open camera.")
n = len(glob.glob(f"{a.out}/left/*.png"))
print(f"Cam {W}x{H}. Board {bw}x{bh}@{ox},{oy} sq={a.sq_px}px (~{est_mm:.2f}mm MEASURE WITH RULER). Pairs: {n}")
print("Keys: SPACE=save f=force d=detect p=preview q=quit (window focused, no terminal clicks)")

detect_on, show_pip = not a.no_detect, not a.no_pip
okL = okR = False
cL = cR = None
det_w = det_h = 0
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
pip_w = a.pip_width; pip_h = int(pip_w * a.height / a.width)
px = SW - pip_w - 20; py = 20
assert py + pip_h < TOP - 10, f"PiP {pip_w}x{pip_h} overflows top bar {TOP}. Lower --pip-width."
fid = 0; t0 = time.time(); fps = 0.0

cv2.namedWindow("calib", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("calib", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while True:
    ret, frame = cap.read()
    if not ret: time.sleep(0.05); continue
    fid += 1
    h, w = frame.shape[:2]; hw = w // 2
    if detect_on and fid % a.detect_every == 0:
        dw, dh = int(hw * a.detect_scale), int(h * a.detect_scale)
        gl = cv2.cvtColor(frame[:, :hw], cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(frame[:, hw:], cv2.COLOR_BGR2GRAY)
        if a.detect_scale != 1.0:
            gl = cv2.resize(gl, (dw, dh)); gr = cv2.resize(gr, (dw, dh))
        okL, cL = cv2.findChessboardCorners(gl, (a.cols, a.rows), flags)
        okR, cR = cv2.findChessboardCorners(gr, (a.cols, a.rows), flags)
        det_w, det_h = dw, dh
    if fid % 10 == 0:
        fps = 10.0 / max(time.time() - t0, 1e-6); t0 = time.time()

    img = base.copy()
    # status top-left
    det = f"L:{'OK' if okL else '--'} R:{'OK' if okR else '--'}" if detect_on else "detect:OFF"
    col = (0, 170, 0) if (okL and okR) else (0, 0, 255)
    cv2.putText(img, f"{det} saved:{n} {fps:.1f}fps", (20, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2)
    cv2.putText(img, "SPACE=save f=force d=detect p=preview q=quit", (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2)
    cv2.putText(img, f"sq~{est_mm:.1f}mm MEASURE! -> --square 0.0XXX", (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2)
    # PiP top-right (never over squares: confined to top bar)
    if show_pip:
        pip = cv2.resize(frame, (pip_w, pip_h))
        # overlay detected corners as lines (scaled from detection res to PiP halves)
        if detect_on and det_w > 0 and cL is not None and cR is not None:
            try:
                hw_pip = pip_w // 2
                sx, sy = hw_pip / det_w, pip_h / det_h
                s = np.array([sx, sy], np.float32)
                cLp = (cL.reshape(-1, 2) * s).reshape(-1, 1, 2)
                cRp = (cR.reshape(-1, 2) * s).reshape(-1, 1, 2)
                pipL = pip[:, :hw_pip]
                pipR = pip[:, hw_pip:]
                cv2.drawChessboardCorners(pipL, (a.cols, a.rows), cLp, bool(okL))
                cv2.drawChessboardCorners(pipR, (a.cols, a.rows), cRp, bool(okR))
            except Exception:
                pass
        cv2.line(pip, (pip_w // 2, 0), (pip_w // 2, pip_h), (0, 255, 0), 1)
        img[py:py + pip_h, px:px + pip_w] = pip
        cv2.rectangle(img, (px, py), (px + pip_w, py + pip_h), (0, 150, 0), 2)
    else:
        cv2.putText(img, "[preview hidden: press p]", (px, py + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)
    cv2.imshow("calib", img)
    k = cv2.waitKey(1) & 0xFF
    if a.smoke and fid >= a.smoke: print(f"smoke OK ({fid} frames)"); break
    if k in (27, ord('q')): break
    elif k == ord('d'):
        detect_on = not detect_on; print("detect", "ON" if detect_on else "OFF")
    elif k == ord('p'):
        show_pip = not show_pip
    elif k == ord(' ') or k == ord('f'):
        both = bool(okL and okR)
        if k == ord(' ') and detect_on and not both:
            print("Rejected: need BOTH OK (or 'f')."); continue
        left, right = frame[:, :hw].copy(), frame[:, hw:].copy()
        cv2.imwrite(f"{a.out}/left/left{n:03d}.png", left)
        cv2.imwrite(f"{a.out}/right/right{n:03d}.png", right)
        cv2.imwrite(f"{a.out}/full/full{n:03d}.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"saved pair {n:03d} both={both}"); n += 1

cap.release(); cv2.destroyAllWindows()
print(f"Done. {n} pairs. Measure squares -> python3 stereo_calibrate.py --square <meters>")
