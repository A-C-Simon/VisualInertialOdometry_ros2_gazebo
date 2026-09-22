#!/usr/bin/env python3
"""Checkerboard for screen calibration - WINDOWED by default so it can share
one monitor with the capture preview (fixes fullscreen-cover problem).

Board: 9x6 inner (10x7 squares). 1:1 pixels, no blur.
Measure displayed square with ruler (avg over 5), then:
  python3 stereo_calibrate.py --square <meters>   e.g. --square 0.0352

Usage:
  python3 show_checkerboard_screen.py [--sq-px 150] [--fullscreen 0]
Keys in window: f=toggle fullscreen  q/ESC=quit

Single-monitor layout:
  1. Run this script (small centered window appears).
  2. Drag it to the LEFT half of the screen.
  3. Run: python3 capture_stereo.py --preview-width 900
  4. Drag capture preview to RIGHT half. Both visible at once.
  5. Point camera at board, move camera/board for varied views.
"""
import cv2, numpy as np, argparse
ap = argparse.ArgumentParser()
ap.add_argument("--sq-px", type=int, default=150, help="square size in screen pixels")
ap.add_argument("--fullscreen", type=int, default=0, help="1=start fullscreen, 0=windowed")
ap.add_argument("--cols", type=int, default=9); ap.add_argument("--rows", type=int, default=6)
a = ap.parse_args()

NX, NY = a.cols + 1, a.rows + 1
board = np.ones((NY * a.sq_px, NX * a.sq_px), np.uint8) * 255
for y in range(NY):
    for x in range(NX):
        if (x + y) % 2 == 0:
            board[y*a.sq_px:(y+1)*a.sq_px, x*a.sq_px:(x+1)*a.sq_px] = 0
board = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
# margin + label so window is self-contained (no need for 2560 canvas)
M = 40
canvas = np.ones((board.shape[0] + M*2 + 30, board.shape[1] + M*2, 3), np.uint8) * 255
canvas[M+30:M+30+board.shape[0], M:M+board.shape[1]] = board
cv2.putText(canvas, f"9x6 inner SQ={a.sq_px}px  f=fullscreen q=quit",
            (M, M), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

print(f"Square = {a.sq_px}px. Est on 2560px=600mm screen: {a.sq_px*600/2560:.2f}mm - MEASURE WITH RULER!")
print("Windowed mode: drag this + capture preview side by side. Press 'f' for fullscreen.")
cv2.namedWindow("board", cv2.WINDOW_NORMAL)
cv2.imshow("board", canvas)
if a.fullscreen:
    cv2.setWindowProperty("board", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    fs = True
else:
    fs = False
while True:
    k = cv2.waitKey(30) & 0xFF
    if k in (27, ord('q')): break
    if k == ord('f'):
        fs = not fs
        cv2.setWindowProperty("board", cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN if fs else cv2.WINDOW_NORMAL)
cv2.destroyAllWindows()
