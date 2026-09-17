#!/usr/bin/env python3
"""Generate exact-scale 9x6 (inner) checkerboard, 25mm squares.
Outputs:
  checkerboard_9x6_25mm.png  (for screen viewing, 200px/square)
  checkerboard_9x6_25mm.pdf  (for printing at 100% scale, exact mm)
Inner corners: 9 cols x 6 rows => 10 x 7 squares.
Board size: 250mm x 175mm + 25mm white border all around.
IMPORTANT: print PDF at 100% / Actual size, NO fit-to-page. Verify with ruler.
"""
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A3, landscape
from reportlab.pdfgen import canvas
import cv2, numpy as np

COLS_INNER, ROWS_INNER = 9, 6
SQUARE_MM = 25
SQ_PX = 200  # png resolution

nx, ny = COLS_INNER + 1, ROWS_INNER + 1  # 10 x 7 squares

# --- PNG (screen / quick print check, NOT guaranteed scale) ---
img_h, img_w = ny * SQ_PX, nx * SQ_PX
board = np.ones((img_h, img_w), np.uint8) * 255
for y in range(ny):
    for x in range(nx):
        if (x + y) % 2 == 0:
            board[y*SQ_PX:(y+1)*SQ_PX, x*SQ_PX:(x+1)*SQ_PX] = 0
# white margin one square
margin = SQ_PX
board_m = np.ones((img_h + 2*margin, img_w + 2*margin), np.uint8) * 255
board_m[margin:margin+img_h, margin:margin+img_w] = board
cv2.imwrite("checkerboard_9x6_25mm.png", board_m)
print("wrote checkerboard_9x6_25mm.png", board_m.shape)

# --- PDF (exact scale) ---
pdf_path = "checkerboard_9x6_25mm.pdf"
border = SQUARE_MM * mm
bw, bh = nx * SQUARE_MM * mm, ny * SQUARE_MM * mm
page_w, page_h = landscape(A3)  # big enough for 300x225mm incl border
c = canvas.Canvas(pdf_path, pagesize=landscape(A3))
# center on page
ox = (page_w - bw) / 2
oy = (page_h - bh) / 2
# white background + border note
c.setFillColorRGB(1, 1, 1)
c.rect(0, 0, page_w, page_h, fill=1, stroke=0)
# squares (black where (x+y) even to match PNG)
c.setFillColorRGB(0, 0, 0)
for y in range(ny):
    for x in range(nx):
        if (x + y) % 2 == 0:
            # PDF origin bottom-left: flip y
            c.rect(ox + x*SQUARE_MM*mm, oy + (ny-1-y)*SQUARE_MM*mm,
                   SQUARE_MM*mm, SQUARE_MM*mm, fill=1, stroke=0)
c.setFillColorRGB(0, 0, 0)
c.setFont("Helvetica", 10)
c.drawString(20*mm, 12*mm,
    f"Checkerboard {COLS_INNER}x{ROWS_INNER} inner, square {SQUARE_MM}mm. Print at 100% (Actual size). Verify squares = {SQUARE_MM}mm.")
c.drawString(20*mm, 7*mm,
    "Board: 10x7 squares = 250x175mm. Glue flat, good light, avoid glare. Capture ~25 views both cameras.")
c.showPage(); c.save()
print(f"wrote {pdf_path} (A3 landscape, centered, exact mm). Print at 100%.")
