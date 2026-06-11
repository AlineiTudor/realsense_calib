#!/usr/bin/env python3
"""Generate a reference ChArUco board image matching the physical boards.

Usage:
    python3 generate_boards.py [--output_dir /tmp/charuco_boards]

The user already has two identical 4X4 7x5 boards (39.5mm square, 30mm marker).
This script produces a reference PNG for verification only.
"""

import argparse
import os

import cv2
import numpy as np

DICT_ID = cv2.aruco.DICT_4X4_50
SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH = 0.0395
MARKER_LENGTH = 0.030
DPI = 300
MARGIN_MM = 10


def main():
    parser = argparse.ArgumentParser(description='Generate reference ChArUco board image')
    parser.add_argument('--output_dir', default='/tmp/charuco_boards')
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict,
    )

    px_per_m = DPI / 0.0254
    w_px = int(SQUARES_X * SQUARE_LENGTH * px_per_m)
    h_px = int(SQUARES_Y * SQUARE_LENGTH * px_per_m)
    margin_px = int(MARGIN_MM / 1000.0 * px_per_m)

    board_img = board.generateImage((w_px, h_px))
    canvas = np.full((h_px + 2 * margin_px, w_px + 2 * margin_px), 255, dtype=np.uint8)
    canvas[margin_px:margin_px + h_px, margin_px:margin_px + w_px] = board_img

    path = os.path.join(args.output_dir, 'charuco_reference.png')
    cv2.imwrite(path, canvas)
    print(f'Saved {path}  ({canvas.shape[1]}x{canvas.shape[0]} px)')
    print(f'Board: {SQUARES_X}x{SQUARES_Y}, square={SQUARE_LENGTH*1000:.1f}mm, '
          f'marker={MARKER_LENGTH*1000:.1f}mm, dict=DICT_4X4_50')
    print('Print at 100% scale and verify square size with a ruler.')


if __name__ == '__main__':
    main()
