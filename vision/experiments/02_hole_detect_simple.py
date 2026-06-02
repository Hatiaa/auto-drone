#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Step 2: simple hole / dark-region centroid (prototype).

Assumes the hole interior is darker than the surrounding wall (common for a physical opening).
Tune --thresh, --min-area, --max-area, and ROI via margin ratio if needed.
"""

import argparse
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np


def largest_dark_blob_centroid(
    gray: np.ndarray,
    thresh: int,
    invert_otsu: bool,
    min_area: float,
    max_area: float,
    margin: float,
) -> Tuple[Optional[Tuple[int, int]], np.ndarray]:
    h, w = gray.shape[:2]
    if invert_otsu:
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)

    bw = cv2.medianBlur(bw, 5)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel, iterations=2)

    x0 = int(w * margin)
    y0 = int(h * margin)
    x1 = int(w * (1.0 - margin))
    y1 = int(h * (1.0 - margin))
    mask_roi = np.zeros_like(bw)
    mask_roi[y0:y1, x0:x1] = 255
    bw = cv2.bitwise_and(bw, mask_roi)

    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0.0
    for c in contours:
        a = float(cv2.contourArea(c))
        if a < min_area or a > max_area:
            continue
        if a > best_area:
            best_area = a
            best = c

    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 128, 0), 2)

    center = None
    if best is not None:
        M = cv2.moments(best)
        if M["m00"] > 1e-6:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center = (cx, cy)
        cv2.drawContours(vis, [best], -1, (0, 255, 0), 2)
        if center is not None:
            cv2.drawMarker(vis, center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=40, thickness=2)

    return center, vis


def open_capture(camera_id: int):
    # OpenCV 3.2 (Jetson Nano default apt) does not accept (index, backend) signature.
    try:
        return cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    except TypeError:
        return cv2.VideoCapture(camera_id)


def main() -> int:
    p = argparse.ArgumentParser(description="Simple dark-hole centroid + optional MP4.")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fourcc", type=str, default="MJPG")
    p.add_argument("--thresh", type=int, default=120, help="Fixed threshold if not using Otsu.")
    p.add_argument("--otsu", action="store_true", help="Use Otsu instead of fixed --thresh.")
    p.add_argument("--min-area", type=float, default=5000.0)
    p.add_argument("--max-area", type=float, default=800000.0)
    p.add_argument("--margin", type=float, default=0.05, help="Ignore borders (fraction of W/H).")
    p.add_argument("--save", type=str, default="", help="Write annotated MP4.")
    p.add_argument("--show-fps", action="store_true", help="Print FPS about once per second.")
    args = p.parse_args()

    cap = open_capture(args.camera)
    if not cap.isOpened():
        print("ERROR: cannot open camera", args.camera, file=sys.stderr)
        return 1

    if len(args.fourcc) == 4:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_cam = cap.get(cv2.CAP_PROP_FPS)
    out_fps = fps_cam if fps_cam and fps_cam > 1 else 25.0
    print("resolution:", w, "x", h)

    writer = None
    if args.save:
        writer = cv2.VideoWriter(
            args.save, cv2.VideoWriter_fourcc(*"mp4v"), out_fps, (w, h)
        )
        if not writer.isOpened():
            print("ERROR: VideoWriter", args.save, file=sys.stderr)
            cap.release()
            return 1

    t_end = time.monotonic() + args.seconds
    t0 = time.monotonic()
    frames = 0
    t_last = t0
    last_center = None

    while time.monotonic() < t_end:
        ok, bgr = cap.read()
        if not ok or bgr is None:
            print("WARN: read failed")
            break
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        center, vis = largest_dark_blob_centroid(
            gray,
            thresh=args.thresh,
            invert_otsu=args.otsu,
            min_area=args.min_area,
            max_area=args.max_area,
            margin=args.margin,
        )
        last_center = center

        if center is not None:
            cv2.putText(
                vis,
                f"cx,cy={center[0]},{center[1]}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                vis,
                "no target",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        frames += 1
        now = time.monotonic()
        if args.show_fps and now - t_last >= 1.0:
            print("fps:", round(frames / max(now - t0, 1e-6), 2), "last_center:", last_center)
            t_last = now

        if writer is not None:
            writer.write(vis)

    cap.release()
    if writer is not None:
        writer.release()

    print("last_center:", last_center, "frames:", frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# --- Usage examples (commented out) ---
#
# 1. 先用 640x480 跑通 + 保存视频
#    /usr/bin/python3 02_hole_detect_simple.py --seconds 20 --width 640 --height 480 --fourcc MJPG --otsu --show-fps --save /tmp/hole_640.mp4
#
# 2. 尝试 720p（如果 FPS 够用再上）
#    /usr/bin/python3 02_hole_detect_simple.py --seconds 20 --width 1280 --height 720 --fourcc MJPG --otsu --show-fps --save /tmp/hole_720.mp4
#
# 3. 固定阈值（不用 Otsu）
#    /usr/bin/python3 02_hole_detect_simple.py --seconds 20 --width 640 --height 480 --fourcc MJPG --thresh 120 --show-fps --save /tmp/hole_thresh.mp4
