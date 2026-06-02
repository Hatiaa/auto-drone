#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Step 1: stable capture loop, FPS stats, optional MP4 output (WSL / Linux)."""

import argparse
import sys
import time

import cv2


def open_capture(camera_id: int):
    # OpenCV 3.2 (Jetson Nano default apt) does not accept (index, backend) signature.
    try:
        return cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    except TypeError:
        return cv2.VideoCapture(camera_id)


def main() -> int:
    p = argparse.ArgumentParser(description="Camera capture benchmark + optional save.")
    p.add_argument("--camera", type=int, default=0, help="Video device index (usually 0).")
    p.add_argument("--seconds", type=float, default=30.0, help="Run duration in seconds.")
    p.add_argument("--width", type=int, default=0, help="Requested width (0 = leave default).")
    p.add_argument("--height", type=int, default=0, help="Requested height (0 = leave default).")
    p.add_argument(
        "--fourcc",
        type=str,
        default="MJPG",
        help="FourCC for capture preference, e.g. MJPG or YUYV.",
    )
    p.add_argument("--save", type=str, default="", help="If set, write annotated MP4 to this path.")
    args = p.parse_args()

    cap = open_capture(args.camera)
    if not cap.isOpened():
        print("ERROR: cannot open camera index", args.camera, file=sys.stderr)
        return 1

    if len(args.fourcc) == 4:
        fc = cv2.VideoWriter_fourcc(*args.fourcc)
        cap.set(cv2.CAP_PROP_FOURCC, fc)
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_cam = cap.get(cv2.CAP_PROP_FPS)
    print("resolution:", w, "x", h, "camera_fps_prop:", fps_cam)

    writer = None
    if args.save:
        out_fps = fps_cam if fps_cam and fps_cam > 1 else 30.0
        fourcc_out = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc_out, out_fps, (w, h))
        if not writer.isOpened():
            print("ERROR: cannot open VideoWriter for", args.save, file=sys.stderr)
            cap.release()
            return 1

    t_end = time.monotonic() + args.seconds
    t0 = time.monotonic()
    frames = 0
    t_last_report = t0

    while time.monotonic() < t_end:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("WARN: read failed")
            break
        frames += 1
        now = time.monotonic()
        if now - t_last_report >= 1.0:
            inst_fps = frames / (now - t0)
            print("elapsed_s:", round(now - t0, 2), "frames:", frames, "avg_fps:", round(inst_fps, 2))
            t_last_report = now

        if writer is not None:
            cv2.putText(
                frame,
                f"fps~{frames / max(1e-6, (now - t0)):.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            writer.write(frame)

    cap.release()
    if writer is not None:
        writer.release()

    dt = time.monotonic() - t0
    print("done. total_frames:", frames, "total_s:", round(dt, 3), "avg_fps:", round(frames / max(dt, 1e-6), 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# --- Usage examples (commented out) ---
#
# 1. 默认：跑 10 秒看 FPS
#    /usr/bin/python3 01_capture_loop.py --seconds 10
#
# 2. 指定 720p + MJPG（USB 摄像头常用）
#    /usr/bin/python3 01_capture_loop.py --seconds 10 --width 1280 --height 720 --fourcc MJPG
#
# 3. 保存 MP4 便于回看（建议写 /tmp）
#    /usr/bin/python3 01_capture_loop.py --seconds 10 --width 640 --height 480 --fourcc MJPG --save /tmp/capture_out.mp4
