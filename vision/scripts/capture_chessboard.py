import argparse
from pathlib import Path
import time

import cv2


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description="Capture chessboard images for camera calibration.")
    p.add_argument("--camera-id", type=int, default=0, help="Camera index, default=0")
    p.add_argument("--width", type=int, default=640, help="Capture width")
    p.add_argument("--height", type=int, default=480, help="Capture height")
    p.add_argument("--fps", type=int, default=25, help="Requested camera fps")
    p.add_argument("--inner-cols", type=int, default=11, help="Chessboard inner corners (columns)")
    p.add_argument("--inner-rows", type=int, default=8, help="Chessboard inner corners (rows)")
    p.add_argument(
        "--output-dir",
        type=str,
        default=str(root / "calib_images"),
        help="Directory to save calibration images",
    )
    return p.parse_args()


def main():
    args = parse_args()
    pattern_size = (args.inner_cols, args.inner_rows)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.camera_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera id={args.camera_id}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4)])

    print(f"[INFO] camera={args.camera_id} actual={actual_w}x{actual_h}@{actual_fps:.2f} fourcc={fourcc_str}")
    print(f"[INFO] pattern_size(inner corners)={pattern_size}")
    print(f"[INFO] output_dir={out_dir}")
    print("[INFO] Controls: SPACE=save current frame (only when corners found), q=quit")

    save_idx = 0
    last_hint_ts = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Empty frame, retrying...")
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        vis = frame.copy()
        if found:
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            cv2.putText(vis, "Corners: FOUND", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 2)
        else:
            cv2.putText(vis, "Corners: NOT FOUND", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.putText(vis, f"saved={save_idx}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("capture_chessboard", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            if found:
                filename = out_dir / f"chess_{save_idx:03d}.png"
                cv2.imwrite(str(filename), frame)
                print(f"[SAVE] {filename}")
                save_idx += 1
            else:
                now = time.time()
                if now - last_hint_ts > 1.0:
                    print("[HINT] Corners not found, move board / improve lighting and try again.")
                    last_hint_ts = now

    cap.release()
    cv2.destroyAllWindows()
    print(f"[DONE] total_saved={save_idx}")


if __name__ == "__main__":
    main()
