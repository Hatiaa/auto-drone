import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description="Calibrate camera from chessboard images.")
    p.add_argument(
        "--images-dir",
        type=str,
        default=str(root / "calib_images"),
        help="Directory containing chessboard images",
    )
    p.add_argument("--glob", type=str, default="*.png", help="Image glob pattern")
    p.add_argument("--inner-cols", type=int, default=11, help="Chessboard inner corners (columns)")
    p.add_argument("--inner-rows", type=int, default=8, help="Chessboard inner corners (rows)")
    p.add_argument(
        "--square-size",
        type=float,
        required=True,
        help="Chessboard square size in meters, e.g. 0.024",
    )
    p.add_argument(
        "--output",
        type=str,
        default=str(root / "camera_params.npz"),
        help="Output npz path",
    )
    return p.parse_args()


def mean_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    total_error = 0.0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += err
    return total_error / max(1, len(objpoints))


def main():
    args = parse_args()
    pattern_size = (args.inner_cols, args.inner_rows)
    images_dir = Path(args.images_dir)
    image_paths = sorted(images_dir.glob(args.glob))

    if not image_paths:
        raise RuntimeError(f"No images found in {images_dir} with glob={args.glob}")

    objp = np.zeros((args.inner_rows * args.inner_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.inner_cols, 0 : args.inner_rows].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []
    imgpoints = []
    image_size = None

    # Termination criteria for corner refinement.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    used = 0
    for p in image_paths:
        img = cv2.imread(str(p))
        if img is None:
            print(f"[SKIP] cannot read {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not found:
            print(f"[SKIP] corners not found: {p.name}")
            continue

        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        image_size = gray.shape[::-1]
        used += 1
        print(f"[OK] {p.name}")

    if used < 10:
        raise RuntimeError(f"Only {used} valid images. Need at least 10 (recommended 20-30).")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    mean_err = mean_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(output_path),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=image_size[0],
        image_height=image_size[1],
        pattern_cols=args.inner_cols,
        pattern_rows=args.inner_rows,
        square_size=args.square_size,
        rms_error=rms,
        mean_reprojection_error=mean_err,
    )

    print("\n========== Calibration Result ==========")
    print(f"used_images: {used}/{len(image_paths)}")
    print(f"image_size: {image_size[0]}x{image_size[1]}")
    print(f"pattern_size(inner corners): {pattern_size}")
    print(f"square_size(m): {args.square_size}")
    print(f"rms_error: {rms:.6f}")
    print(f"mean_reprojection_error(px): {mean_err:.6f}")
    print("camera_matrix:")
    print(camera_matrix)
    print("dist_coeffs:")
    print(dist_coeffs.ravel())
    print(f"saved: {output_path}")


if __name__ == "__main__":
    main()
