import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description="ArUco pose estimation using calibrated camera parameters.")
    p.add_argument("--camera-id", type=int, default=0, help="Camera index")
    p.add_argument("--width", type=int, default=640, help="Capture width")
    p.add_argument("--height", type=int, default=480, help="Capture height")
    p.add_argument("--fps", type=int, default=25, help="Requested FPS")
    p.add_argument("--dict", type=str, default="DICT_4X4_50", help="ArUco dictionary name")
    p.add_argument(
        "--params",
        type=str,
        default=str(root / "camera_params.npz"),
        help="Path to camera_params.npz",
    )
    p.add_argument("--marker-length", type=float, required=True, help="Marker side length in meters, e.g. 0.10")
    p.add_argument("--print-interval", type=float, default=0.5, help="Seconds between console logs")
    p.add_argument(
        "--reference-id",
        type=int,
        default=None,
        help="Marker id to use as the world origin for camera/body pose reporting. Default: first detected marker.",
    )
    p.add_argument(
        "--mount-preset",
        type=str,
        default="none",
        choices=["none", "forward_frd"],
        help=(
            "Camera-to-body axis preset. "
            "'forward_frd' assumes OpenCV camera axes (x right, y down, z forward) and body FRD axes "
            "(x forward, y right, z down)."
        ),
    )
    p.add_argument(
        "--camera-offset-body",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X_B", "Y_B", "Z_B"),
        help="Camera origin expressed in body/flight-controller frame, meters.",
    )
    p.add_argument(
        "--camera-rpy-body-deg",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("ROLL", "PITCH", "YAW"),
        help="Additional camera-to-body roll/pitch/yaw correction in degrees, applied on top of --mount-preset.",
    )
    return p.parse_args()


def get_aruco_dict(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_markers(frame, aruco_dict, detector_params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
        return detector.detectMarkers(frame)
    return cv2.aruco.detectMarkers(frame, aruco_dict, parameters=detector_params)


def open_capture(camera_id: int):
    # OpenCV 3.2 (Jetson Nano default apt) does not accept (index, backend) signature.
    try:
        return cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    except TypeError:
        return cv2.VideoCapture(camera_id)


def build_transform(rotation_matrix, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inv_transform = np.eye(4, dtype=np.float64)
    inv_transform[:3, :3] = rotation.T
    inv_transform[:3, 3] = -rotation.T.dot(translation)
    return inv_transform


def rodrigues_to_matrix(rvec):
    rotation_matrix, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return rotation_matrix


def euler_xyz_to_matrix(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz.dot(ry).dot(rx)


def rotation_matrix_to_rpy_deg(rotation_matrix):
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = 0.0

    return np.degrees([roll, pitch, yaw])


def mount_preset_rotation(name):
    if name == "none":
        return np.eye(3, dtype=np.float64)
    if name == "forward_frd":
        return np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )
    raise ValueError(f"Unknown mount preset: {name}")


def format_pose(label, transform):
    xyz = transform[:3, 3]
    roll_deg, pitch_deg, yaw_deg = rotation_matrix_to_rpy_deg(transform[:3, :3])
    return (
        f"{label} pos(m)=({xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f}) "
        f"rpy(deg)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f})"
    )


def main():
    args = parse_args()
    params_path = Path(args.params)
    if not params_path.exists():
        raise FileNotFoundError(f"Camera params not found: {params_path}")

    data = np.load(str(params_path))
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    aruco_dict = get_aruco_dict(args.dict)
    detector_params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") else cv2.aruco.DetectorParameters()

    cap = open_capture(args.camera_id)
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
    print(f"[INFO] dict={args.dict} marker_length={args.marker_length} m")
    print(f"[INFO] params={params_path}")
    print(f"[INFO] world_frame=marker_id:{args.reference_id if args.reference_id is not None else 'first_detected'}")
    print(f"[INFO] mount_preset={args.mount_preset} camera_offset_body(m)={tuple(args.camera_offset_body)}")
    print(f"[INFO] camera_rpy_body_deg={tuple(args.camera_rpy_body_deg)}")
    print("[INFO] Press q to quit")

    last_print = 0.0
    frame_idx = 0
    camera_to_body_rotation = euler_xyz_to_matrix(*args.camera_rpy_body_deg).dot(mount_preset_rotation(args.mount_preset))
    body_from_camera = build_transform(camera_to_body_rotation, args.camera_offset_body)
    camera_from_body = invert_transform(body_from_camera)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Empty frame, retrying...")
            time.sleep(0.05)
            continue

        corners, ids, rejected = detect_markers(frame, aruco_dict, detector_params)
        marker_count = 0

        if ids is not None and len(ids) > 0:
            marker_count = len(ids)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, args.marker_length, camera_matrix, dist_coeffs
            )

            for i in range(marker_count):
                marker_id = int(ids[i][0])
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, args.marker_length * 0.5)
                label = f"id={marker_id} x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}m"
                x0, y0 = corners[i][0][0].astype(int)
                cv2.putText(frame, label, (x0, max(20, y0 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            now = time.time()
            if now - last_print >= args.print_interval:
                selected_index = 0
                if args.reference_id is not None:
                    matched = np.where(ids.flatten() == args.reference_id)[0]
                    if len(matched) == 0:
                        print(
                            f"[FRAME {frame_idx}] markers={marker_count} "
                            f"reference_id={args.reference_id} not_detected rejected={len(rejected)}"
                        )
                        last_print = now
                        cv2.imshow("aruco_pose", frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("q"):
                            break
                        frame_idx += 1
                        continue
                    selected_index = int(matched[0])

                first_id = int(ids[selected_index][0])
                first_tvec = tvecs[selected_index][0]
                first_rvec = rvecs[selected_index][0]
                camera_from_marker = build_transform(rodrigues_to_matrix(first_rvec), first_tvec)
                marker_from_camera = invert_transform(camera_from_marker)
                body_from_marker = marker_from_camera.dot(camera_from_body)
                print(
                    f"[FRAME {frame_idx}] markers={marker_count} first_id={first_id} "
                    f"tvec(m)=({first_tvec[0]:.4f},{first_tvec[1]:.4f},{first_tvec[2]:.4f}) "
                    f"rvec=({first_rvec[0]:.4f},{first_rvec[1]:.4f},{first_rvec[2]:.4f}) rejected={len(rejected)}"
                )
                print(f"  {format_pose('camera_in_marker', camera_from_marker)}")
                print(f"  {format_pose('body_in_marker', body_from_marker)}")
                last_print = now
        else:
            now = time.time()
            if now - last_print >= args.print_interval:
                print(f"[FRAME {frame_idx}] markers=0 rejected={len(rejected)}")
                last_print = now

        cv2.imshow("aruco_pose", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

# --- Usage examples (commented out) ---
#
# 1. 在 Nano 上（建议先用 640x480），marker-length 单位是米
#    /usr/bin/python3 scripts/aruco_pose.py --camera-id 0 --width 640 --height 480 --fps 25 --dict DICT_4X4_50 --params ~/camera_params.npz --marker-length 0.10
