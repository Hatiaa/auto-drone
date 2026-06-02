"""GatePose: marker(s) -> gate center in body frame (FRD)."""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from aruco_pose import (
    build_transform,
    detect_markers,
    euler_xyz_to_matrix,
    get_aruco_dict,
    invert_transform,
    mount_preset_rotation,
    open_capture,
    rodrigues_to_matrix,
    rotation_matrix_to_rpy_deg,
)


def load_yaml(path):
    try:
        import yaml
    except ImportError as exc:
        raise ImportError("PyYAML required: pip install pyyaml") from exc
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class GatePose:
    def __init__(
        self,
        valid,
        x_body_m,
        y_body_m,
        z_body_m,
        yaw_error_rad,
        source,
        layout,
        markers_seen,
    ):
        self.valid = valid
        self.x_body_m = x_body_m
        self.y_body_m = y_body_m
        self.z_body_m = z_body_m
        self.yaw_error_rad = yaw_error_rad
        self.source = source
        self.layout = layout
        self.markers_seen = markers_seen

    def __str__(self):
        if not self.valid:
            return f"GatePose(invalid layout={self.layout} markers={self.markers_seen})"
        return (
            f"GatePose(valid layout={self.layout} src={self.source} "
            f"x={self.x_body_m:.3f} y={self.y_body_m:.3f} z={self.z_body_m:.3f} "
            f"yaw_err={math.degrees(self.yaw_error_rad):.1f}deg markers={self.markers_seen})"
        )


def corner_position_gate_m(corner_name, width_m, height_m, margin_m):
    hw = width_m * 0.5
    hh = height_m * 0.5
    m = margin_m
    positions = {
        "top_left": (-hw - m, hh + m, 0.0),
        "top_right": (hw + m, hh + m, 0.0),
        "bottom_right": (hw + m, -hh - m, 0.0),
        "bottom_left": (-hw - m, -hh - m, 0.0),
    }
    if corner_name not in positions:
        raise KeyError(corner_name)
    return np.array(positions[corner_name], dtype=np.float64)


def single_marker_gate_offset_m(cfg):
    offset = cfg.get("marker_to_gate_offset_m")
    if offset is not None:
        return np.asarray(offset, dtype=np.float64).reshape(3)

    distance = float(cfg.get("marker_to_gate_distance_m", 1.0))
    direction = str(cfg.get("marker_to_gate_direction", "marker_pos_y"))
    axis_map = {
        "marker_pos_x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "marker_neg_x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
        "marker_pos_y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "marker_neg_y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
        "marker_pos_z": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "marker_neg_z": np.array([0.0, 0.0, -1.0], dtype=np.float64),
    }
    if direction not in axis_map:
        raise ValueError(
            "single.marker_to_gate_direction must be one of: "
            + ", ".join(sorted(axis_map))
        )
    return axis_map[direction] * distance


def gate_center_from_marker(body_from_marker, marker_pos_gate_m):
    offset = -np.asarray(marker_pos_gate_m, dtype=np.float64).reshape(3)
    rotation = body_from_marker[:3, :3]
    translation = body_from_marker[:3, 3]
    return rotation.dot(offset) + translation


def yaw_error_from_body_gate(body_from_gate):
    rpy_deg = rotation_matrix_to_rpy_deg(body_from_gate[:3, :3])
    return math.radians(rpy_deg[2])


def wrap_angle_rad(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class ArucoGatePoseProvider:
    def __init__(self, layout_path, camera_matrix, dist_coeffs):
        self.layout = load_yaml(layout_path)
        self.layout_path = str(layout_path)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length_m = float(self.layout["marker_length_m"])
        self.active_layout = str(self.layout["active_layout"])
        self.aruco_dict = get_aruco_dict(self.layout["dictionary"])
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()
        cam = self.layout.get("camera", {})
        offset = cam.get("offset_body_m", [0.0, 0.0, 0.0])
        rpy = cam.get("rpy_body_deg", [0.0, 0.0, 0.0])
        preset = cam.get("mount_preset", "forward_frd")
        rot = euler_xyz_to_matrix(*rpy).dot(mount_preset_rotation(preset))
        self.body_from_camera = build_transform(rot, offset)
        self.camera_from_body = invert_transform(self.body_from_camera)

    def _apply_yaw_zero_offset(self, yaw_rad, section_name):
        section = self.layout.get(section_name, {})
        offset_deg = float(section.get("yaw_zero_offset_deg", 0.0))
        return wrap_angle_rad(yaw_rad - math.radians(offset_deg))

    def estimate(self, frame) -> GatePose:
        corners, ids, _ = detect_markers(frame, self.aruco_dict, self.detector_params)
        if ids is None or len(ids) == 0:
            return GatePose(False, 0.0, 0.0, 0.0, 0.0, "aruco", self.active_layout, 0)

        pose_result = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length_m, self.camera_matrix, self.dist_coeffs
        )
        if len(pose_result) == 3:
            rvecs, tvecs, _ = pose_result
        elif len(pose_result) == 2:
            rvecs, tvecs = pose_result
        else:
            raise RuntimeError(
                "Unexpected estimatePoseSingleMarkers return shape: "
                f"{len(pose_result)} values"
            )
        n = len(ids)
        if self.active_layout == "single":
            return self._estimate_single(ids, rvecs, tvecs, n)
        if self.active_layout == "four_corners":
            return self._estimate_four_corners(ids, rvecs, tvecs, n)
        raise ValueError(f"Unknown active_layout: {self.active_layout}")

    def _marker_body_pose(self, rvec, tvec):
        camera_from_marker = build_transform(rodrigues_to_matrix(rvec), tvec)
        # OpenCV pose gives marker -> camera. We need marker pose expressed in
        # body coordinates, i.e. body_from_marker = body_from_camera * camera_from_marker.
        return self.body_from_camera.dot(camera_from_marker)

    def _estimate_single(self, ids, rvecs, tvecs, n) -> GatePose:
        cfg = self.layout["single"]
        ref_id = int(cfg["reference_id"])
        offset = single_marker_gate_offset_m(cfg)
        matched = np.where(ids.flatten() == ref_id)[0]
        if len(matched) == 0:
            return GatePose(False, 0.0, 0.0, 0.0, 0.0, "aruco", self.active_layout, n)

        i = int(matched[0])
        body_from_marker = self._marker_body_pose(rvecs[i][0], tvecs[i][0])
        rotation = body_from_marker[:3, :3]
        translation = body_from_marker[:3, 3]
        gate_in_body = rotation.dot(offset) + translation
        marker_from_gate = build_transform(np.eye(3), offset)
        body_from_gate = body_from_marker.dot(marker_from_gate)
        yaw = self._apply_yaw_zero_offset(
            yaw_error_from_body_gate(body_from_gate), "single"
        )
        return GatePose(
            True,
            float(gate_in_body[0]),
            float(gate_in_body[1]),
            float(gate_in_body[2]),
            float(yaw),
            "aruco",
            "single",
            n,
        )

    def _estimate_four_corners(self, ids, rvecs, tvecs, n) -> GatePose:
        hole = self.layout["hole"]
        w = float(hole["width_m"])
        h = float(hole["height_m"])
        margin = float(hole["marker_margin_m"])
        id_map = self.layout["four_corners"]["ids"]
        name_by_id = {int(v): k for k, v in id_map.items()}

        estimates = []
        body_gates = []
        for i in range(n):
            mid = int(ids[i][0])
            if mid not in name_by_id:
                continue
            pos_gate = corner_position_gate_m(name_by_id[mid], w, h, margin)
            body_from_marker = self._marker_body_pose(rvecs[i][0], tvecs[i][0])
            estimates.append(gate_center_from_marker(body_from_marker, pos_gate))
            marker_from_gate = build_transform(np.eye(3), -pos_gate)
            body_gates.append(body_from_marker.dot(marker_from_gate))

        if not estimates:
            return GatePose(False, 0.0, 0.0, 0.0, 0.0, "aruco", self.active_layout, n)

        center = np.mean(estimates, axis=0)
        body_from_gate = body_gates[0] if len(body_gates) == 1 else body_gates[0]
        if len(body_gates) > 1:
            body_from_gate = np.mean(body_gates, axis=0)
            body_from_gate[:3, :3] = body_gates[0][:3, :3]
        yaw = self._apply_yaw_zero_offset(
            yaw_error_from_body_gate(body_from_gate), "four_corners"
        )
        return GatePose(
            True,
            float(center[0]),
            float(center[1]),
            float(center[2]),
            float(yaw),
            "aruco",
            "four_corners",
            n,
        )


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description="Estimate GatePose from ArUco (single or four_corners).")
    p.add_argument("--layout", type=str, default=str(root / "config" / "gate_layout.yaml"))
    p.add_argument("--params", type=str, default=str(root / "camera_params.npz"))
    p.add_argument("--camera-id", type=int, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--print-interval", type=float, default=0.5)
    p.add_argument("--headless", action="store_true", help="Run without cv2.imshow")
    return p.parse_args()


def main():
    args = parse_args()
    params_path = Path(args.params)
    if not params_path.exists():
        raise FileNotFoundError(f"Camera params not found: {params_path}")

    data = np.load(str(params_path))
    provider = ArucoGatePoseProvider(
        args.layout, data["camera_matrix"], data["dist_coeffs"]
    )
    print(f"[INFO] layout_file={args.layout} active_layout={provider.active_layout}")

    cap = open_capture(args.camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera id={args.camera_id}")

    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        pose = provider.estimate(frame)
        now = time.time()
        if now - last_print >= args.print_interval:
            print(pose)
            last_print = now
        if not args.headless:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
