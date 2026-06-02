#!/usr/bin/env python3
"""Generate print-ready ArUco PNGs (DICT_4X4_50) with correct physical black square size."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="Generate ArUco markers for printing.")
    p.add_argument("--dict", type=str, default="DICT_4X4_50")
    p.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    p.add_argument("--marker-length-mm", type=float, default=100.0, help="Black square side length (mm)")
    p.add_argument("--margin-mm", type=float, default=20.0, help="Extra white border around marker (mm)")
    p.add_argument("--dpi", type=int, default=300)
    p.add_argument(
        "--out-dir",
        type=str,
        default="print/aruco_DICT_4X4_50",
        help="Output directory relative to the vision module root",
    )
    return p.parse_args()


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def get_dict(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_module_count(dict_name: str) -> int:
    # DICT_4X4_* -> 4x4 inner pattern
    if "_4X4_" in dict_name.upper():
        return 4
    if "_5X5_" in dict_name.upper():
        return 5
    if "_6X6_" in dict_name.upper():
        return 6
    raise ValueError(f"Cannot infer module count from {dict_name}")


def generate_marker_image(aruco_dict, marker_id: int, side_pixels: int, border_bits: int = 1) -> np.ndarray:
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(aruco_dict, marker_id, side_pixels, borderBits=border_bits)
    return cv2.aruco.drawMarker(aruco_dict, marker_id, side_pixels, borderBits=border_bits)


def build_print_page(
    aruco_dict,
    dict_name: str,
    marker_id: int,
    marker_length_mm: float,
    margin_mm: float,
    dpi: int,
) -> np.ndarray:
    border_bits = 1
    modules = marker_module_count(dict_name)
    black_ratio = (modules + 2 * border_bits) / (modules + 4 * border_bits)

    black_px = mm_to_px(marker_length_mm, dpi)
    side_pixels = max(64, int(round(black_px / black_ratio)))
    marker = generate_marker_image(aruco_dict, marker_id, side_pixels, border_bits)

    margin_px = mm_to_px(margin_mm, dpi)
    h, w = marker.shape[:2]
    canvas = np.full((h + 2 * margin_px, w + 2 * margin_px), 255, dtype=np.uint8)
    canvas[margin_px : margin_px + h, margin_px : margin_px + w] = marker

    label = f"ID {marker_id}  |  {dict_name}  |  black {marker_length_mm:.0f} mm"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.45, min(1.0, w / 900.0))
    thickness = max(1, int(round(scale * 2)))
    (tw, th), _ = cv2.getTextSize(label, font, scale, thickness)
    pad = mm_to_px(4, dpi)
    bar_h = th + 2 * pad
    out = np.full((canvas.shape[0] + bar_h, canvas.shape[1]), 255, dtype=np.uint8)
    out[: canvas.shape[0], :] = canvas
    x = max(0, (out.shape[1] - tw) // 2)
    y = canvas.shape[0] + pad + th
    cv2.putText(out, label, (x, y), font, scale, 0, thickness, cv2.LINE_AA)
    return out


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    out_dir = (root / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    aruco_dict = get_dict(args.dict)
    paths = []
    for marker_id in args.ids:
        img = build_print_page(
            aruco_dict,
            args.dict,
            marker_id,
            args.marker_length_mm,
            args.margin_mm,
            args.dpi,
        )
        name = f"aruco_{args.dict}_id{marker_id:02d}_{int(args.marker_length_mm)}mm.png"
        path = out_dir / name
        cv2.imwrite(str(path), img)
        paths.append(path)

    readme = out_dir / "README_PRINT.txt"
    readme.write_text(
        "\n".join(
            [
                "ArUco print pack (project default)",
                f"Dictionary: {args.dict}",
                f"Marker IDs: {', '.join(str(i) for i in args.ids)}",
                f"Black square side: {args.marker_length_mm:.1f} mm (must match gate_layout.yaml marker_length_m)",
                f"Extra white margin: {args.margin_mm:.1f} mm",
                f"Rendered at: {args.dpi} DPI",
                "",
                "Print settings (preferred):",
                "  - Scale: 100% / Actual size (do NOT fit to page)",
                "  - Paper: matte photo paper or plain paper on cardboard",
                "",
                "If printer cannot force 100% scale:",
                "  - Measure BLACK square after print (mm)",
                "  - gate_layout.yaml marker_length_m = measured_mm / 1000",
                "  - Update vision/config/*.yaml marker_length_m to measured_mm / 1000",
                "",
                "ID usage (gate_layout.yaml):",
                "  ID 0: V1 single reference + four_corners top_left",
                "  ID 1-3: four_corners top_right, bottom_right, bottom_left",
                "  ID 4: spare / second test gate",
                "",
                "Files:",
                *[f"  - {p.name}" for p in paths],
            ]
        ),
        encoding="utf-8",
    )
    print(f"[OK] wrote {len(paths)} markers to {out_dir}")
    for p in paths:
        print(f"  {p}")


if __name__ == "__main__":
    main()
