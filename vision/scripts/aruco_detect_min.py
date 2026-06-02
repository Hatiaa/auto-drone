import argparse
import time
import cv2


def parse_args():
    p = argparse.ArgumentParser(description='Minimal ArUco detector')
    p.add_argument('--camera-id', type=int, default=0, help='Camera index, default=0')
    p.add_argument('--dict', type=str, default='DICT_4X4_50', help='ArUco dictionary name')
    p.add_argument('--width', type=int, default=1280, help='Capture width')
    p.add_argument('--height', type=int, default=720, help='Capture height')
    p.add_argument('--max-frames', type=int, default=0, help='Stop after N frames (0 means infinite)')
    p.add_argument('--headless', action='store_true', help='Run without cv2.imshow')
    return p.parse_args()


def get_dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f'Unknown ArUco dictionary: {name}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_markers(frame, aruco_dict, detector_params):
    # OpenCV 4.7+ API
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
        return detector.detectMarkers(frame)
    # OpenCV <=4.6 API
    return cv2.aruco.detectMarkers(frame, aruco_dict, parameters=detector_params)


def open_capture(camera_id: int):
    # OpenCV 3.2 (Jetson Nano default apt) does not accept (index, backend) signature.
    try:
        return cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    except TypeError:
        return cv2.VideoCapture(camera_id)


def main():
    args = parse_args()

    aruco_dict = get_dictionary(args.dict)
    detector_params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()

    # Prefer V4L2 backend when supported; fallback for older OpenCV.
    cap = open_capture(args.camera_id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 25)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise RuntimeError(f'Cannot open camera id={args.camera_id}')

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = ''.join([chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4)])
    print(f'[INFO] camera={args.camera_id} backend=V4L2 req={args.width}x{args.height}@25 MJPG')
    print(f'[INFO] camera_actual={actual_w}x{actual_h}@{actual_fps:.2f} fourcc={fourcc_str}')

    frame_idx = 0
    last_print = 0.0

    print('[INFO] Press q to quit (or Ctrl+C in headless mode).')

    while True:
        ok, frame = cap.read()
        if not ok:
            print('[WARN] Empty frame, retrying...')
            time.sleep(0.05)
            continue

        corners, ids, rejected = detect_markers(frame, aruco_dict, detector_params)

        detected_ids = []
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            detected_ids = [int(x[0]) for x in ids]

        now = time.time()
        if now - last_print > 0.5:
            print(f'[FRAME {frame_idx}] markers={len(detected_ids)} ids={detected_ids} rejected={len(rejected)}')
            last_print = now

        if not args.headless:
            cv2.putText(
                frame,
                f'markers={len(detected_ids)} ids={detected_ids}',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow('aruco_detect_min', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        frame_idx += 1
        if args.max_frames > 0 and frame_idx >= args.max_frames:
            break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

# --- Usage examples (commented out) ---
#
# 1. Headless（SSH 里推荐），每 0.5s 打印一次检测到的 ids
#    /usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0 --width 640 --height 480 --dict DICT_4X4_50 --headless
#
# 2. 有桌面环境时开启窗口显示
#    /usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0 --width 1280 --height 720 --dict DICT_4X4_50
