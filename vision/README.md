# vision

`vision/` 保存 ArUco 检测、相机标定、洞口布局建模和视觉跟随控制脚本。当前稳定主线为单 ArUco 标记下的 `align_yz_yaw + --enable-x` 顺序状态机：先调整高度方向和 yaw，再调整横向偏差，并在允许时控制前后距离。

## 目录

- `scripts/gate_align_hold.py`：视觉跟随主脚本，读取 ArUco 位姿并向飞控发送机体系速度控制。
- `scripts/gate_pose.py`：根据相机内参、ArUco 检测结果和布局配置计算洞口相对机体系位姿。
- `scripts/aruco_pose.py`：单 ArUco 位姿估计工具。
- `scripts/aruco_detect_min.py`：最小 ArUco 检测验证工具。
- `scripts/capture_chessboard.py`：棋盘格标定图片采集工具。
- `scripts/calibrate_camera.py`：相机内参标定工具。
- `scripts/generate_aruco_print.py`：ArUco 打印图生成工具。
- `scripts/run_gate_follow.sh`：已验证视觉跟随参数的一键运行入口。
- `config/gate_follow_static_example.yaml`：单码静态跟随示例布局。
- `camera_params.npz`：当前项目相机标定示例文件，更换相机或分辨率后需要重新生成。

## 最小运行命令

```bash
cd vision
/usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0
/usr/bin/python3 scripts/gate_pose.py --layout config/gate_follow_static_example.yaml --params camera_params.npz --camera-id 0 --width 640 --height 480
./scripts/run_gate_follow.sh
```

视觉跟随入口默认使用 `/dev/ttyTHS1`、`921600`、`640x480`、`DICT_4X4_50`、`marker_length_m = 0.1465` 和当前示例相机标定。

