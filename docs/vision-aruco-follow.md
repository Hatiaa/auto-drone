# ArUco 视觉跟随

视觉主线使用 USB 相机识别 ArUco 标记，通过相机内参和布局文件估计目标在机体系中的位置，再由 `gate_align_hold.py` 向飞控发送机体系速度指令。当前稳定入口为 `vision/scripts/run_gate_follow.sh`，对应 `align_yz_yaw + --enable-x` 顺序状态机。

## 运行入口

```bash
cd auto-drone/vision
./scripts/run_gate_follow.sh
```

该脚本等价于：

```bash
/usr/bin/python3 scripts/gate_align_hold.py \
  --device /dev/ttyTHS1 \
  --baud 921600 \
  --layout config/gate_follow_static_example.yaml \
  --params camera_params.npz \
  --camera-id 0 \
  --width 640 \
  --height 480 \
  --mode align_yz_yaw \
  --pose-source live \
  --print-interval 0.3 \
  --enable-x \
  --target-x-m 2.5 \
  --kp-x 0.39 \
  --vx-max 0.195 \
  --x-tol 0.15 \
  --deadband-x 0.08 \
  --kp-y 0.585 \
  --vy-max 0.234 \
  --kp-yaw 0.35 \
  --yaw-rate-max-deg 4 \
  --slew-yaw-deg 5 \
  --yaw-tol-deg 12
```

## 状态机

`align_yz_yaw` 模式不会同时强行调整所有坐标。主线控制逻辑为：

```text
SEARCH: 等待 GUIDED、armed、CH6/CH7 high、高度满足、目标可见
ALIGN:  按 z -> yaw -> y 的顺序消除主要误差
HOLD:   目标稳定后保持，并在 --enable-x 打开时维持目标前后距离
```

`--enable-x` 打开后，程序会控制目标前后距离，使 ArUco/洞口保持在 `--target-x-m` 附近。该控制在顺序状态机内运行，避免 `hold_2m` 同时调整 x/y/z/yaw 带来的耦合问题。

## 布局配置

当前默认示例为 `config/gate_follow_static_example.yaml`：

```yaml
active_layout: single
dictionary: DICT_4X4_50
marker_length_m: 0.1465
single:
  reference_id: 0
  marker_to_gate_offset_m: [0.0, 0.0, 0.0]
  yaw_zero_offset_deg: 90.0
camera:
  mount_preset: forward_frd
  offset_body_m: [0.0, 0.0, 0.0]
  rpy_body_deg: [0.0, 0.0, 0.0]
```

关键字段含义：

```text
dictionary:              ArUco 字典，必须与打印标记一致
marker_length_m:          黑色正方形边长，必须用实物测量值
single.reference_id:      单码模式下使用的 ArUco ID
marker_to_gate_offset_m:  标记中心到洞口中心的三维偏移
yaw_zero_offset_deg:      标记安装方向导致的 yaw 固定偏置
camera.offset_body_m:     相机坐标原点相对机体坐标原点的位置
camera.rpy_body_deg:      相机安装相对机体的 roll/pitch/yaw 修正
```

更换相机、分辨率、打印尺寸、ArUco ID、相机安装位置或标记相对洞口的位置后，必须更新对应字段。

## 单独验证视觉

检测 ArUco：

```bash
/usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0 --width 640 --height 480 --dict DICT_4X4_50 --headless
```

估计目标位姿：

```bash
/usr/bin/python3 scripts/gate_pose.py \
  --layout config/gate_follow_static_example.yaml \
  --params camera_params.npz \
  --camera-id 0 \
  --width 640 \
  --height 480
```

正常输出应包含稳定变化的 `x_body_m`、`y_body_m`、`z_body_m` 和 `yaw_error_rad`。若相机能看到标记但位姿数值方向明显错误，应优先检查 `marker_length_m`、相机内参、`yaw_zero_offset_deg` 和相机安装外参。

## 与飞控联动

视觉跟随脚本在 live 模式下同样要求：

```text
mode == GUIDED
armed == True
CH6 > 1700
CH7 > 1700
range altitude >= min-altitude
```

目标丢失超过 `--lost-timeout` 后，脚本会回到搜索/零速度状态。遥控器切回非 GUIDED、CH6 拉低或 CH7 拉低时，自主控制门控关闭。

