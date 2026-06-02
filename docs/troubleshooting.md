# 排错指南

## Nano 热点和 SSH

电脑连接 Nano 热点后，使用固定 IP 登录：

```bash
ssh jetson@192.168.1.100
```

密码为实验环境中设置的 Nano 用户密码。若无法连接，先确认电脑已经连接 Nano 热点，再检查 Nano 是否开机完成、热点是否自动启动、IP 是否仍为 `192.168.1.100`。

## TELEM2 没有 heartbeat

现象：

```text
[connect] waiting for heartbeat...
```

常见原因：

```text
TX/RX 没有交叉连接
GND 没有共地
Pixhawk 没有独立供电
SERIAL2_BAUD 与程序波特率不一致
SERIAL2_PROTOCOL 不是 MAVLink2
Jetson 串口不是 /dev/ttyTHS1
```

排查顺序：

```bash
ls -l /dev/ttyTHS1
/usr/bin/python3 flight/src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
```

若 USB 连接飞控正常而 TELEM2 不正常，优先检查 TELEM2 线序和 ArduPilot `SERIAL2_*` 参数。

## MAVProxy 报 adsb init

MAVProxy 某些版本会自动加载 `adsb` 模块并报：

```text
module 'adsb' has no attribute 'init'
```

使用最小模块启动：

```bash
python3 -m MAVProxy.mavproxy \
  --master=/dev/ttyTHS1 \
  --baudrate 921600 \
  --default-modules=log \
  --state-basedir=/home/$USER/mavproxy-test
```

如果只是验证本仓库代码，优先使用 `rc_mode_monitor.py` 或 `connect_pixhawk.py`。

## Rangefinder 无数据或高度跳变

检查 QGC MAVLink Inspector：

```text
RANGEFINDER.distance
DISTANCE_SENSOR.current_distance
```

若 `DISTANCE_SENSOR` 有数据但 `RANGEFINDER` 没有，检查 ArduPilot Rangefinder 类型、方向和串口协议。若距离偶尔变成极小值，检查 TFmini 是否被机架遮挡、供电是否稳定、地面反射是否异常、传感器方向是否朝下。

本项目自主前飞脚本使用 `RANGEFINDER` 作为主高度源。`GLOBAL_POSITION_INT.relative_alt` 可能来自 EKF/baro，不作为脚本高度控制的主数据源。

## GUIDED 不接管

运行：

```bash
cd auto-drone/flight
/usr/bin/python3 src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
```

确认输出同时满足：

```text
mode=GUIDED
armed=True
CH6=HIGH
CH7=HIGH
flight_ready=YES
```

若 `mode` 不变，检查 CH5 飞行模式设置。若 CH6/CH7 数值不变，检查遥控器通道映射。若 `armed=False`，检查飞控 PreArm 报错、电池、罗盘和安全开关。

## ArUco 检测不到

运行：

```bash
cd auto-drone/vision
/usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0 --width 640 --height 480 --dict DICT_4X4_50 --headless
```

常见原因：

```text
相机 ID 不对
OpenCV 没有安装 contrib 包
打印标记字典与配置不一致
标记过小、过远、反光或模糊
曝光过暗或运动模糊
```

若脚本报 `cv2.aruco` 不存在，重新安装：

```bash
python3 -m pip install -r requirements-vision.txt
```

## 位姿方向不对

先确认 `marker_length_m` 是打印后实测的黑色正方形边长。若距离尺度错误，通常是该字段或相机内参不匹配。若左右/上下方向反了，检查 `camera.mount_preset`、`camera.rpy_body_deg` 和相机实际安装方向。若 yaw 固定偏差明显，调整 `yaw_zero_offset_deg`。

## 相机打不开

检查设备：

```bash
ls /dev/video*
```

换用其他 ID：

```bash
/usr/bin/python3 scripts/aruco_detect_min.py --camera-id 1 --width 640 --height 480 --headless
```

若通过 SSH 运行且没有图形界面，添加 `--headless`。视觉跟随主脚本不依赖 `imshow`，但相机必须能被 OpenCV 打开。

