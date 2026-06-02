# auto-drone 文档

`auto-drone` 是一个基于 Pixhawk 2.4.8、ArduPilot、Jetson Nano 和 ArUco 视觉定位的四旋翼自主飞行项目。当前稳定工作流为：人工起飞到安全高度后切换到 `GUIDED`，由 Jetson Nano 通过 MAVLink 接管，完成自主前飞或 ArUco 视觉跟随。

## 快速入口

- [硬件接线](./hardware-wiring.md)：Pixhawk、Jetson Nano、TELEM2、TFmini、光流和相机连接方式。
- [ArduPilot 参数配置](./ardupilot-params.md)：串口、MAVLink、飞行模式、Rangefinder 和光流参数。
- [自主前飞流程](./flight-guided-forward.md)：飞控通信、RC 门控、定高前飞和降落流程。
- [ArUco 视觉跟随](./vision-aruco-follow.md)：视觉识别、位姿估计、顺序状态机和默认运行命令。
- [相机与布局标定](./calibration.md)：相机内参、ArUco 尺寸、相机外参和布局配置。
- [检查表](./checklists.md)：无桨测试、上桨前检查、首次飞行和视觉跟随检查。
- [排错指南](./troubleshooting.md)：通信、高度计、GUIDED、ArUco 和相机常见问题。

## 最小运行命令

飞控通信检查：

```bash
cd auto-drone/flight
/usr/bin/python3 src/connect_pixhawk.py --system-address serial:///dev/ttyTHS1:921600
```

自主前飞：

```bash
cd auto-drone/flight
./scripts/run_guided_forward.sh 0.8 1.5 0.2
```

视觉跟随：

```bash
cd auto-drone/vision
./scripts/run_gate_follow.sh
```

## 安全边界

本项目代码会向真实飞控发送控制指令。任何电机测试必须先拆除螺旋桨；任何上桨飞行必须保留遥控器接管能力，并先完成通信、传感器、模式切换和无桨验证。

