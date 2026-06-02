# auto-drone

`auto-drone` 是一个基于 Pixhawk 2.4.8、ArduPilot、Jetson Nano 和 ArUco 视觉定位的四旋翼自主飞行项目。项目当前稳定公开工作流为：人工完成起飞并上升到安全高度，将飞行模式切换到 `GUIDED`，再由 Jetson Nano 通过 MAVLink 接管无人机，完成定高前飞或 ArUco 视觉跟随。

本仓库保留已实测通过的核心代码和复现文档，目标是让后续同学在当前系统基础上继续开发自动飞行、视觉导航和穿洞相关功能。

## 系统架构

```text
RC transmitter
    |
    | CH5: STABILIZE / ALTHOLD / GUIDED
    | CH6: autonomous enable signal
    | CH7: arm / disarm
    v
Pixhawk 2.4.8 + ArduPilot <---- TELEM2 MAVLink ----> Jetson Nano
    |                                                    |
    | Rangefinder / optical flow / attitude             | Python control scripts
    | motor outputs                                     | USB camera + ArUco pose
    v                                                    v
Quadrotor frame                                  Guided flight / gate following
```

## 当前能力

- 飞控通信：Jetson Nano 通过 `/dev/ttyTHS1` 和 Pixhawk TELEM2 建立 MAVLink 通信，波特率为 `921600`。
- 无桨测试：支持单电机测试、四电机顺序测试、四电机同时低油门测试。
- 遥控门控：CH5 切换飞行模式，CH6 作为程序启动信号，CH7 作为解锁/上锁信号。
- 自主前飞：人工起飞到安全高度后，Nano 在 `GUIDED` 模式下根据目标高度、距离和速度完成前飞并切入降落。
- 视觉跟随：USB 相机识别 ArUco 标记，估计洞口或标记相对机体系位姿，使用 `align_yz_yaw + --enable-x` 顺序状态机完成对齐和跟随。

## 仓库结构

```text
auto-drone/
  flight/
    src/                         # 飞控通信、自主前飞、无桨电机测试
    scripts/                     # 飞控侧一键运行脚本
  vision/
    scripts/                     # ArUco 检测、位姿估计、视觉跟随、标定工具
    config/                      # 洞口/标记布局配置
    experiments/                 # 早期视觉实验原型
    camera_params.npz            # 当前项目相机标定示例
  docs/                          # 复现、配置、排错和检查表
```

## 环境安装

完整环境：

```bash
cd auto-drone
python3 -m pip install -r requirements.txt
```

只安装飞控侧依赖：

```bash
python3 -m pip install -r requirements-flight.txt
```

只安装视觉侧依赖：

```bash
python3 -m pip install -r requirements-vision.txt
```

视觉代码依赖 `opencv-contrib-python`，因为 ArUco 模块位于 OpenCV contrib 包中。

## 快速开始

飞控通信检查：

```bash
cd auto-drone/flight
/usr/bin/python3 src/connect_pixhawk.py --system-address serial:///dev/ttyTHS1:921600
```

遥控器门控检查：

```bash
/usr/bin/python3 src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
```

无桨单电机测试：

```bash
/usr/bin/python3 src/motor_test.py --device /dev/ttyTHS1 --baud 921600 --motor 1 --throttle 12 --duration 2 --props-removed
```

自主前飞：

```bash
./scripts/run_guided_forward.sh 0.8 1.5 0.2
```

三个参数依次为目标高度、前飞距离和前飞速度。该流程要求已经人工起飞、切换到 `GUIDED`、CH6 拉高、CH7 拉高并处于解锁状态。

视觉跟随：

```bash
cd auto-drone/vision
./scripts/run_gate_follow.sh
```

该入口固化了本项目实测成功的 `align_yz_yaw + --enable-x` 参数。`hold_2m` 属于实验/基线模式，不作为主线运行方式。

## 文档

在线文档站：[https://thuautodronedoc.netlify.app](https://thuautodronedoc.netlify.app)

- [硬件接线](docs/hardware-wiring.md)
- [ArduPilot 参数配置](docs/ardupilot-params.md)
- [自主前飞流程](docs/flight-guided-forward.md)
- [ArUco 视觉跟随](docs/vision-aruco-follow.md)
- [相机与布局标定](docs/calibration.md)
- [检查表](docs/checklists.md)
- [排错指南](docs/troubleshooting.md)

## 文档站部署

仓库内置 VitePress 文档站配置，可直接部署到 Netlify。Netlify 构建配置为：

```text
Build command: npm run docs:build
Publish directory: docs/.vitepress/dist
```

本仓库不需要为文档单独拆分新仓库。文档与代码保持在同一仓库中，便于后续 PR 同时更新功能实现和对应说明。

## 安全边界

本仓库的飞行代码会向真实飞控发送控制指令。任何电机测试必须先拆除螺旋桨；任何上桨飞行必须保留遥控器接管能力，先完成通信、传感器、模式切换和无桨电机验证，再进入低高度、低速度、短距离测试。

## 后续开发

后续开发请先 fork 本仓库，不建议直接在原仓库上修改。新的自动飞行特性、视觉识别功能、控制策略、仿真流程、文档补充或代码重构，欢迎通过 Pull Request 提交，共同完善这个项目。

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=Hatiaa/auto-drone&type=Date)](https://star-history.com/#Hatiaa/auto-drone&Date)
