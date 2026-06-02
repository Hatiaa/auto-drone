# 硬件接线

本项目硬件由 Pixhawk 2.4.8、Jetson Nano、TFmini/Rangefinder、光流/超声波高度模块、USB 相机、四旋翼机架、电机和电池组成。Pixhawk 负责姿态、电机输出和安全检查，Jetson Nano 负责运行 Python 程序并通过 MAVLink 发送自主控制指令。

## Pixhawk TELEM2 与 Jetson Nano UART

Pixhawk 2.4.8 的 TELEM2 是 6 针 JST-GH 串口，不适合直接插普通杜邦线。连接 Jetson Nano 时应使用 TELEM2 转杜邦线或自制 JST-GH 线束，只连接 `TX`、`RX`、`GND`，不要用 TELEM2 给 Nano 供电。

```text
Jetson Nano physical pin 8  / UART TX  ---> Pixhawk TELEM2 RX
Jetson Nano physical pin 10 / UART RX  <--- Pixhawk TELEM2 TX
Jetson Nano GND                         --- Pixhawk TELEM2 GND
```

串口通信必须交叉连接：Nano 的 TX 接飞控 RX，Nano 的 RX 接飞控 TX。地线必须共地。Pixhawk 需要由电池、电调供电板或 USB 独立上电；只接 TX/RX/GND 时飞控不会从 Nano 获得电源。

Jetson Nano 上对应串口通常为 `/dev/ttyTHS1`。本项目飞控侧主脚本默认使用 `/dev/ttyTHS1` 和 `921600`。

## TFmini / Rangefinder

TFmini 串口版本接入 Pixhawk 的空闲串口，例如 SERIAL4/5 口。典型连接如下：

```text
TFmini 5V   ---> Pixhawk 5V
TFmini GND  --- Pixhawk GND
TFmini TX   ---> Pixhawk serial RX
TFmini RX   <--- Pixhawk serial TX
```

TFmini 用作向下高度计时，传感器必须牢固朝下安装，视场内避免桨叶、机架、绑带或起落架遮挡。飞控和 Nano 程序读取的是 ArduPilot 发布的 `RANGEFINDER` MAVLink 数据，稳定性取决于 TFmini 安装、供电和参数配置。

## 光流 / 超声波高度模块

光流模块用于室内无 GPS 条件下辅助水平速度/位置估计。部分光流模块集成超声波高度计，固件中可能显示为对应的 Rangefinder 类型。当前仓库的自主前飞脚本不直接依赖光流完成水平定位，后续扩展室内悬停、Loiter 或更稳定的路径控制时需要接入光流并完成 EKF 参数配置。

光流模块安装时应保持镜头朝下，固定牢靠，离地面纹理充足，避免纯白地面、强反光地面和昏暗环境。

## USB 相机

视觉跟随使用普通 USB 摄像头，连接 Jetson Nano 后通常识别为 `/dev/video0`，OpenCV 中对应 `--camera-id 0`。当前示例标定文件对应 `640x480` 使用场景。更换相机、镜头、分辨率或安装位置后，应重新生成 `camera_params.npz` 并更新布局文件中的相机外参。

## 供电

Pixhawk、电机和电调由飞行电池供电。Jetson Nano 可由独立电源或稳定降压模块供电，不应依赖 Pixhawk TELEM 口供电。所有参与串口通信的设备必须共地。上桨飞行前应确认电池电压、电调方向、电机编号和机架方向。

