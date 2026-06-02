# ArduPilot 参数配置

本项目使用 Pixhawk 2.4.8 和 ArduPilot Copter 固件。参数设置完成后需要重启飞控，并在 QGroundControl 或 MAVLink Inspector 中确认数据已经正常输出。

> 不同版本 ArduPilot 参数名称和选项可能略有差异，实际配置请参考地面站实际显示的参数名称和选项。以下示例仅供参考。

## TELEM2 与 Nano MAVLink

TELEM2 对应 ArduPilot 的 `SERIAL2_*` 参数。Nano 通过 `/dev/ttyTHS1` 连接 TELEM2，当前稳定配置为：

```text
SERIAL2_PROTOCOL = 2        # MAVLink2
SERIAL2_BAUD     = 921      # 921600 baud
```

设置后重启飞控，在 Nano 上执行：

```bash
/usr/bin/python3 flight/src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
```

能持续打印 `mode`、`armed`、`CH5`、`CH6`、`CH7` 等信息即表示 TELEM2 通信正常。

## 遥控器通道

当前项目使用三个关键通道：

```text
CH5: 三挡飞行模式，低/中/高分别对应 STABILIZE / ALTHOLD / GUIDED
CH6: 程序启动信号，高电平表示允许 Nano 接管
CH7: Arm / Disarm
```

三个通道的 PWM 范围通常为 1000-2000，具体数值取决于遥控器和接收机的设置。默认低电平为 1000-1300，中电平为 1300-1700，高电平为 1700-2000。实际配置时请确认遥控器和接收机的 PWM 输出范围，并在程序中设置相应的门槛值。

CH6 不需要绑定 ArduPilot 功能，Nano 程序只读取其 PWM 值。程序中默认门槛为 `1700`，即 CH6 和 CH7 都高于 `1700` 且模式为 `GUIDED` 时才允许进入自主控制。

## TFmini 作为 Rangefinder

TFmini 串口版本接入空闲串口，例如 SERIAL4。典型参数为：

```text
SERIAL4_PROTOCOL = 9        # Rangefinder serial protocol
SERIAL4_BAUD     = 115      # 115200 baud

RNGFND1_TYPE     = Benewake Serial
RNGFND1_ORIENT   = 25       # Down
RNGFND1_MIN      = 0.10     # m，按实际传感器和安装情况设置
RNGFND1_MAX      = 10.0     # m，按实际可靠量程设置
RNGFND1_GNDCLEAR = 0.05     # m，传感器离地安装高度
```

不同 ArduPilot 版本中参数名可能显示为 `RNGFND1_MIN_CM`、`RNGFND1_MAX_CM` 或下拉选项文字。原则保持一致：类型选择 Benewake Serial，方向选择 Down，串口协议为 Lidar，串口波特率为 115200。

验证方式：

```text
QGC MAVLink Inspector:
  RANGEFINDER.distance 应随手动抬高/降低传感器稳定变化
  DISTANCE_SENSOR.current_distance 可作为辅助确认
```

飞控侧 `guided_forward_test.py` 使用 `RANGEFINDER` 消息作为高度数据源，不使用 `GLOBAL_POSITION_INT.relative_alt` 作为主高度。

## 光流与集成超声波高度计

光流模块参数取决于具体型号。配置原则为：先让飞控稳定输出光流数据，再把 EKF 水平速度来源切到 OpticalFlow。常见配置方向如下：

```text
FLOW_TYPE          = 按模块型号选择
FLOW_ORIENT_YAW    = 按安装朝向设置
EK2_SRC1_POSXY     = 0        # 无 GPS 室内环境不使用水平位置源
EK2_SRC1_VELXY     = 5        # OpticalFlow
EK2_SRC1_POSZ      = 1        # Baro 或按项目高度源设置
EK2_SRC1_YAW       = 1        # Compass
```

若光流模块集成超声波高度计，固件中可能通过 `RNGFND1_TYPE` 的模块类型输出高度。此时同样应在 MAVLink Inspector 中确认 `RANGEFINDER` 或 `DISTANCE_SENSOR` 数据稳定，再进入飞行测试。

## 飞行模式与安全检查

首次测试使用以下模式顺序：

```text
STABILIZE: 地面检查和手动控制
ALTHOLD:   手动定高验证高度计
GUIDED:    Nano 接管自主控制
LAND:      程序结束或异常时降落
```

进入自主程序前应满足：飞控无严重 PreArm 报错，电池电压正常，罗盘/姿态可用，Rangefinder 数据稳定，CH6/CH7 可被 Nano 读取，遥控器可以随时切回人工控制模式。

## 参考

- [ArduPilot Rangefinder Setup](https://ardupilot.org/copter/docs/common-rangefinder-setup.html)
- [ArduPilot Benewake TFmini Lidar](https://ardupilot.org/copter/docs/common-benewake-tfmini-lidar.html)
- [ArduPilot Optical Flow Sensor Setup](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)

