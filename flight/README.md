# flight

`flight/` 保存 Jetson Nano 与 Pixhawk 2.4.8 / ArduPilot 通信、自主前飞和无桨测试脚本。当前稳定工作流为手动起飞到安全高度后切换到 `GUIDED`，再由 Nano 通过 MAVLink 接管。

## 目录

- `src/connect_pixhawk.py`：MAVSDK 通信检查，确认 Nano 能连接飞控并读取基础遥测。
- `src/rc_mode_monitor.py`：读取飞行模式、解锁状态和 CH5/CH6/CH7，验证遥控器门控。
- `src/motor_test.py`：使用 ArduPilot motor test 指令逐个测试电机，必须无桨。
- `src/no_props_all_motors_test.py`：四个电机按顺序进行 20% / 10s 无桨测试。
- `src/no_props_simultaneous_motors_test.py`：四个电机同时 20% / 10s 无桨测试。
- `src/guided_forward_test.py`：手动起飞后 Nano 接管，按目标高度、距离、速度完成前飞并降落。
- `scripts/run_guided_forward.sh`：自主前飞的一键运行入口。

## 最小运行命令

```bash
cd flight
/usr/bin/python3 src/connect_pixhawk.py --system-address serial:///dev/ttyTHS1:921600
/usr/bin/python3 src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
/usr/bin/python3 src/motor_test.py --device /dev/ttyTHS1 --baud 921600 --motor 1 --throttle 12 --duration 2 --props-removed
./scripts/run_guided_forward.sh 0.8 1.5 0.2
```

`run_guided_forward.sh` 的三个位置参数依次为目标高度、前飞距离和前飞速度。

