# 自主前飞流程

自主前飞主脚本为 `flight/src/guided_forward_test.py`。该脚本使用 Nano 与 Pixhawk 的 MAVLink 串口连接，等待遥控器门控满足后，在 `GUIDED` 模式下控制无人机到达目标高度、按机头方向前飞指定距离，最后切换到 `LAND`。

## 控制入口

```bash
cd auto-drone/flight
./scripts/run_guided_forward.sh 0.8 1.5 0.2
```

三个位置参数依次为：

```text
0.8  -> target altitude, m
1.5  -> forward distance, m
0.2  -> forward speed, m/s
```

也可直接运行：

```bash
/usr/bin/python3 src/guided_forward_test.py --altitude 0.8 --distance 1.5 --speed 0.2
```

主脚本当前固定使用 `/dev/ttyTHS1` 和 `921600`。如硬件端口或波特率改变，需要同步修改脚本顶部的 `DEFAULT_DEVICE` 和 `DEFAULT_BAUD`。

## 启动前状态

进入程序前应完成：

```text
1. Pixhawk、Nano、相机和高度计供电正常
2. TELEM2 与 Nano 串口通信正常
3. Rangefinder 高度数据稳定
4. 遥控器 CH5 可切换 STABILIZE / ALTHOLD / GUIDED
5. CH6 可作为自主启动信号被 Nano 读取
6. CH7 可完成 arm / disarm
7. 飞机已人工起飞到低风险高度，并可随时手动接管
```

程序门控条件为：

```text
mode == GUIDED
CH6 > 1700
CH7 > 1700
armed == True
```

任一条件丢失时，脚本会中止自主控制并进入异常处理流程。

## 程序流程

```text
connect Pixhawk
request MAVLink streams
wait GUIDED + CH6 high + CH7 high
wait armed heartbeat
lock current yaw as forward reference
send takeoff / altitude target
actively control altitude using RANGEFINDER
wait LOCAL_POSITION_NED + ATTITUDE
hold briefly before forward motion
fly forward with body-frame velocity command
correct lateral crosstrack error
hold briefly after reaching target
switch to LAND
monitor landing telemetry
```

高度数据来自 MAVLink `RANGEFINDER.distance`，用于目标高度到达判断和飞行中的高度保持。水平前进距离来自 `LOCAL_POSITION_NED`，脚本记录进入前飞阶段时的位置和 yaw，把当前机头方向作为前进方向。

## 日志判读

通信正常时会看到：

```text
[connect] heartbeat received system=1 component=0 type=2 autopilot=3
```

门控满足时会看到：

```text
[gate] control allowed
[telemetry] mode=GUIDED armed=True CH6=2000 CH7=2000 range=...
```

高度控制阶段会看到：

```text
[takeoff] target=0.80m altitude=0.62m error=0.18m down_v=-0.11m/s
[takeoff] altitude target reached
```

前飞阶段会看到：

```text
[forward] target=1.50m progress=0.84m remaining=0.66m crosstrack=0.02m
[forward] position target reached
```

正常结束时会切换降落：

```text
[land] sending LAND mode
[land] monitoring for 12.0s
```

## 安全限制

脚本内置了姿态、高度误差、横向偏差、yaw 偏差和超时限制。出现异常时会打印 `[abort]` 并发送 `LAND`。这不替代人工接管；遥控器应始终保持可用。

