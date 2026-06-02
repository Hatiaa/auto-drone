# 检查表

## 无桨电机测试

- [ ] 螺旋桨已全部拆除
- [ ] Pixhawk 和 Nano 供电正常
- [ ] TELEM2 串口通信正常
- [ ] 电池电压正常
- [ ] 电机编号与机架方向已确认
- [ ] 急停和断电方式明确

单电机测试：

```bash
cd auto-drone/flight
/usr/bin/python3 src/motor_test.py --device /dev/ttyTHS1 --baud 921600 --motor 1 --throttle 12 --duration 2 --props-removed
```

四电机顺序测试：

```bash
/usr/bin/python3 src/no_props_all_motors_test.py --device /dev/ttyTHS1 --baud 921600 --props-removed
```

四电机同时测试：

```bash
/usr/bin/python3 src/no_props_simultaneous_motors_test.py --device /dev/ttyTHS1 --baud 921600 --props-removed
```

## 上桨前检查

- [ ] 电机方向与桨叶方向匹配
- [ ] 所有螺丝、相机、高度计、光流模块固定牢靠
- [ ] TFmini / Rangefinder 数据稳定
- [ ] 遥控器可切换 STABILIZE / ALTHOLD / GUIDED
- [ ] CH6 高低状态可被 Nano 读取
- [ ] CH7 可解锁和上锁
- [ ] 飞控无严重 PreArm 报错
- [ ] 测试区域无人靠近
- [ ] 遥控器操作者全程持控

## 自主前飞检查

- [ ] 已完成手动起飞和低高度悬停
- [ ] ALTHOLD 高度保持稳定
- [ ] 切换 GUIDED 后飞机无异常动作
- [ ] CH6 拉高前程序保持等待
- [ ] CH6 拉高后程序进入 control allowed
- [ ] 日志中 range 高度稳定
- [ ] 首次参数使用低高度、短距离、低速度

推荐首次参数：

```bash
./scripts/run_guided_forward.sh 0.5 0.5 0.15
```

## 视觉跟随检查

- [ ] 相机可以打开并稳定输出画面
- [ ] ArUco 字典与打印标记一致
- [ ] marker_length_m 与打印实测边长一致
- [ ] camera_params.npz 对应当前相机和分辨率
- [ ] gate_pose.py 输出方向与实际移动方向一致
- [ ] GUIDED、armed、CH6/CH7 门控正常
- [ ] 首次视觉跟随使用低速度和足够安全距离

视觉检测：

```bash
cd auto-drone/vision
/usr/bin/python3 scripts/aruco_detect_min.py --camera-id 0 --width 640 --height 480 --dict DICT_4X4_50 --headless
```

视觉跟随：

```bash
./scripts/run_gate_follow.sh
```

## 紧急接管

1. 遥控器切回 STABILIZE 或 ALTHOLD
2. 拉低 CH6，关闭 Nano 自主控制门控
3. 必要时 CH7 上锁或切 LAND
4. 断开程序后检查飞控模式、armed 状态和电机输出
