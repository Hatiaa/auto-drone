# experiments

本目录保存早期视觉原型工具，不作为当前主线飞行方案。

- `01_capture_loop.py`：摄像头采集、FPS 统计和视频保存验证。
- `02_hole_detect_simple.py`：基于暗色区域的洞口中心检测原型。

当前稳定视觉跟随方案使用 ArUco 位姿估计，主入口位于 `vision/scripts/gate_align_hold.py` 和 `vision/scripts/run_gate_follow.sh`。

