# 相机与布局标定

视觉跟随的精度主要由三类标定决定：相机内参、ArUco 实物尺寸、相机到机体坐标变换。更换相机、镜头、分辨率、打印尺寸、安装位置或洞口布局后，应重新完成对应标定。

## 相机内参

采集棋盘格图片：

```bash
cd auto-drone/vision
/usr/bin/python3 scripts/capture_chessboard.py \
  --camera-id 0 \
  --width 640 \
  --height 480 \
  --inner-cols 11 \
  --inner-rows 8 \
  --output-dir calib_images
```

按空格保存能检测到角点的图片。推荐保存 20 到 30 张，覆盖画面中心、四角、远近和不同角度。

计算内参：

```bash
/usr/bin/python3 scripts/calibrate_camera.py \
  --images-dir calib_images \
  --inner-cols 11 \
  --inner-rows 8 \
  --square-size 0.024 \
  --output camera_params.npz
```

`--square-size` 为棋盘每个方格的实际边长，单位为米。输出文件中包含 `camera_matrix`、`dist_coeffs`、图像尺寸和重投影误差。当前仓库保留的 `camera_params.npz` 只作为本项目相机示例，不应直接套用到其他相机或其他分辨率。

## ArUco 打印

生成打印图：

```bash
/usr/bin/python3 scripts/generate_aruco_print.py \
  --dict DICT_4X4_50 \
  --ids 0 1 2 3 4 \
  --marker-length-mm 146.5 \
  --margin-mm 20 \
  --dpi 300
```

打印时使用 100% 原始比例，不使用“适应页面”。打印后用尺子测量黑色正方形边长，并把测量值写入布局文件：

```yaml
marker_length_m: 0.1465
```

该数值必须是黑色正方形的边长，不是整张白边图片的边长。

## 布局文件

单码布局示例：

```yaml
active_layout: single
dictionary: DICT_4X4_50
marker_length_m: 0.1465
single:
  reference_id: 0
  marker_to_gate_offset_m: [0.0, 0.0, 0.0]
  yaw_zero_offset_deg: 90.0
```

若 ArUco 标记中心就是目标中心，`marker_to_gate_offset_m` 保持 `[0.0, 0.0, 0.0]`。若标记贴在洞口旁边或下方，需要测量标记中心到洞口中心的偏移并写入该字段。

四角布局示例：

```yaml
active_layout: four_corners
four_corners:
  ids:
    top_left: 0
    top_right: 1
    bottom_right: 2
    bottom_left: 3
```

四角模式要求四个标记 ID、洞口宽高和标记到洞口边缘的距离与实物一致。

## 相机到机体坐标变换

配置字段：

```yaml
camera:
  mount_preset: forward_frd
  offset_body_m: [0.0, 0.0, 0.0]
  rpy_body_deg: [0.0, 0.0, 0.0]
```

本项目采用机体系 FRD：

```text
x: 机头向前
y: 机体右侧
z: 机体向下
```

`offset_body_m` 表示相机光心相对机体参考点的偏移。`rpy_body_deg` 表示相机实际安装角度相对理想安装角度的修正。相机前视安装时通常使用 `mount_preset: forward_frd`。

标定流程：

```text
1. 固定相机安装位置
2. 测量相机光心到机体中心的 x/y/z 偏移
3. 保持无人机正对 ArUco，观察 yaw 输出偏差
4. 用 yaw_zero_offset_deg 修正标记方向偏差
5. 若 y/z 方向符号异常，检查相机安装方向和 rpy_body_deg
```

完成后运行 `gate_pose.py`，在不同距离和左右上下偏移下确认坐标方向与实际一致。

