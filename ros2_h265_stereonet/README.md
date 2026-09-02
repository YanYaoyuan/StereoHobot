# ros2_h265_stereonet

ROS2 节点：订阅 H.265 编码的左右相机视频流（`foxglove_msgs/CompressedVideo`），通过 FFmpeg
解码后送入 StereoNet BPU 模型完成双目深度估计，输出视差图、深度图、可视化图、点云等。

## 架构

```text
/image_left_raw/h265  ─┐
                       ├─► [H265Decoder] ─► BGR ─┐
                       │                         ├─► [Sync] ─► resize ─► NV12 ─► [StereoNet BPU] ─► disp/depth/pcd
                       │                         │
/image_right_raw/h265 ─┤                         │
                       └─► [H265Decoder] ─► BGR ─┘
```

- **H265Decoder**：基于 FFmpeg libavcodec 的有状态解码器，每个相机独立实例。
- **Sync**：按时间戳（`builtin_interfaces/Time`）对左右帧进行匹配，容差可配置（默认 10ms）。
- **StereoNet BPU**：复用父工程 `hobot_stereonet` 的 `StereonetProcess` 推理引擎。

## 依赖

| 依赖 | 说明 |
|------|------|
| ROS2 (humble / jazzy) | `rclcpp`, `sensor_msgs`, `std_msgs` |
| `foxglove_msgs` | 提供 `CompressedVideo` 消息类型；S100 部署包会从 vendored 源码一同构建 |
| FFmpeg (libavcodec, libavutil, libswscale) | H.265 解码 |
| OpenCV 4.x | 图像处理 |
| PCL | 点云（`stereonet_process.h` 编译需要） |
| DNN / UCP | S100/X5 BPU 推理库 |
| hobot_stereonet (父工程) | 共享 `include/` 和 `src/` 源码 |

### 安装 foxglove_msgs

```bash
# apt 安装（推荐）
sudo apt install ros-${ROS_DISTRO}-foxglove-msgs

# 或从源码构建
cd ~/ros2_ws/src
git clone https://github.com/foxglove/ros-foxglove-msgs.git
cd ~/ros2_ws && colcon build --packages-select foxglove_msgs
```

### 安装 FFmpeg 开发库

```bash
# 目标板上（aarch64）
sudo apt install libavcodec-dev libavutil-dev libswscale-dev

# 交叉编译时需要在 sysroot 中包含对应的 aarch64 版本
```

## 构建

### 在目标板上直接编译

```bash
# 确保 ros2 环境已 source
source /opt/ros/${ROS_DISTRO}/setup.bash

# 进入工作空间
cd ~/ros2_ws/src
ln -s /path/to/hobot_stereonet/ros2_h265_stereonet .

# 编译
cd ~/ros2_ws
colcon build --packages-select ros2_h265_stereonet \
  --cmake-args -DPLATFORM_S100=ON
```

### S100 本地 Docker 交叉编译

```bash
cd /home/user/vbot/StereoHobot
bash ros2_h265_stereonet/run_build_S100.sh
```

脚本使用 D-Robotics 官方 `pc_tros_ubuntu22.04:v1.0.0` 镜像、固定版本的
`robot_dev_config` 和 S100 sysroot，在 Docker 容器内执行 ROS2 Humble
交叉编译。宿主机上的 `/usr/bin/aarch64-linux-gnu-*` 不参与构建。

首次运行会下载并加载官方镜像；本机会自动复用
`/data/omni-s100-local/workspace/cache` 中已经验证过的 S100 sysroot（如果存在）。
可用 `S100_WORK_ROOT`、`S100_CACHE_ROOT`、`S100_IMAGE_ARCHIVE` 覆盖默认位置。

构建完成后得到：

```text
dist/StereoH265_ROS2_S100.tar.gz
└── StereoH265_ROS2_S100/
    ├── bin/       # ARM64 stereo_h265_node
    ├── lib/       # UCP、OpenCV、foxglove_msgs 运行库
    ├── config/    # ROS 参数及相机标定
    ├── launch/    # ROS2 launch
    ├── model/     # S100 HBM 模型
    ├── install/   # colcon overlay
    └── result/    # 可选推理结果
```

### GitHub 云端 CI

`.github/workflows/build-ros2-s100.yml` 在 push、PR 或手动触发时执行相同的
容器构建脚本，成功后上传 `StereoH265_ROS2_S100` Artifact。该流程使用公开的
官方镜像归档，不依赖 `D_ROBOTICS_USERNAME` 或 `D_ROBOTICS_PASSWORD`。

### 复制到 S100 板端

```bash
scp dist/StereoH265_ROS2_S100.tar.gz root@<board-ip>:/userdata/
ssh root@<board-ip>
cd /userdata
tar -xzf StereoH265_ROS2_S100.tar.gz
cd StereoH265_ROS2_S100

./check_environment.sh
./run_stereo_h265.sh
```

板端需使用与 RDK OS 匹配的 TROS Humble（通常为 `/opt/tros/humble`，也支持
`/opt/ros/humble`）。无需在板端再次编译。

## 运行

### 方式一：直接运行

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ros2_h265_stereonet stereo_h265_node \
  --ros-args \
  -p model_path:=./model/dstereo_s100_320_640_352_v2.4.hbm \
  -p left_topic:=/image_left_raw/h265 \
  -p right_topic:=/image_right_raw/h265 \
  -p fx:=380.0 -p fy:=380.0 -p cx:=320.0 -p cy:=240.0 \
  -p baseline:=0.12 \
  -p result_dir:=./result
```

### 方式二：使用 launch 文件

```bash
# 使用默认参数
ros2 launch ros2_h265_stereonet stereo_h265.launch.py

# 覆盖参数
ros2 launch ros2_h265_stereonet stereo_h265.launch.py \
  left_topic:=/image_left_raw/h265_half \
  right_topic:=/image_right_raw/h265_half \
  params_file:=/path/to/my_params.yaml
```

### 方式三：使用参数文件

修改 `config/stereo_h265_params.yaml` 中的参数后：

```bash
ros2 run ros2_h265_stereonet stereo_h265_node \
  --ros-args --params-file config/stereo_h265_params.yaml
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | launch 自动定位已安装的 S100 HBM | BPU 模型文件路径 |
| `post_version` | string | `auto` | 后处理版本 |
| `left_topic` | string | `/image_left_raw/h265` | 左相机 H.265 话题 |
| `right_topic` | string | `/image_right_raw/h265` | 右相机 H.265 话题 |
| `uncertainty_th` | double | `-0.10` | 不确定度阈值 |
| `sync_tolerance_ms` | double | `10.0` | 左右帧时间戳匹配容差（ms） |
| `max_buffer_size` | int | `10` | 每个相机最大缓冲帧数 |
| `intrinsic_file` | string | `""` | 相机内参文件路径（优先于直接参数） |
| `fx, fy, cx, cy` | double | `0.0` | 相机内参 |
| `baseline` | double | `0.0` | 基线距离（米） |
| `result_dir` | string | `./result` | 结果保存目录 |
| `save_results` | bool | `true` | 是否保存结果 |
| `save_freq` | int | `1` | 每 N 帧保存一次 |
| `max_frames` | int | `-1` | 最大处理帧数，-1 表示不限 |
| `save_visual` | bool | `true` | 保存可视化图 |
| `save_pcd` | bool | `true` | 保存点云 |
| `save_disp` | bool | `true` | 保存视差图 |
| `save_depth` | bool | `true` | 保存深度图 |
| `save_uncert` | bool | `false` | 保存不确定度图 |
| `save_epipolar` | bool | `false` | 保存极线对齐检查图 |

## 支持的 Topic 变体

除默认的 `/image_left_raw/h265` 和 `/image_right_raw/h265` 外，还可指定不同分辨率版本：

| Topic | 说明 |
|-------|------|
| `/image_left_raw/h265` | 全分辨率 |
| `/image_left_raw/h265_half` | 1/2 分辨率 |
| `/image_left_raw/h265_quarter` | 1/4 分辨率 |
| `/image_left_raw/h265_undistort` | 去畸变版本 |

右相机同理，将 `left` 替换为 `right`。

## 输出文件

结果保存在 `result_dir` 目录下，每帧以时间戳命名：

| 文件 | 说明 |
|------|------|
| `left_{ts}.png` | 左图（resize 后） |
| `right_{ts}.png` | 右图（resize 后） |
| `disp_{ts}.pfm` | 视差图（像素） |
| `depth_{ts}.png` | 深度图（mm） |
| `uncert_{ts}.pfm` | 不确定度图 |
| `visual_{ts}.png` | 深度伪彩色可视化 |
| `pointcloud_{ts}.pcd` | 彩色 3D 点云 |
| `epipolar_{ts}.png` | 极线对齐检查 |
| `camera_intrinsic.txt` | resize 后的相机内参 |

## 目录结构

```text
ros2_h265_stereonet/
├── CMakeLists.txt              # ament_cmake 构建配置
├── package.xml                 # ROS2 包描述
├── README.md                   # 本文档
├── include/
│   └── h265_decoder.h          # H.265 解码器头文件
├── src/
│   ├── stereo_h265_node.cpp    # ROS2 节点（含 main）
│   └── h265_decoder.cpp        # H.265 解码器实现
├── launch/
│   └── stereo_h265.launch.py   # Launch 文件
├── config/
│   └── stereo_h265_params.yaml # 默认参数配置
└── run_build_S100.sh           # 官方 TROS Docker 本地构建入口
```

  新增 Topic 发布功能
发布的 Topics
Topic	类型	说明
~/stereonet_depth	sensor_msgs/msg/Image	深度图 (mono16, 单位 mm)
~/stereonet_pointcloud2	sensor_msgs/msg/PointCloud2	彩色 3D 点云 (XYZRGB)
~/stereonet_visual	sensor_msgs/msg/Image	可视化图 (bgr8, 默认关闭)
~/stereonet_disp	sensor_msgs/msg/Image	视差图 (32FC1, 默认关闭)
新增参数
参数	默认值	说明
publish_depth	true	发布深度图 Topic
publish_pointcloud	true	发布点云 Topic
publish_visual	false	发布可视化图 Topic
publish_disp	false	发布视差图 Topic
depth_topic	~/stereonet_depth	深度图 topic 名称
pointcloud_topic	~/stereonet_pointcloud2	点云 topic 名称
frame_id	camera_link	所有发布消息的参考坐标系
pointcloud_downsample_step	2	点云降采样步长
pointcloud_depth_max	5.0	点云最大深度 (m)
关键实现
深度图: 直接将 CV_16UC1 (mm) 的深度矩阵打包为 sensor_msgs::msg::Image，encoding 为 mono16
点云: 使用 sensor_msgs::PointCloud2Iterator 逐像素反投影 (u,v,depth) → (x,y,z)，附带 RGB 颜色，支持降采样和最大深度截断
所有消息的 header.stamp 与原始 H.265 帧时间戳一致，frame_id 可配置
