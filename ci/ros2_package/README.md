# StereoH265 ROS2 S100 部署包

该包订阅左右 `foxglove_msgs/msg/CompressedVideo` H.265 话题，在 RDK S100
BPU 上运行 StereoNet，并发布深度图、彩色深度图、视差图和点云。

## 目录

- `bin/`：ARM64 ROS2 节点
- `lib/`：D-Robotics UCP、OpenCV、foxglove_msgs 运行库
- `config/`：节点参数和双目标定
- `launch/`：ROS2 launch 文件
- `model/`：S100 HBM 模型
- `install/`：可迁移的 colcon overlay
- `result/`：可选的落盘结果

## 板端运行

```bash
tar -xzf StereoH265_ROS2_S100.tar.gz -C /userdata
cd /userdata/StereoH265_ROS2_S100

./check_environment.sh
./run_stereo_h265.sh
```

覆盖机器狗话题：

```bash
LEFT_TOPIC=/dog/camera/left/h265 \
RIGHT_TOPIC=/dog/camera/right/h265 \
./run_stereo_h265.sh
```

使用自己的标定文件：

```bash
CALIB_YAML_FILE=/userdata/calibration/stereo.yaml ./run_stereo_h265.sh
```

目标板需安装与 RDK OS 匹配的 TROS Humble 基础环境，通常位于
`/opt/tros/humble`。部署包自带 `foxglove_msgs/CompressedVideo` 的 ARM64
类型支持，不需要在板端重新编译本项目。
