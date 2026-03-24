#ifndef DEPTH_NODE_HPP_
#define DEPTH_NODE_HPP_

#include <memory>
#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

// Foxglove CompressedVideo 支持（如果可用）
#ifdef FOXGLOVE_MSGS_FOUND
#include <foxglove_msgs/msg/compressed_video.hpp>
#endif

// 深度推理核心类
#include "depth_anything_v2.hpp"
#include "h265_decoder.hpp"

/**
 * @brief 基于 DepthAnythingV2 的 ROS2 节点
 *
 * 功能：
 *  - 订阅实时 RGB 图像 Topic（sensor_msgs::msg::Image）
 *  - 调用 BPU 模型进行深度估计
 *  - 发布彩色深度图（伪彩色，可直接在 RViz 中显示）
 *
 * 主要参数（ROS 参数）：
 *  - model_path  : .hbm 模型文件路径
 *  - input_topic : 输入图像 Topic，默认 /camera/color/image_raw
 *  - output_topic: 输出深度图 Topic，默认 /camera/depth_colored
 */
class DepthInferenceNode : public rclcpp::Node {
public:
    explicit DepthInferenceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DepthInferenceNode() override = default;

private:
    /// 未压缩图像回调：接收 RGB 图像并发布深度伪彩图
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /// 压缩图像回调：接收 H265/JPEG 压缩图像并发布深度伪彩图
    void on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    
#ifdef FOXGLOVE_MSGS_FOUND
    /// Foxglove H265 视频流回调：接收 CompressedVideo 并发布深度伪彩图
    void on_compressed_video(const foxglove_msgs::msg::CompressedVideo::SharedPtr msg);
#endif
    
    /// 核心处理函数：从 cv::Mat 进行深度推理
    void process_image(const cv::Mat& bgr, const std_msgs::msg::Header& header);

    /// 将深度图投影为点云并发布
    void publish_pointcloud(const cv::Mat& depth_f32,
                            const std_msgs::msg::Header& header,
                            int img_w,
                            int img_h);

    // 深度模型实例
    std::unique_ptr<DepthAnythingV2> model_;

    // ROS 通信接口（支持多种订阅方式）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
#ifdef FOXGLOVE_MSGS_FOUND
    rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr compressed_video_sub_;
#endif
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 线程安全标志：确保 BPU 忙碌时直接丢帧，维持实时性
    std::atomic<bool> is_processing_{false};

    // H265 解码器（如果使用压缩图像）
    std::unique_ptr<H265Decoder> h265_decoder_;

    // 参数配置
    std::string model_path_;
    std::string input_topic_;
    std::string output_topic_;
    std::string input_format_;  // "raw"、"h265"、"compressed" 或 "foxglove_h265"

    // 点云参数（用相机内参把深度投影到 3D）
    bool publish_pointcloud_{false};
    std::string pointcloud_topic_{"/camera/depth/points"};
    int pointcloud_stride_{4};     // 下采样步长，默认 4（降低点数与带宽）
    double depth_scale_{1.0};      // 深度尺度：Z = depth * depth_scale（DepthAnything 多为相对深度）
    double fx_{-1.0}, fy_{-1.0};   // 相机内参（像素焦距）
    double cx_{-1.0}, cy_{-1.0};   // 主点（像素坐标），<0 时默认用图像中心
    bool pointcloud_flip_z_{true}; // 为 true 时 Z 取反，修正「前后相反」
    bool pointcloud_flip_x_{true}; // 为 true 时 X 取反，修正「左右相反」
    double pointcloud_min_depth_{0.2};  // 点云有效最小深度（米）
    double pointcloud_max_depth_{10.0}; // 点云有效最大深度（米）
    double pointcloud_extra_yaw_deg_{0.0};   // 额外在雷达坐标系绕 Z 轴的小角度微调（度）
    double pointcloud_extra_pitch_deg_{0.0}; // 额外在雷达坐标系绕 Y 轴的小角度微调（度）
    double pointcloud_extra_roll_deg_{0.0};  // 额外在雷达坐标系绕 X 轴的小角度微调（度）
    std::string pointcloud_frame_id_{}; // 非空时强制点云 frame_id，须为相机光系（如 camera_optical_frame），保证相机在机器狗上
};

#endif // DEPTH_NODE_HPP_