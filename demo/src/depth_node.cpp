#include "depth_node.hpp"
#include <cstring>  // for std::memcpy
#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

DepthInferenceNode::DepthInferenceNode(const rclcpp::NodeOptions& options)
    : Node("depth_anything_node", options) {
    
    // 1. 参数声明与读取
    this->declare_parameter<std::string>("model_path", "depth_any.hbm");
    this->declare_parameter<std::string>("input_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("output_topic", "/camera/depth_colored");
    this->declare_parameter<std::string>("input_format", "raw");  // "raw"、"h265"、"compressed" 或 "foxglove_h265"
    this->declare_parameter<bool>("publish_pointcloud", true);
    this->declare_parameter<std::string>("pointcloud_topic", "/camera/depth_points");
    this->declare_parameter<int>("pointcloud_stride", 4);
    this->declare_parameter<double>("depth_scale", 0.8);
    this->declare_parameter<double>("fx", 805.299072);
    this->declare_parameter<double>("fy", 805.879883);
    this->declare_parameter<double>("cx", 959.196655);
    this->declare_parameter<double>("cy", 538.812378);
    this->declare_parameter<bool>("pointcloud_flip_z", false);   // 前后相反时设为 true
    this->declare_parameter<bool>("pointcloud_flip_x", false);   // 左右相反时设为 true
    this->declare_parameter<double>("pointcloud_min_depth", 0.2);
    this->declare_parameter<double>("pointcloud_max_depth", 10.0);
    this->declare_parameter<double>("pointcloud_extra_yaw_deg", -25.0);   // 雷达系下绕 Z 轴的额外微调角度（度）
    this->declare_parameter<double>("pointcloud_extra_pitch_deg", -15.0);   // 雷达系下绕 Y 轴的额外微调角度（度）
    this->declare_parameter<double>("pointcloud_extra_roll_deg", 0.0);    // 雷达系下绕 X 轴的额外微调角度（度）
    this->declare_parameter<std::string>("pointcloud_frame_id", "stereo_camera"); // 点云坐标系，应为相机光系，空则用图像 header.frame_id
    this->declare_parameter<bool>("depth_convert_inverse_to_depth", false); // 标准模型输出反深度，true 时转为直观深度 1/raw

    model_path_ = this->get_parameter("model_path").as_string();
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    input_format_ = this->get_parameter("input_format").as_string();

    publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    pointcloud_stride_ = this->get_parameter("pointcloud_stride").as_int();
    depth_scale_ = this->get_parameter("depth_scale").as_double();
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();
    pointcloud_flip_z_ = this->get_parameter("pointcloud_flip_z").as_bool();
    pointcloud_flip_x_ = this->get_parameter("pointcloud_flip_x").as_bool();
    pointcloud_min_depth_ = this->get_parameter("pointcloud_min_depth").as_double();
    pointcloud_max_depth_ = this->get_parameter("pointcloud_max_depth").as_double();
    pointcloud_frame_id_ = this->get_parameter("pointcloud_frame_id").as_string();
    pointcloud_extra_yaw_deg_ = this->get_parameter("pointcloud_extra_yaw_deg").as_double();
    pointcloud_extra_pitch_deg_ = this->get_parameter("pointcloud_extra_pitch_deg").as_double();
    pointcloud_extra_roll_deg_ = this->get_parameter("pointcloud_extra_roll_deg").as_double();
    bool depth_convert_inverse = this->get_parameter("depth_convert_inverse_to_depth").as_bool();

    if (pointcloud_stride_ < 1) pointcloud_stride_ = 1;
    if (pointcloud_max_depth_ <= pointcloud_min_depth_) {
        pointcloud_max_depth_ = pointcloud_min_depth_ + 0.1;
    }

    // 2. 初始化核心组件
    try {
        model_ = std::make_unique<DepthAnythingV2>(model_path_, depth_convert_inverse);
        
        // 如果使用 H265 格式，初始化解码器
        // keyframe_only=true：只处理 I 帧，避免 B/P 帧缺少参考帧导致的错误
        // 虽然会降低帧率，但可以保证深度图的正确性
        if (input_format_ == "h265" || input_format_ == "compressed" || input_format_ == "foxglove_h265") {
            h265_decoder_ = std::make_unique<H265Decoder>(1920, 1080, true);  // keyframe_only=true
            RCLCPP_INFO(this->get_logger(), "H265 decoder initialized");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
        throw;
    }

    // 3. 设置 QoS（针对传感器数据优化：可靠性低、延迟优先）
    auto qos = rclcpp::SensorDataQoS();

    // 4. 创建订阅者与发布者（根据输入格式选择）
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    if (publish_pointcloud_) {
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
        RCLCPP_INFO(this->get_logger(),
                    "PointCloud enabled. Publishing to %s (stride=%d, depth_scale=%.6f)",
                    pointcloud_topic_.c_str(), pointcloud_stride_, depth_scale_);
        if (fx_ <= 0.0 || fy_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                        "PointCloud intrinsics fx/fy not set (fx=%.3f, fy=%.3f). "
                        "Please set parameters fx, fy (and optionally cx, cy).",
                        fx_, fy_);
        }
    }

    if (input_format_ == "raw") {
        // 订阅未压缩图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, qos,
            std::bind(&DepthInferenceNode::on_image, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to uncompressed image: %s", input_topic_.c_str());
    } else if (input_format_ == "h265" || input_format_ == "compressed") {
        // 订阅压缩图像（H265 或其他格式）
        compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic_, qos,
            std::bind(&DepthInferenceNode::on_compressed_image, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to compressed image (H265): %s", input_topic_.c_str());
    } else if (input_format_ == "foxglove_h265") {
        // 订阅 Foxglove CompressedVideo（H265）
#ifdef FOXGLOVE_MSGS_FOUND
        compressed_video_sub_ = this->create_subscription<foxglove_msgs::msg::CompressedVideo>(
            input_topic_, qos,
            std::bind(&DepthInferenceNode::on_compressed_video, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to Foxglove CompressedVideo (H265): %s", input_topic_.c_str());
#else
        RCLCPP_FATAL(this->get_logger(), "foxglove_msgs not found! Please install foxglove_msgs package or use 'h265' format instead.");
        throw std::runtime_error("foxglove_msgs not available");
#endif
    } else {
        RCLCPP_FATAL(this->get_logger(), "Unknown input_format: %s. Use 'raw', 'h265', 'compressed', or 'foxglove_h265'", input_format_.c_str());
        throw std::runtime_error("Invalid input_format parameter");
    }

    RCLCPP_INFO(this->get_logger(), "DepthInferenceNode initialized. Publishing to %s", output_topic_.c_str());
}

void DepthInferenceNode::on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    // --- 实时性控制：如果 BPU 还在算上一帧，直接丢弃当前帧 ---
    bool expected = false;
    if (!is_processing_.compare_exchange_strong(expected, true)) {
        return; 
    }

    try {
        // 1. ROS Image -> OpenCV BGR（手动转换，不使用 cv_bridge）
        cv::Mat bgr;
        
        // 检查编码格式
        if (msg->encoding == "bgr8" || msg->encoding == "BGR8") {
            // BGR8 格式：直接复制数据
            bgr = cv::Mat(msg->height, msg->width, CV_8UC3, 
                         const_cast<uint8_t*>(msg->data.data()));
            bgr = bgr.clone();  // 深拷贝，避免数据被释放
        } else if (msg->encoding == "rgb8" || msg->encoding == "RGB8") {
            // RGB8 格式：需要转换为 BGR
            cv::Mat rgb(msg->height, msg->width, CV_8UC3, 
                       const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "mono8" || msg->encoding == "MONO8") {
            // 灰度图：转换为 BGR
            cv::Mat gray(msg->height, msg->width, CV_8UC1, 
                        const_cast<uint8_t*>(msg->data.data()));
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        } else if (msg->encoding == "nv12" || msg->encoding == "NV12") {
            // NV12：Y (H*W) + UV (H/2*W)，总大小约为 1.5 * H * W
            const auto& buf = msg->data;
            const int width  = static_cast<int>(msg->width);
            const int height = static_cast<int>(msg->height);
            const size_t expect_size = static_cast<size_t>(width) * height * 3 / 2;
            if (buf.size() < expect_size) {
                RCLCPP_WARN(this->get_logger(),
                            "NV12 image data too small: got %zu, expect >= %zu",
                            buf.size(), expect_size);
                is_processing_.store(false);
                return;
            }

            // 直接构造 NV12 平面：总高度为 height * 3 / 2，宽度为 width
            cv::Mat nv12(height * 3 / 2, width, CV_8UC1,
                         const_cast<uint8_t*>(buf.data()));
            cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
            is_processing_.store(false);
            return;
        }

        if (bgr.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image, skip.");
            is_processing_.store(false);
            return;
        }

        // 调用公共处理函数
        process_image(bgr, msg->header);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in on_image: %s", e.what());
    }

    // 释放处理标志
    is_processing_.store(false);
}

void DepthInferenceNode::on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // --- 实时性控制：如果 BPU 还在算上一帧，直接丢弃当前帧 ---
    bool expected = false;
    if (!is_processing_.compare_exchange_strong(expected, true)) {
        return;
    }

    try {
        auto t_decode_start = this->now();
        cv::Mat bgr;

        // 检查压缩格式
        if (msg->format.find("h265") != std::string::npos || 
            msg->format.find("hevc") != std::string::npos ||
            input_format_ == "h265") {
            // H265 解码
            if (!h265_decoder_) {
                RCLCPP_ERROR(this->get_logger(), "H265 decoder not initialized!");
                is_processing_.store(false);
                return;
            }

            cv::Mat rgb;
            if (!h265_decoder_->Decode(msg->data, rgb)) {
                RCLCPP_WARN(this->get_logger(), "H265 decode failed, skip.");
                is_processing_.store(false);
                return;
            }

            // RGB -> BGR 转换（因为模型期望 BGR）
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

        } else if (msg->format.find("jpeg") != std::string::npos || 
                   msg->format.find("jpg") != std::string::npos) {
            // JPEG 解码（使用 OpenCV imdecode，不使用 cv_bridge）
            std::vector<uint8_t> jpeg_data(msg->data.begin(), msg->data.end());
            bgr = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if (bgr.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode JPEG image");
                is_processing_.store(false);
                return;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported compressed format: %s", msg->format.c_str());
            is_processing_.store(false);
            return;
        }

        if (bgr.empty()) {
            RCLCPP_WARN(this->get_logger(), "Decoded image is empty, skip.");
            is_processing_.store(false);
            return;
        }

        auto t_decode_end = this->now();
        double decode_ms = (t_decode_end - t_decode_start).seconds() * 1000.0;
        RCLCPP_INFO(this->get_logger(), "[TIMING] Compressed image decode: %.2f ms", decode_ms);

        // 调用公共处理函数
        process_image(bgr, msg->header);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in on_compressed_image: %s", e.what());
    }

    // 释放处理标志
    is_processing_.store(false);
}

#ifdef FOXGLOVE_MSGS_FOUND
void DepthInferenceNode::on_compressed_video(const foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
    // --- 实时性控制：如果 BPU 还在算上一帧，直接丢弃当前帧 ---
    bool expected = false;
    if (!is_processing_.compare_exchange_strong(expected, true)) {
        return;
    }

    try {
        auto t_decode_start = this->now();
        // 检查格式是否为 H265
        if (msg->format != "h265" && msg->format != "hevc") {
            RCLCPP_WARN(this->get_logger(), "Unsupported video format: %s (expected h265/hevc)", msg->format.c_str());
            is_processing_.store(false);
            return;
        }

        // H265 解码
        if (!h265_decoder_) {
            RCLCPP_ERROR(this->get_logger(), "H265 decoder not initialized!");
            is_processing_.store(false);
            return;
        }

        // H265 解码（FFmpeg 软件解码，直接输出 BGR）
        cv::Mat bgr;
        if (!h265_decoder_->Decode(msg->data, bgr)) {
            RCLCPP_WARN(this->get_logger(), "H265 decode failed, skip.");
            is_processing_.store(false);
            return;
        }

        if (bgr.empty()) {
            RCLCPP_WARN(this->get_logger(), "Decoded image is empty, skip.");
            is_processing_.store(false);
            return;
        }

        auto t_decode_end = this->now();
        double decode_ms = (t_decode_end - t_decode_start).seconds() * 1000.0;
        RCLCPP_INFO(this->get_logger(), "[TIMING] H265 decode: %.2f ms", decode_ms);

        // 构建 header（从 CompressedVideo 的 timestamp / frame_id 字段）
        std_msgs::msg::Header header;
        header.stamp = msg->timestamp;
        header.frame_id = msg->frame_id;

        // 调用公共处理函数
        process_image(bgr, header);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in on_compressed_video: %s", e.what());
    }

    // 释放处理标志
    is_processing_.store(false);
}
#endif

void DepthInferenceNode::process_image(const cv::Mat& bgr, const std_msgs::msg::Header& header) {
    auto t0 = this->now();

    // 1. 模型前处理
    model_->pre_process(bgr);
    auto t1 = this->now();
    double preprocess_ms = (t1 - t0).seconds() * 1000.0;
    RCLCPP_INFO(this->get_logger(), "[TIMING] 模型前处理耗时: %.2f ms", preprocess_ms);

    // 2. 模型推理
    model_->infer();
    auto t2 = this->now();
    double infer_ms = (t2 - t1).seconds() * 1000.0;
    RCLCPP_INFO(this->get_logger(), "[TIMING] 模型推理耗时: %.2f ms", infer_ms);

    // 3. 模型后处理
    cv::Mat color_depth;
    cv::Mat depth_f32;
    model_->post_process(bgr.cols, bgr.rows, color_depth, depth_f32);
    auto t3 = this->now();
    double postprocess_ms = (t3 - t2).seconds() * 1000.0;
    RCLCPP_INFO(this->get_logger(), "[TIMING] 模型后处理耗时: %.2f ms", postprocess_ms);

    // 4. 发布结果（手动构建 ROS Image 消息，不使用 cv_bridge）
    auto out_msg = std::make_shared<sensor_msgs::msg::Image>();
    
    // 设置 header
    out_msg->header = header;
    
    // 设置图像属性
    out_msg->height = color_depth.rows;
    out_msg->width = color_depth.cols;
    out_msg->encoding = "bgr8";
    out_msg->is_bigendian = false;
    out_msg->step = color_depth.cols * color_depth.channels();
    
    // 复制图像数据
    size_t data_size = color_depth.rows * color_depth.cols * color_depth.channels();
    out_msg->data.resize(data_size);
    std::memcpy(out_msg->data.data(), color_depth.data, data_size);

    depth_pub_->publish(*out_msg);

    // 5. 发布点云（可选）
    if (publish_pointcloud_ && cloud_pub_ && !depth_f32.empty()) {
        publish_pointcloud(depth_f32, header, bgr.cols, bgr.rows);
    }

    auto t4 = this->now();
    double pub_ms = (t4 - t3).seconds() * 1000.0;
    double total_ms = (t4 - t0).seconds() * 1000.0;

    RCLCPP_INFO(this->get_logger(), "[TIMING] 发布耗时: %.2f ms", pub_ms);
    RCLCPP_INFO(this->get_logger(), "[TIMING] 总耗时: %.2f ms", total_ms);
}

void DepthInferenceNode::publish_pointcloud(const cv::Mat& depth_f32,
                                            const std_msgs::msg::Header& header,
                                            int img_w, int img_h)
{
    if (depth_f32.empty() || fx_ <= 0.0) {
        return;
    }

    // 使用标定文件 vita_calib.yaml 中 stereo_left 相机的内参：
    // fx = 805.299072, fy = 805.879883, cx = 959.196655, cy = 538.812378
    const float fx = static_cast<float>(fx_);
    const float fy = static_cast<float>(fy_);

    float cx = static_cast<float>(cx_);
    float cy = static_cast<float>(cy_);
    if (cx <= 0.0f) cx = img_w * 0.5f;
    if (cy <= 0.0f) cy = img_h * 0.5f;

    const float inv_fx = 1.0f / fx;
    const float inv_fy = 1.0f / fy;

    // 相机坐标系 C -> 雷达坐标系 L 的刚体变换（硬编码自 vita_calib.yaml）：
    //
    // Extr_B_C (body -> stereo_left camera):
    //   orientation: [-0.4962027465, 0.5058446649, -0.5064091394, 0.4913794795]
    //   translation: [0.2447188952, 0.0349999205, 0.1335284966]
    //
    // Extr_B_L (body -> lidar):
    //   orientation: [0, 0, 0, 1]
    //   translation: [0.17288, 0, 0.21835]
    //
    // 通过 T_LC = inv(T_BL) * T_BC 得到：
    //   R_LC =
    //     [-0.0041919812,  0.0246810282, -0.9996865880;
    //       0.9999764175, -0.0053347643, -0.0043249053;
    //      -0.0054398354, -0.9996811427, -0.0246580830]
    //   t_LC = [-0.0718388952, -0.0349999205, -0.0848215034]
    const float R_LC[3][3] = {
        { -0.0041919812f,  0.0246810282f, -0.9996865880f },
        {  0.9999764175f, -0.0053347643f, -0.0043249053f },
        { -0.0054398354f, -0.9996811427f, -0.0246580830f }
    };

    // const float R_LC[3][3] = {
    //     { -0.024658f, -0.999481f, 0.00554f},
    //     { -0.004315f, -0.005334f, -0.999918f},
    //     { 0.999672f, -0.024546f, -0.004192f}
    // };0311WZW

    float t_LC[3] = {
        -0.0718388952f,
        -0.0349999205f,
        -0.0848215034f
    };

    // float t_LC[3] = {
    //     0.0718388952f,
    //     0.0349999205f,
    //     -0.0848215034f
    // }; 0311WZW

    // 允许在雷达坐标系再做一个很小的欧拉角微调旋转（Z-Y-X 顺序），以人工和激光点云对齐。
    // 参数单位均为度，正方向为右手系下的逆时针：
    //   yaw  : 绕 Z 轴
    //   pitch: 绕 Y 轴
    //   roll : 绕 X 轴
    const float extra_yaw_rad   = static_cast<float>(pointcloud_extra_yaw_deg_ * M_PI / 180.0);
    const float extra_pitch_rad = static_cast<float>(pointcloud_extra_pitch_deg_ * M_PI / 180.0);
    const float extra_roll_rad  = static_cast<float>(pointcloud_extra_roll_deg_ * M_PI / 180.0);

    const float cyaw = std::cos(extra_yaw_rad);
    const float syaw = std::sin(extra_yaw_rad);
    const float cpitch = std::cos(extra_pitch_rad);
    const float spitch = std::sin(extra_pitch_rad);
    const float croll = std::cos(extra_roll_rad);
    const float sroll = std::sin(extra_roll_rad);

    // Rz(yaw)
    float Rz[3][3] = {
        { cyaw, -syaw, 0.0f },
        { syaw,  cyaw, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    // Ry(pitch)
    float Ry[3][3] = {
        {  cpitch, 0.0f, spitch },
        { 0.0f, 1.0f, 0.0f },
        { -spitch, 0.0f, cpitch }
    };
    // Rx(roll)
    float Rx[3][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f,  croll, -sroll },
        { 0.0f,  sroll,  croll }
    };

    // 先算 R_tmp = Rz * Ry
    float R_tmp[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_tmp[i][j] = Rz[i][0] * Ry[0][j] +
                          Rz[i][1] * Ry[1][j] +
                          Rz[i][2] * Ry[2][j];
        }
    }

    // 再算 R_extra = R_tmp * Rx = Rz * Ry * Rx
    float R_extra[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_extra[i][j] = R_tmp[i][0] * Rx[0][j] +
                            R_tmp[i][1] * Rx[1][j] +
                            R_tmp[i][2] * Rx[2][j];
        }
    }

    // 最终使用的旋转：R_LC_adj = R_extra * R_LC
    float R_LC_adj[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_LC_adj[i][j] = R_extra[i][0] * R_LC[0][j] +
                             R_extra[i][1] * R_LC[1][j] +
                             R_extra[i][2] * R_LC[2][j];
        }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = "vita_lidar"; 
    
    const int cols = depth_f32.cols;
    const int rows = depth_f32.rows;
    const uint32_t num_points = static_cast<uint32_t>(cols * rows);

    cloud.height = 1;
    cloud.width = num_points;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const float q_nan = std::numeric_limits<float>::quiet_NaN();

    // --- 坐标投影与 C->L 刚体变换 ---
    for (int v = 0; v < rows; ++v) {
        const float* row_ptr = depth_f32.ptr<float>(v);
        for (int u = 0; u < cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
            float z_depth = row_ptr[u] * static_cast<float>(depth_scale_);

            if (z_depth < static_cast<float>(pointcloud_min_depth_) ||
                z_depth > static_cast<float>(pointcloud_max_depth_) ||
                !std::isfinite(z_depth)) {
                *iter_x = *iter_y = *iter_z = q_nan;
                continue;
            }

            // 1. 像素坐标 (u,v) + 深度 z_depth -> 相机光学坐标系 C
            //    相机坐标系：X 右, Y 下, Z 向前
            float x_c = (static_cast<float>(u) - cx) * z_depth * inv_fx;
            float y_c = (static_cast<float>(v) - cy) * z_depth * inv_fy;
            float z_c = z_depth;

            // 2. 相机系 C -> 雷达系 L：p_L = R_LC * p_C + t_LC
            float x_l =
                R_LC_adj[0][0] * x_c + R_LC_adj[0][1] * y_c + R_LC_adj[0][2] * z_c + t_LC[0];
            float y_l =
                R_LC_adj[1][0] * x_c + R_LC_adj[1][1] * y_c + R_LC_adj[1][2] * z_c + t_LC[1];
            float z_l =
                R_LC_adj[2][0] * x_c + R_LC_adj[2][1] * y_c + R_LC_adj[2][2] * z_c + t_LC[2];

            *iter_x = x_l;
            *iter_y = y_l;
            *iter_z = z_l;
        }
    }
    cloud_pub_->publish(cloud);
}