#include "depth_anything_v2.hpp"
#include <iostream>
#include <algorithm>

#define CHECK_RET(ret, msg) if ((ret) != 0) { \
    std::cerr << msg << " Error code: " << ret << std::endl; \
    exit(-1); \
}

DepthAnythingV2::DepthAnythingV2(const std::string& model_path, bool convert_inverse_to_depth) {
    convert_inverse_to_depth_ = convert_inverse_to_depth;
    const char* model_file = model_path.c_str();
    
    // 1. 初始化模型
    CHECK_RET(hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file, 1), "Load HBM failed");
    
    // 2. 获取模型句柄（取第一个模型）
    const char **model_name_list;
    int model_count = 0;
    hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_);
    hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_list[0]);

    // 3. 准备 Tensor 结构
    int in_count, out_count;
    hbDNNGetInputCount(&in_count, dnn_handle_);
    hbDNNGetOutputCount(&out_count, dnn_handle_);
    input_tensors_.resize(in_count);
    output_tensors_.resize(out_count);

    // 4. 获取 Tensor 属性并准备内存（使用 utils 中的工具函数，自动处理动态 stride）
    for (size_t i = 0; i < input_tensors_.size(); i++) {
        hbDNNGetInputTensorProperties(&input_tensors_[i].properties, dnn_handle_, i);
    }
    for (size_t i = 0; i < output_tensors_.size(); i++) {
        hbDNNGetOutputTensorProperties(&output_tensors_[i].properties, dnn_handle_, i);
    }
    
    // 使用 utils 中的工具函数分配内存（自动处理 stride 对齐）
    prepare_input_tensor(input_tensors_);
    prepare_output_tensor(output_tensors_);

    // 5. 缓存输入输出尺寸
    input_h_ = input_tensors_[0].properties.validShape.dimensionSize[2];
    input_w_ = input_tensors_[0].properties.validShape.dimensionSize[3];
    output_h_ = output_tensors_[0].properties.validShape.dimensionSize[1];
    output_w_ = output_tensors_[0].properties.validShape.dimensionSize[2];

    // 打印模型 IO 形状和量化类型，便于在板端确认配置是否正确
    auto& in_prop  = input_tensors_[0].properties;
    auto& out_prop = output_tensors_[0].properties;

    std::cout << "[DepthAnythingV2] Initialized." << std::endl;
    std::cout << "  Input tensor name: pixel_values" << std::endl;
    std::cout << "  Input valid shape: ["
              << in_prop.validShape.dimensionSize[0] << ", "
              << in_prop.validShape.dimensionSize[1] << ", "
              << in_prop.validShape.dimensionSize[2] << ", "
              << in_prop.validShape.dimensionSize[3] << "]" << std::endl;
    std::cout << "  Input tensorType : " << in_prop.tensorType
              << ", quantiType: " << in_prop.quantiType << std::endl;

    std::cout << "  Output tensor name: predicted_depth" << std::endl;
    std::cout << "  Output valid shape: ["
              << out_prop.validShape.dimensionSize[0] << ", "
              << out_prop.validShape.dimensionSize[1] << ", "
              << out_prop.validShape.dimensionSize[2] << "]" << std::endl;
    std::cout << "  Output tensorType : " << out_prop.tensorType
              << ", quantiType: " << out_prop.quantiType << std::endl;
}

DepthAnythingV2::~DepthAnythingV2() {
    for (auto& t : input_tensors_) hbUCPFree(&t.sysMem);
    for (auto& t : output_tensors_) hbUCPFree(&t.sysMem);
    hbDNNRelease(packed_dnn_handle_);
}

// prepare_tensors() 已移除，改用 utils 中的 prepare_input_tensor() 和 prepare_output_tensor()

void DepthAnythingV2::pre_process(const cv::Mat& src) {
    // A. Letterbox Resize（保持宽高比，避免拉伸变形）
    //    先创建目标尺寸的 Mat，然后使用 letterbox_resize 填充
    cv::Mat resized_img(input_h_, input_w_, CV_8UC3);
    letterbox_resize(src, resized_img, 0);  // padding 使用 0（黑色）
    
    // B. Color Convert
    cv::Mat rgb_img;
    cv::cvtColor(resized_img, rgb_img, cv::COLOR_BGR2RGB);

    // C. Normalization (Float32)
    cv::Mat float_img;
    rgb_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    
    // (x - mean) / std
    float_img = float_img - mean_;
    cv::Scalar weights(1.0 / std_val_[0], 1.0 / std_val_[1], 1.0 / std_val_[2]);
    cv::multiply(float_img, weights, float_img);

    // D. HWC to CHW (使用 utils 中的工具函数，自动处理 stride 对齐)
    std::vector<cv::Mat> chw_channels(3);
    cv::split(float_img, chw_channels);  // 分离为 3 个 H×W 的通道
    
    // 使用 utils 中的 write_chw32_to_tensor，自动处理 stride 和缓存刷新
    write_chw32_to_tensor(chw_channels, input_tensors_);
}

void DepthAnythingV2::infer() {
    hbUCPTaskHandle_t task_handle{nullptr};
    CHECK_RET(hbDNNInferV2(&task_handle, output_tensors_.data(), input_tensors_.data(), dnn_handle_), "Infer failed");

    hbUCPSchedParam ctrl_param;
    HB_UCP_INITIALIZE_SCHED_PARAM(&ctrl_param);
    ctrl_param.backend = HB_UCP_BPU_CORE_ANY;

    CHECK_RET(hbUCPSubmitTask(task_handle, &ctrl_param), "Submit task failed");
    CHECK_RET(hbUCPWaitTaskDone(task_handle, 0), "Wait task failed");
    
    // 准备读取输出
    hbUCPMemFlush(&output_tensors_[0].sysMem, HB_SYS_MEM_CACHE_INVALIDATE);
    hbUCPReleaseTask(task_handle);
}

void DepthAnythingV2::post_process(int orig_w, int orig_h, cv::Mat& out_color) {
    cv::Mat depth_f32;
    post_process(orig_w, orig_h, out_color, depth_f32);
}

void DepthAnythingV2::post_process(int orig_w, int orig_h, cv::Mat& out_color, cv::Mat& out_depth_f32) {
    float* raw_output = reinterpret_cast<float*>(output_tensors_[0].sysMem.virAddr);
    cv::Mat pred_depth(output_h_, output_w_, CV_32FC1, raw_output);

    cv::Mat depth_valid;
    crop_letterbox_depth(pred_depth, orig_w, orig_h, input_w_, input_h_, output_w_, output_h_, depth_valid);
    cv::resize(depth_valid, out_depth_f32, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);

    const float eps = 1e-6f;

    // 简化回每帧 min/max 的 inverse-depth 映射
    double r_min_d, r_max_d;
    cv::minMaxLoc(out_depth_f32, &r_min_d, &r_max_d);
    float r_min = static_cast<float>(r_min_d);
    float r_max = static_cast<float>(r_max_d);
    if (r_max - r_min < 1e-5f) {
        r_max = r_min + 1e-5f;
    }

    for (int i = 0; i < out_depth_f32.rows; ++i) {
        float* p = out_depth_f32.ptr<float>(i);
        for (int j = 0; j < out_depth_f32.cols; ++j) {
            float r = p[j];
            float norm_r = (r - r_min) / (r_max - r_min + eps);
            norm_r = std::max(0.0f, std::min(1.0f, norm_r));
            float inv_z = (1.0f / d_max_) + norm_r * (1.0f / d_min_ - 1.0f / d_max_);
            p[j] = 1.0f / (inv_z + eps);
        }
    }

    // 恢复彩色伪深度图（Turbo）
    cv::Mat depth_u8;
    out_depth_f32.convertTo(depth_u8, CV_8UC1, -255.0 / (d_max_ - d_min_), 255.0 * d_max_ / (d_max_ - d_min_));
    cv::applyColorMap(depth_u8, out_color, cv::COLORMAP_TURBO);
}