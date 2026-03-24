#ifndef DEPTH_ANYTHING_V2_HPP_
#define DEPTH_ANYTHING_V2_HPP_

#include <string>
#include <vector>
#include "common_utils.hpp"
#include "preprocess_utils.hpp"
#include "postprocess_utils.hpp"
#include <opencv2/opencv.hpp>

// 地平线 DNN 库
#include "hobot/dnn/hb_dnn.h"
#include "hobot/hb_ucp.h"
#include "hobot/hb_ucp_sys.h"

/**
 * @class DepthAnythingV2
 * @brief 封装 DepthAnythingV2 在 BPU 上的推理流程
 */
class DepthAnythingV2 {
public:
    /**
     * @brief 构造函数，加载模型并分配内存
     * @param model_path [in] .hbm 模型文件路径
     * @param convert_inverse_to_depth [in] 若为 true，将模型输出的反深度转为直观深度 depth=1/raw（标准相对模型输出为反深度）
     */
    explicit DepthAnythingV2(const std::string& model_path, bool convert_inverse_to_depth = true);

    /**
     * @brief 析构函数，释放 BPU 内存和句柄
     */
    ~DepthAnythingV2();

    /**
     * @brief 预处理：将 BGR 图像转换为模型要求的 NCHW Float32 格式
     * @param src [in] 输入的 OpenCV BGR 图像
     */
    void pre_process(const cv::Mat& src);

    /**
     * @brief 执行 BPU 硬件推理
     */
    void infer();

    /**
     * @brief 后处理：获取深度图，归一化并生成伪彩色图
     * @param orig_w [in] 原始图像宽度，用于 Resize 回原图
     * @param orig_h [in] 原始图像高度
     * @param out_color [out] 输出的伪彩色深度图 (CV_8UC3)
     */
    void post_process(int orig_w, int orig_h, cv::Mat& out_color);

    /**
     * @brief 后处理：输出原图尺寸的浮点深度图 + 伪彩色深度图
     * @param orig_w [in] 原始图像宽度
     * @param orig_h [in] 原始图像高度
     * @param out_color [out] 输出的伪彩色深度图 (CV_8UC3, size=orig_h×orig_w)
     * @param out_depth_f32 [out] 输出的浮点深度图 (CV_32FC1, size=orig_h×orig_w)
     */
    void post_process(int orig_w, int orig_h, cv::Mat& out_color, cv::Mat& out_depth_f32);

private:
    // 模型句柄
    hbDNNPackedHandle_t packed_dnn_handle_;
    hbDNNHandle_t dnn_handle_;

    // Tensor 容器
    std::vector<hbDNNTensor> input_tensors_;
    std::vector<hbDNNTensor> output_tensors_;

    // 模型输入输出属性
    int input_h_, input_w_;
    int output_h_, output_w_;

    // 常数定义 (DepthAnything 默认归一化参数)
    const cv::Scalar mean_{0.485, 0.456, 0.406};
    const cv::Scalar std_val_{0.229, 0.224, 0.225};

    bool convert_inverse_to_depth_{true}; // 是否将反深度转为 depth=1/raw

    // 简化版映射参数：每帧 min/max + inverse-depth 区间映射
    float d_min_{0.6f};
    float d_max_{10.0f};
};

#endif // DEPTH_ANYTHING_V2_HPP_