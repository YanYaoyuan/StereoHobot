#ifndef H265_DECODER_H_
#define H265_DECODER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

// 使用地平线硬件解码器 hb_media_codec
extern "C" {
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include "hbmem.h"
}

/**
 * @brief H265Decoder（硬件解码版本）
 *
 * 基于地平线 hb_media_codec 硬件解码器，参考 sample_codec.c 实现。
 * 使用硬件加速，性能优于 FFmpeg 软件解码。
 *
 * 设计：
 *  - 使用 hb_mm_mc_* 系列 API
 *  - 输入：H265 Annex-B 字节流（来自 Foxglove CompressedVideo.data）
 *  - 输出：BGR 图像（cv::Mat, CV_8UC3）
 *  - 内部将硬件解码的 NV12 格式转换为 BGR
 */
class H265Decoder {
public:
    /**
     * @brief 构造函数
     * @param max_width  最大宽度（用于初始化解码器）
     * @param max_height 最大高度（用于初始化解码器）
     * @param keyframe_only 如果为 true，只解码关键帧（I 帧），跳过 B/P 帧
     *                      注意：硬件解码器会自动处理，此参数主要用于兼容性
     */
    explicit H265Decoder(int max_width = 1920, int max_height = 1080, bool keyframe_only = false);
    ~H265Decoder();

    // 禁止拷贝，防止句柄多次释放
    H265Decoder(const H265Decoder&) = delete;
    H265Decoder& operator=(const H265Decoder&) = delete;

    /**
     * @brief 解码一帧 H265 字节流（同步接口）
     * @param data 原始 H265 NALU 数据（单帧）
     * @param out_bgr 输出的 BGR 图像 (CV_8UC3)
     * @return true 解码成功并获得一帧图像；false 表示解码失败
     */
    bool Decode(const std::vector<uint8_t>& data, cv::Mat& out_bgr);

private:
    bool init_success_ = false;
    int max_width_  = 0;
    int max_height_ = 0;
    bool keyframe_only_ = false;

    // 硬件解码器上下文
    media_codec_context_t context_;
    
    // 输入输出缓冲区
    media_codec_buffer_t input_buffer_;
    media_codec_buffer_t output_buffer_;
    media_codec_output_buffer_info_t output_info_;
    
    // 序列头相关（SPS/PPS）
    bool seq_header_sent_ = false;
    std::vector<uint8_t> seq_header_;
    
    // 错误计数（用于检测解码器挂起）
    int consecutive_errors_ = 0;
    static const int MAX_CONSECUTIVE_ERRORS = 30;
    
    // 统计发送的帧数
    uint64_t frames_sent_ = 0;
    
    // 辅助函数
    bool init_hardware_decoder();
    void cleanup_hardware_decoder();
    bool reset_decoder();  // 重置解码器
    bool send_input_data(const uint8_t* data, size_t data_size, bool eos = false);
    bool get_output_frame(cv::Mat& out_bgr, int timeout_ms = 100);
    bool nv12_to_bgr(const uint8_t* nv12_data, int width, int height, int stride, int vstride, cv::Mat& out_bgr);
    void log_decoder_status();  // 打印解码器状态（调试用）
};

#endif // H265_DECODER_H_
