#include "h265_decoder.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>  // for usleep

// 参考 sample_codec.c 中的 vp_decode_config_param
static int32_t vp_decode_config_param_h265(media_codec_context_t *context, 
                                           int32_t width, int32_t height) {
    mc_video_codec_dec_params_t *params;
    context->encoder = false; // decoder
    params = &context->video_dec_params;
    params->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    params->pix_fmt = MC_PIXEL_FORMAT_NV12;
    params->bitstream_buf_size = (width * height * 3 / 2 + 0x3ff) & ~0x3ff;
    // 使用与官方 sample_codec 一致的缓冲区数量
    params->bitstream_buf_count = 3;  // 输入缓冲区数量
    params->frame_buf_count = 3;       // 输出缓冲区数量

    context->codec_id = MEDIA_CODEC_ID_H265;
    params->h265_dec_config.bandwidth_Opt = true;
    params->h265_dec_config.reorder_enable = true;
    params->h265_dec_config.skip_mode = 0;
    params->h265_dec_config.cra_as_bla = false;
    params->h265_dec_config.dec_temporal_id_mode = 0;
    params->h265_dec_config.target_dec_temporal_id_plus1 = 0;
    // 注意：error_concealment 字段可能不在当前版本的 API 中

    return 0;
}

// 参考 sample_codec.c 中的 vp_codec_init
static int32_t vp_codec_init_h265(media_codec_context_t *context) {
    int32_t ret = 0;

    ret = hb_mm_mc_initialize(context);
    if (0 != ret) {
        std::cerr << "[H265Decoder] hb_mm_mc_initialize failed, ret=0x" 
                  << std::hex << ret << std::dec << std::endl;
        return -1;
    }

    ret = hb_mm_mc_configure(context);
    if (0 != ret) {
        std::cerr << "[H265Decoder] hb_mm_mc_configure failed, ret=0x" 
                  << std::hex << ret << std::dec << std::endl;
        hb_mm_mc_release(context);
        return -1;
    }

    return 0;
}

// 参考 sample_codec.c 中的 vp_codec_start
static int32_t vp_codec_start_h265(media_codec_context_t *context) {
    int32_t ret = 0;
    mc_av_codec_startup_params_t startup_params = {0};

    ret = hb_mm_mc_start(context, &startup_params);
    if (ret != 0) {
        std::cerr << "[H265Decoder] hb_mm_mc_start failed, ret=0x" 
                  << std::hex << ret << std::dec << std::endl;
        return -1;
    }

    return ret;
}

// 参考 sample_codec.c 中的 vp_codec_stop
static int32_t vp_codec_stop_h265(media_codec_context_t *context) {
    // 必须调用 hb_mm_mc_stop 而不是 hb_mm_mc_pause
    hb_mm_mc_stop(context);
    return 0;
}

// 参考 sample_codec.c 中的 vp_codec_deinit
static int32_t vp_codec_deinit_h265(media_codec_context_t *context) {
    int32_t ret = 0;
    ret = hb_mm_mc_release(context);
    if (ret != 0) {
        std::cerr << "[H265Decoder] hb_mm_mc_release failed, ret=0x" 
                  << std::hex << ret << std::dec << std::endl;
        return -1;
    }
    return 0;
}

// 参考 sample_codec.c 中的 vp_codec_set_input
static int32_t vp_codec_set_input_h265(media_codec_context_t *context,
                                       media_codec_buffer_t *frame_buffer,
                                       uint8_t *data, uint32_t data_size, int32_t eos) {
    int32_t ret = 0;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL) || (!eos && (data == NULL))) {
        return -1;
    }

    buffer = frame_buffer;
    buffer->type = MC_VIDEO_STREAM_BUFFER;
    
    // 使用短超时，避免阻塞整个 ROS callback
    ret = hb_mm_mc_dequeue_input_buffer(context, buffer, 50);
    if (ret != 0) {
        // std::cerr << "[H265Decoder] hb_mm_mc_dequeue_input_buffer failed, ret=0x" 
        //          << std::hex << ret << std::dec << std::endl;
        return -1;
    }

    if (buffer->vstream_buf.size < data_size) {
        std::cerr << "[H265Decoder] Input stream buffer too small: " 
                  << buffer->vstream_buf.size << " < " << data_size << std::endl;
        hb_mm_mc_queue_input_buffer(context, buffer, 50);
        return -1;
    }

    buffer->type = MC_VIDEO_STREAM_BUFFER;
    if (eos == 0) {
        buffer->vstream_buf.size = data_size;
        buffer->vstream_buf.stream_end = 0;
    } else {
        buffer->vstream_buf.size = 0;
        buffer->vstream_buf.stream_end = 1;
    }

    memcpy(buffer->vstream_buf.vir_ptr, data, data_size);

    ret = hb_mm_mc_queue_input_buffer(context, buffer, 50);
    if (ret != 0) {
        std::cerr << "[H265Decoder] hb_mm_mc_queue_input_buffer failed, ret=0x" 
                  << std::hex << ret << std::dec << std::endl;
        return -1;
    }

    return 0;
}

// 参考 sample_codec.c 中的 vp_codec_get_output
static int32_t vp_codec_get_output_h265(media_codec_context_t *context,
                                        media_codec_buffer_t *frame_buffer,
                                        media_codec_output_buffer_info_t *buffer_info,
                                        int32_t timeout) {
    int32_t ret = 0;
    media_codec_output_buffer_info_t *info = NULL;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL) || (buffer_info == NULL)) {
        return -1;
    }
    buffer = frame_buffer;
    info = buffer_info;

    ret = hb_mm_mc_dequeue_output_buffer(context, buffer, info, timeout);
    if (ret != 0 && ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
        // -268435443 是 HB_MEDIA_ERR_WAIT_TIMEOUT 的值
        if (ret != -268435443) {
            // std::cerr << "[H265Decoder] hb_mm_mc_dequeue_output_buffer failed, ret=0x" 
            //          << std::hex << ret << std::dec << std::endl;
        }
        return -1;
    } else if (ret == -268435443 || ret == (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
        // 超时，正常情况
        return -1;
    }

    if (buffer->type != MC_VIDEO_FRAME_BUFFER) {
        // 不是视频帧缓冲区，释放并返回
        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0) {
            std::cerr << "[H265Decoder] hb_mm_mc_queue_output_buffer failed, ret=0x" 
                      << std::hex << ret << std::dec << std::endl;
            return -1;
        }
        return -1;
    }

    if (info->video_frame_info.decode_result != 0 || buffer->vframe_buf.size == 0) {
        // 解码失败或帧大小为0，释放并返回
        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0) {
            std::cerr << "[H265Decoder] hb_mm_mc_queue_output_buffer failed, ret=0x" 
                      << std::hex << ret << std::dec << std::endl;
            return -1;
        }
        return -1;
    }

    return ret;
}

// 参考 sample_codec.c 中的 vp_codec_release_output
static int32_t vp_codec_release_output_h265(media_codec_context_t *context,
                                            media_codec_buffer_t *frame_buffer) {
    int32_t ret = 0;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL)) {
        std::cerr << "[H265Decoder] vp_codec_release_output_h265: param is NULL!" << std::endl;
        return -1;
    }
    buffer = frame_buffer;

    if (buffer != NULL) {
        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0) {
            std::cerr << "[H265Decoder] hb_mm_mc_queue_output_buffer failed, ret=0x" 
                      << std::hex << ret << std::dec << std::endl;
            return -1;
        }
    }

    return ret;
}

H265Decoder::H265Decoder(int max_width, int max_height, bool keyframe_only)
    : max_width_(max_width), max_height_(max_height), keyframe_only_(keyframe_only) {
    
    memset(&context_, 0, sizeof(media_codec_context_t));
    memset(&input_buffer_, 0, sizeof(media_codec_buffer_t));
    memset(&output_buffer_, 0, sizeof(media_codec_buffer_t));
    memset(&output_info_, 0, sizeof(media_codec_output_buffer_info_t));
    
    init_success_ = init_hardware_decoder();
    
    if (init_success_) {
        std::cout << "[H265Decoder] Initialized hardware H265 decoder, max resolution "
                  << max_width_ << "x" << max_height_;
        if (keyframe_only_) {
            std::cout << " (keyframe-only mode)";
        }
        std::cout << std::endl;
    }
}

H265Decoder::~H265Decoder() {
    cleanup_hardware_decoder();
}

bool H265Decoder::init_hardware_decoder() {
    // 0. 清空上下文
    memset(&context_, 0, sizeof(media_codec_context_t));

    // 1. 配置解码参数（参考 sample_codec.c 的 vp_decode_config_param）
    int32_t ret = vp_decode_config_param_h265(&context_, max_width_, max_height_);
    if (ret != 0) {
        std::cerr << "[H265Decoder] vp_decode_config_param_h265 failed" << std::endl;
        return false;
    }

    // 2. 初始化解码器（参考 sample_codec.c 的 vp_codec_init）
    ret = vp_codec_init_h265(&context_);
    if (ret != 0) {
        std::cerr << "[H265Decoder] vp_codec_init_h265 failed" << std::endl;
        return false;
    }

    // 3. 启动解码器（参考 sample_codec.c 的 vp_codec_start）
    ret = vp_codec_start_h265(&context_);
    if (ret != 0) {
        std::cerr << "[H265Decoder] vp_codec_start_h265 failed" << std::endl;
        vp_codec_deinit_h265(&context_);
        return false;
    }

    // 重置状态
    consecutive_errors_ = 0;
    frames_sent_ = 0;
    init_success_ = true;

    return true;
}

void H265Decoder::cleanup_hardware_decoder() {
    if (init_success_) {
        vp_codec_stop_h265(&context_);
        vp_codec_deinit_h265(&context_);
        init_success_ = false;
        consecutive_errors_ = 0;
    }
}

bool H265Decoder::reset_decoder() {
    std::cerr << "[H265Decoder] Decoder stuck, resetting..." << std::endl;
    
    // 1. 停止并清理当前解码器
    cleanup_hardware_decoder();
    
    // 2. 等待资源释放（重要：给硬件一些时间释放资源）
    usleep(100000);  // 100ms
    
    // 3. 重新初始化
    bool success = init_hardware_decoder();
    if (success) {
        std::cerr << "[H265Decoder] Decoder reset successful" << std::endl;
    } else {
        std::cerr << "[H265Decoder] Decoder reset failed, will retry later" << std::endl;
        // 如果初始化失败，等待更长时间后重试
        usleep(500000);  // 500ms
    }
    
    return success;
}

void H265Decoder::log_decoder_status() {
    if (!init_success_) {
        return;
    }
    
    // 查询解码器状态（如果 API 可用）
    // 注意：hb_mm_mc_get_status 可能不在所有版本中可用
    // 这里先注释掉，如果编译错误可以移除
    /*
    mc_video_codec_status_t status = {0};
    int32_t ret = hb_mm_mc_get_status(&context_, &status);
    if (ret == 0) {
        std::cerr << "[H265Decoder] Status: in_q=" << status.input_buf_cnt 
                  << ", out_q=" << status.output_buf_cnt << std::endl;
    }
    */
}

bool H265Decoder::send_input_data(const uint8_t* data, size_t data_size, bool eos) {
    if (!init_success_ || !data || data_size == 0) {
        return false;
    }

    int32_t ret = vp_codec_set_input_h265(&context_, &input_buffer_, 
                                          const_cast<uint8_t*>(data), 
                                          static_cast<uint32_t>(data_size), 
                                          eos ? 1 : 0);
    return (ret == 0);
}

bool H265Decoder::get_output_frame(cv::Mat& out_bgr, int timeout_ms) {
    if (!init_success_) {
        return false;
    }

    memset(&output_buffer_, 0, sizeof(media_codec_buffer_t));
    memset(&output_info_, 0, sizeof(media_codec_output_buffer_info_t));

    // 尝试获取输出帧
    int32_t ret = vp_codec_get_output_h265(&context_, &output_buffer_, &output_info_, timeout_ms);
    if (ret != 0) {
        // 超时或其他错误，正常情况
        return false;
    }

    // 检查输出缓冲区有效性
    if (output_buffer_.type != MC_VIDEO_FRAME_BUFFER) {
        // 不是视频帧缓冲区，释放并返回
        vp_codec_release_output_h265(&context_, &output_buffer_);
        return false;
    }

    if (output_buffer_.vframe_buf.width <= 0 || 
        output_buffer_.vframe_buf.height <= 0 ||
        !output_buffer_.vframe_buf.vir_ptr[0]) {
        vp_codec_release_output_h265(&context_, &output_buffer_);
        return false;
    }

    // 检查解码结果
    if (output_info_.video_frame_info.decode_result != 0 || 
        output_buffer_.vframe_buf.size == 0) {
        // 解码失败或帧大小为0，释放并返回
        vp_codec_release_output_h265(&context_, &output_buffer_);
        return false;
    }

    int width = output_buffer_.vframe_buf.width;
    int height = output_buffer_.vframe_buf.height;
    int stride = output_buffer_.vframe_buf.stride;
    int vstride = output_buffer_.vframe_buf.vstride;

    // 将 NV12 转换为 BGR
    bool success = nv12_to_bgr(output_buffer_.vframe_buf.vir_ptr[0], 
                               width, height, stride, vstride, out_bgr);

    // 释放输出缓冲区（无论转换是否成功都要释放）
    vp_codec_release_output_h265(&context_, &output_buffer_);

    return success;
}

bool H265Decoder::nv12_to_bgr(const uint8_t* nv12_data, int width, int height, 
                               int stride, int vstride, cv::Mat& out_bgr) {
    if (!nv12_data || width <= 0 || height <= 0) {
        return false;
    }

    // NV12: Y plane (stride * vstride) + interleaved UV plane (stride * vstride / 2)
    // 注意：output_buffer_.vframe_buf.vir_ptr[0] 是 Y plane
    //      output_buffer_.vframe_buf.vir_ptr[1] 是 UV plane
    const uint8_t* y_data = output_buffer_.vframe_buf.vir_ptr[0];
    const uint8_t* uv_data = output_buffer_.vframe_buf.vir_ptr[1];

    if (!y_data || !uv_data) {
        return false;
    }

    // 创建 Y plane Mat（考虑 stride）
    cv::Mat y_plane(vstride, stride, CV_8UC1, const_cast<uint8_t*>(y_data));
    
    // 创建 UV plane Mat（UV 是交错存储的，每行 stride/2 个 UV 对）
    cv::Mat uv_plane(vstride / 2, stride / 2, CV_8UC2, const_cast<uint8_t*>(uv_data));

    // 如果 stride 不等于 width，需要裁剪
    cv::Mat y_cropped, uv_cropped;
    if (stride == width && vstride == height) {
        y_cropped = y_plane;
    } else {
        y_cropped = y_plane(cv::Rect(0, 0, width, height));
    }

    if (stride / 2 == width / 2 && vstride / 2 == height / 2) {
        uv_cropped = uv_plane;
    } else {
        uv_cropped = uv_plane(cv::Rect(0, 0, width / 2, height / 2));
    }

    // 手动构造 NV12 Mat（Y + 交错的 UV）
    // NV12 格式：Y plane (height * width) + UV plane (height/2 * width, 交错)
    cv::Mat nv12_mat(height * 3 / 2, width, CV_8UC1);
    
    // 复制 Y plane
    if (y_cropped.isContinuous() && y_cropped.cols == width && y_cropped.rows == height) {
        memcpy(nv12_mat.data, y_cropped.data, width * height);
    } else {
        y_cropped.copyTo(nv12_mat(cv::Rect(0, 0, width, height)));
    }
    
    // 复制 UV plane（交错格式，需要按行复制）
    uint8_t* uv_dst = nv12_mat.data + width * height;
    const uint8_t* uv_src = uv_cropped.data;
    int uv_row_size = width;  // UV 交错，每行 width 字节（width/2 个 UV 对）
    for (int i = 0; i < height / 2; i++) {
        memcpy(uv_dst + i * uv_row_size, uv_src + i * uv_cropped.step[0], uv_row_size);
    }

    // NV12 -> BGR
    cv::Mat bgr;
    cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);

    if (bgr.empty() || bgr.cols != width || bgr.rows != height) {
        return false;
    }

    out_bgr = bgr.clone();
    return true;
}

bool H265Decoder::Decode(const std::vector<uint8_t>& data, cv::Mat& out_bgr) {
    if (!init_success_) {
        if (!init_hardware_decoder()) {
            usleep(100000);  // 100ms
            return false;
        }
    }
    
    if (data.empty()) return false;

    // 官方 Topic 信息：H265 编码的左/右相机视频流
    // data 压缩视频帧数据 (不支持B帧; h264/h265需Annex B格式; 关键帧需包含SPS/PPS等NAL单元; 必须包含解码一帧所需的完整数据包)
    // 说明我们不需要自己检测 SPS/PPS，关键帧自带完整的配置，直接送入解码器即可

        // 打印前 16 个字节，用于调试 NALU
        if (data.size() >= 16) {
            std::cout << "[H265Decoder] Frame size: " << data.size() << " bytes. First 16 bytes: ";
            for (int i = 0; i < 16; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        // 确保数据以 Annex B 起始码开始 (00 00 00 01 或 00 00 01)
    const uint8_t* nalu_data = data.data();
    size_t nalu_size = data.size();
    
    std::vector<uint8_t> annexb_data;
    if (nalu_size >= 4) {
        if (!(nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 0 && nalu_data[3] == 1) &&
            !(nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 1)) {
            // 如果不是 Annex B 起始码，可能是 AVCC 格式，或者丢失了起始码
            // 尝试强制添加起始码（假设整个 data 是一个完整的 NALU 或 AVCC 已经去掉了长度前缀）
            std::cerr << "[H265Decoder] Warning: Missing Annex B start code. Trying to prepend..." << std::endl;
            annexb_data.reserve(nalu_size + 4);
            annexb_data.push_back(0);
            annexb_data.push_back(0);
            annexb_data.push_back(0);
            annexb_data.push_back(1);
            
            // 如果是 AVCC，前4个字节是长度，跳过它们
            // 但如果不是，这可能不正确，安全起见我们先不跳过，或者做个启发式判断
            // 这里为了稳妥，直接作为整个 payload 发送
            annexb_data.insert(annexb_data.end(), data.begin(), data.end());
            
            nalu_data = annexb_data.data();
            nalu_size = annexb_data.size();
        }
    }

    // 检查是否包含 VPS/SPS/PPS (类型 32/33/34)，或者是否是 IDR 帧 (19/20/21)
    // H265 NALU type 位于第1个字节的 bit 1~6
    // 注意: Annex B 开头是 00 00 00 01，所以 NALU header 在 data[4]
    if (!seq_header_sent_) {
        bool has_keyframe_or_param = false;
        int start_idx = 0;
        if (nalu_size >= 4 && nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 0 && nalu_data[3] == 1) {
            start_idx = 4;
        } else if (nalu_size >= 3 && nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 1) {
            start_idx = 3;
        }
        
        if (start_idx > 0 && start_idx < nalu_size) {
            int nalu_type = (nalu_data[start_idx] & 0x7E) >> 1;
            // 32: VPS, 33: SPS, 34: PPS
            // 19, 20: IDR
            if (nalu_type >= 32 && nalu_type <= 34) {
                has_keyframe_or_param = true;
            } else if (nalu_type >= 16 && nalu_type <= 21) {
                // IRAP pictures (CRA, BLA, IDR)
                has_keyframe_or_param = true;
            }
        }
        
        if (!has_keyframe_or_param) {
            std::cout << "[H265Decoder] Skipping frame because sequence header not yet received." << std::endl;
            return false;
        } else {
            std::cout << "[H265Decoder] First keyframe/param set received, starting decoding." << std::endl;
            seq_header_sent_ = true;
        }
    }
    bool got_frame = false;
    cv::Mat temp_bgr;
    if (frames_sent_ > 0) {
        while (get_output_frame(temp_bgr, 50)) {  // 恢复短超时
            out_bgr = temp_bgr; // 覆盖，保留最新帧
            got_frame = true;
            consecutive_errors_ = 0;
        }
    }

    // 2. 发送新的输入数据
    if (!send_input_data(nalu_data, nalu_size, false)) {
        consecutive_errors_++;
        std::cerr << "[H265Decoder] send_input_data failed, consecutive_errors=" << consecutive_errors_ << std::endl;
        // 如果错误太多，可能是硬件卡死，重置
        if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
            std::cerr << "[H265Decoder] Too many consecutive errors (" << consecutive_errors_ 
                      << ") during send, resetting decoder" << std::endl;
            reset_decoder();
        }
    } else {
        frames_sent_++;
        // std::cout << "[H265Decoder] send_input_data success, size=" << data.size() << std::endl;
    }

    // 3. 发送后再次尝试获取输出帧
    if (!got_frame) {
        if (get_output_frame(temp_bgr, 50)) { // 恢复短超时
            out_bgr = temp_bgr;
            got_frame = true;
            consecutive_errors_ = 0;
        }
    }

    // 只要有任何输出帧，就说明解码成功
    // 如果没有，这属于正常情况（流水线填充中），不用报错
    return got_frame;
}
