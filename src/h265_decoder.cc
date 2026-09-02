#include "h265_decoder.hpp"
#include <iostream>

H265Decoder::H265Decoder(int /*max_width*/, int /*max_height*/, bool keyframe_only)
    : keyframe_only_(keyframe_only) {

    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        std::cerr << "[H265Decoder] avcodec_find_decoder(HEVC) failed" << std::endl;
        return;
    }

    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        std::cerr << "[H265Decoder] avcodec_alloc_context3 failed" << std::endl;
        return;
    }

    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
        std::cerr << "[H265Decoder] avcodec_open2 failed" << std::endl;
        avcodec_free_context(&codec_ctx_);
        return;
    }

    packet_ = av_packet_alloc();
    frame_  = av_frame_alloc();
    if (!packet_ || !frame_) {
        std::cerr << "[H265Decoder] av_packet/frame_alloc failed" << std::endl;
        cleanup();
        return;
    }

    init_success_ = true;
    std::cout << "[H265Decoder] FFmpeg HEVC decoder initialized" << std::endl;
}

H265Decoder::~H265Decoder() {
    cleanup();
}

void H265Decoder::cleanup() {
    if (sws_ctx_)   { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
    if (frame_)     { av_frame_free(&frame_); }
    if (packet_)    { av_packet_free(&packet_); }
    if (codec_ctx_) { avcodec_free_context(&codec_ctx_); }
    init_success_ = false;
}

bool H265Decoder::Decode(const std::vector<uint8_t>& data, cv::Mat& out_bgr) {
    if (!init_success_ || data.empty()) return false;

    // Fill packet (no copy — data must stay valid for the duration of this call)
    av_packet_unref(packet_);
    packet_->data = const_cast<uint8_t*>(data.data());
    packet_->size = static_cast<int>(data.size());

    int ret = avcodec_send_packet(codec_ctx_, packet_);
    if (ret < 0 && ret != AVERROR(EAGAIN)) return false;

    ret = avcodec_receive_frame(codec_ctx_, frame_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return false;
    if (ret < 0) return false;

    int w = frame_->width;
    int h = frame_->height;
    auto fmt = static_cast<enum AVPixelFormat>(frame_->format);

    // Rebuild sws context only when resolution or pixel format changes
    if (!sws_ctx_ || sws_src_width_ != w || sws_src_height_ != h || sws_src_fmt_ != fmt) {
        if (sws_ctx_) { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
        sws_ctx_ = sws_getContext(w, h, fmt,
                                   w, h, AV_PIX_FMT_BGR24,
                                   SWS_BILINEAR, nullptr, nullptr, nullptr);
        sws_src_width_  = w;
        sws_src_height_ = h;
        sws_src_fmt_    = fmt;
    }

    if (!sws_ctx_) return false;

    out_bgr.create(h, w, CV_8UC3);
    uint8_t* dst_data[1]     = { out_bgr.data };
    int      dst_linesize[1] = { static_cast<int>(out_bgr.step) };
    sws_scale(sws_ctx_, frame_->data, frame_->linesize, 0, h, dst_data, dst_linesize);

    av_frame_unref(frame_);
    return true;
}
