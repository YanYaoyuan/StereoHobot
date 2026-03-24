#include "h265_decoder.h"
#include <atomic>
#include <iostream>
#include <vector>

namespace {
void log_hevc_decode_limited(const char *msg) {
  static std::atomic<int> n{0};
  if (n.fetch_add(1) < 40)
    std::cerr << "[H265Decoder] " << msg << std::endl;
}
} // namespace

H265Decoder::H265Decoder() = default;

H265Decoder::~H265Decoder() { reset(); }

bool H265Decoder::annex_b_starts(const uint8_t *d, size_t n) {
  if (n >= 4 && d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 1)
    return true;
  if (n >= 3 && d[0] == 0 && d[1] == 0 && d[2] == 1)
    return true;
  return false;
}

/**
 * 扫描 Annex-B AU：若含 VPS/SPS/PPS 或 IRAP(16–21)，认为可作为进流起点。
 * 与 demo/src/h265_decoder.cc 中 seq_header 判定一致。
 */
bool H265Decoder::hevc_au_has_entry_point(const uint8_t *d, size_t n) {
  if (!d || n < 5)
    return false;
  size_t i = 0;
  while (i + 3 < n) {
    size_t sc = 0;
    if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1)
      sc = 3;
    else if (i + 4 <= n && d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 0 &&
             d[i + 3] == 1)
      sc = 4;
    else {
      ++i;
      continue;
    }
    size_t nh = i + sc;
    if (nh >= n)
      break;
    int nut = (d[nh] >> 1) & 0x3F;
    if (nut >= 32 && nut <= 34)
      return true;
    if (nut >= 16 && nut <= 21)
      return true;
    i = nh + 1;
  }
  return false;
}

bool H265Decoder::init() {
  reset();

  codec_ = avcodec_find_decoder(AV_CODEC_ID_HEVC);
  if (!codec_) {
    std::cerr << "[H265Decoder] HEVC codec not found" << std::endl;
    return false;
  }

  codec_ctx_ = avcodec_alloc_context3(codec_);
  if (!codec_ctx_) {
    std::cerr << "[H265Decoder] Failed to allocate codec context" << std::endl;
    return false;
  }

  codec_ctx_->thread_count = 1;
#if defined(AV_EF_IGNORE_ERR)
  codec_ctx_->err_recognition |= AV_EF_IGNORE_ERR;
#endif
#if defined(AV_CODEC_FLAG2_SHOW_ALL)
  codec_ctx_->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
#endif

  if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
    std::cerr << "[H265Decoder] Failed to open codec" << std::endl;
    avcodec_free_context(&codec_ctx_);
    return false;
  }

  frame_ = av_frame_alloc();
  packet_ = av_packet_alloc();

  if (!frame_ || !packet_) {
    std::cerr << "[H265Decoder] Failed to allocate frame/packet" << std::endl;
    reset();
    return false;
  }

  entry_point_ok_ = !require_entry_point_;
  stats_ = H265DecodeStats{};
  stats_.sync_established = entry_point_ok_;

  initialized_ = true;
  return true;
}

void H265Decoder::flush_and_resync_gate() {
  if (codec_ctx_)
    avcodec_flush_buffers(codec_ctx_);
  entry_point_ok_ = !require_entry_point_;
  stats_.sync_established = entry_point_ok_;
}

void H265Decoder::reset() {
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }
  if (packet_) {
    av_packet_free(&packet_);
    packet_ = nullptr;
  }
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
    codec_ctx_ = nullptr;
  }
  initialized_ = false;
  width_ = 0;
  height_ = 0;
  src_fmt_ = AV_PIX_FMT_NONE;
  entry_point_ok_ = false;
  stats_ = H265DecodeStats{};
}

static AVPixelFormat normalize_pix_fmt(AVPixelFormat fmt) {
  switch (fmt) {
  case AV_PIX_FMT_YUVJ420P:
    return AV_PIX_FMT_YUV420P;
  case AV_PIX_FMT_YUVJ422P:
    return AV_PIX_FMT_YUV422P;
  case AV_PIX_FMT_YUVJ444P:
    return AV_PIX_FMT_YUV444P;
  default:
    return fmt;
  }
}

bool H265Decoder::decode(const uint8_t *data, size_t size, cv::Mat &bgr_out) {
  if (!initialized_) {
    log_hevc_decode_limited("decode: not initialized");
    return false;
  }
  if (!data || size == 0) {
    log_hevc_decode_limited("decode: empty payload");
    return false;
  }

  stats_.packets_total++;

  std::vector<uint8_t> annex_scratch;
  const uint8_t *feed = data;
  size_t feed_size = size;
  if (!annex_b_starts(data, size)) {
    annex_scratch.reserve(size + 4);
    annex_scratch.push_back(0);
    annex_scratch.push_back(0);
    annex_scratch.push_back(0);
    annex_scratch.push_back(1);
    annex_scratch.insert(annex_scratch.end(), data, data + size);
    feed = annex_scratch.data();
    feed_size = annex_scratch.size();
  }

  if (require_entry_point_) {
    if (!entry_point_ok_) {
      if (!hevc_au_has_entry_point(feed, feed_size)) {
        stats_.packets_skipped_no_entry++;
        if (stats_.packets_skipped_no_entry <= 3u ||
            (stats_.packets_skipped_no_entry % 50u) == 0u) {
          std::cerr << "[H265Decoder] gate: skip AU (no VPS/SPS/PPS/IRAP yet), n="
                    << stats_.packets_skipped_no_entry << " bytes=" << feed_size
                    << std::endl;
        }
        return false;
      }
      entry_point_ok_ = true;
      stats_.sync_established = true;
    }
  }

  av_packet_unref(packet_);
  if (av_new_packet(packet_, static_cast<int>(feed_size)) < 0) {
    stats_.send_fail++;
    std::snprintf(stats_.last_err, sizeof(stats_.last_err), "av_new_packet failed");
    log_hevc_decode_limited("av_new_packet failed");
    return false;
  }
  memcpy(packet_->data, feed, feed_size);

  int ret = avcodec_send_packet(codec_ctx_, packet_);
  if (ret < 0) {
    stats_.send_fail++;
    av_strerror(ret, stats_.last_err, sizeof(stats_.last_err));
    return false;
  }

  bool got = false;
  for (;;) {
    ret = avcodec_receive_frame(codec_ctx_, frame_);
    if (ret == 0) {
      if (convert_frame_to_bgr(frame_, bgr_out)) {
        got = true;
        stats_.frames_out++;
      }
    } else if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    } else {
      stats_.recv_fail++;
      av_strerror(ret, stats_.last_err, sizeof(stats_.last_err));
      break;
    }
  }

  return got;
}

bool H265Decoder::convert_frame_to_bgr(AVFrame *frame, cv::Mat &bgr_out) {
  int w = frame->width;
  int h = frame->height;
  if (w <= 0 || h <= 0)
    return false;

  AVPixelFormat fmt =
      normalize_pix_fmt(static_cast<AVPixelFormat>(frame->format));

  bool need_rebuild =
      !sws_ctx_ || w != width_ || h != height_ || fmt != src_fmt_;
  if (need_rebuild) {
    if (sws_ctx_)
      sws_freeContext(sws_ctx_);
    sws_ctx_ = sws_getContext(w, h, fmt, w, h, AV_PIX_FMT_BGR24, SWS_BILINEAR,
                              nullptr, nullptr, nullptr);
    if (!sws_ctx_)
      return false;
    width_ = w;
    height_ = h;
    src_fmt_ = fmt;
    std::cout << "[H265Decoder] Frame " << w << "x" << h
              << "  pix_fmt=" << av_get_pix_fmt_name(fmt) << std::endl;
  }

  bgr_out.create(h, w, CV_8UC3);
  uint8_t *dst_data[1] = {bgr_out.data};
  int dst_linesize[1] = {static_cast<int>(bgr_out.step[0])};

  sws_scale(sws_ctx_, frame->data, frame->linesize, 0, h, dst_data,
            dst_linesize);
  return true;
}
