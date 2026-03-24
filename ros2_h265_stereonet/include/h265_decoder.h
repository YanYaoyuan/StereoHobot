#ifndef ROS2_H265_STEREONET_H265_DECODER_H_
#define ROS2_H265_STEREONET_H265_DECODER_H_

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

/** 累计统计，供节点侧打日志（节流） */
struct H265DecodeStats {
  uint64_t packets_total = 0;
  /** 在建立随机接入点前丢弃的 AU（避免 GOP 中途进流导致 POC 参考缺失） */
  uint64_t packets_skipped_no_entry = 0;
  uint64_t frames_out = 0;
  uint64_t send_fail = 0;
  uint64_t recv_fail = 0;
  bool sync_established = false;
  char last_err[128]{};
};

/**
 * @brief Stateful H.265 (HEVC) decoder backed by FFmpeg libavcodec.
 *
 * Each instance maintains its own codec context so that left and right camera
 * streams can be decoded independently.  The caller feeds Annex-B formatted
 * access-units (one per foxglove_msgs/CompressedVideo message) via decode(),
 * and receives a BGR cv::Mat when a frame is successfully decoded.
 *
 * 默认启用「随机接入点门控」：与 demo 中硬件解码器思路一致，直到 AU 内出现
 * VPS/SPS/PPS(32–34) 或 IRAP/IDR/CRA 等(16–21) 才开始向 FFmpeg 送包，减轻
 * 「Could not find ref with POC」导致的长期无输出。
 */
class H265Decoder {
public:
  H265Decoder();
  ~H265Decoder();

  H265Decoder(const H265Decoder &) = delete;
  H265Decoder &operator=(const H265Decoder &) = delete;

  bool init();
  void reset();

  /** 是否要求首帧前必须出现 HEVC 随机接入相关 NAL（默认 true） */
  void set_require_hevc_entry_point(bool v) { require_entry_point_ = v; }

  /** 清空解码器缓冲并重新开始门控（例如换源、严重错位时由上层调用） */
  void flush_and_resync_gate();

  H265DecodeStats get_stats() const { return stats_; }

  /**
   * @brief Decode one H.265 access unit (Annex-B).
   * @param data  Raw bytes; 若无起始码会尝试前补 00 00 00 01（与 demo 一致）
   * @param size  Byte count of @p data.
   * @param bgr_out  Decoded frame in BGR24 format.  Untouched on failure.
   * @return true if a frame was produced, false otherwise.
   */
  bool decode(const uint8_t *data, size_t size, cv::Mat &bgr_out);

  int width() const { return width_; }
  int height() const { return height_; }

private:
  bool convert_frame_to_bgr(AVFrame *frame, cv::Mat &bgr_out);
  static bool annex_b_starts(const uint8_t *d, size_t n);
  static bool hevc_au_has_entry_point(const uint8_t *d, size_t n);

  const AVCodec *codec_ = nullptr;
  AVCodecContext *codec_ctx_ = nullptr;
  AVFrame *frame_ = nullptr;
  AVPacket *packet_ = nullptr;
  SwsContext *sws_ctx_ = nullptr;
  AVPixelFormat src_fmt_ = AV_PIX_FMT_NONE;
  int width_ = 0;
  int height_ = 0;
  bool initialized_ = false;
  bool require_entry_point_ = true;
  bool entry_point_ok_ = false;
  H265DecodeStats stats_;
};

#endif // ROS2_H265_STEREONET_H265_DECODER_H_
