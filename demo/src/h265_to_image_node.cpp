#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef FOXGLOVE_MSGS_FOUND
#include <foxglove_msgs/msg/compressed_video.hpp>
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

class H265ToImageNode : public rclcpp::Node {
public:
    H265ToImageNode() : Node("h265_to_image_node") {
        this->declare_parameter<std::string>("input_topic", "/image_left_raw/h265_undistort");
        this->declare_parameter<std::string>("output_topic", "/camera/color/image_raw");
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        init_decoder();

#ifdef FOXGLOVE_MSGS_FOUND
        subscription_ = this->create_subscription<foxglove_msgs::msg::CompressedVideo>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&H265ToImageNode::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "H265 to Image Node initialized. Subscribing to %s, Publishing to %s", input_topic.c_str(), output_topic.c_str());
#else
        RCLCPP_ERROR(this->get_logger(), "foxglove_msgs not found, this node requires it to subscribe to CompressedVideo.");
#endif
    }

    ~H265ToImageNode() {
        if (codec_ctx_) avcodec_free_context(&codec_ctx_);
        if (frame_) av_frame_free(&frame_);
        if (packet_) av_packet_free(&packet_);
        if (sws_ctx_) sws_freeContext(sws_ctx_);
    }

private:
    void init_decoder() {
        const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
        if (!codec) {
            RCLCPP_ERROR(this->get_logger(), "HEVC decoder not found");
            throw std::runtime_error("HEVC decoder not found");
        }
        
        codec_ctx_ = avcodec_alloc_context3(codec);
        if (!codec_ctx_) {
            throw std::runtime_error("Could not allocate codec context");
        }
        
        codec_ctx_->thread_count = 4; // Multi-threading for faster software decoding
        
        if (avcodec_open2(codec_ctx_, codec, NULL) < 0) {
            throw std::runtime_error("Could not open codec");
        }
        
        frame_ = av_frame_alloc();
        packet_ = av_packet_alloc();
    }

#ifdef FOXGLOVE_MSGS_FOUND
    void topic_callback(const foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
        if (msg->data.empty()) return;
        
        // Try to handle missing Annex B
        std::vector<uint8_t> data;
        const uint8_t* nalu_data = msg->data.data();
        size_t nalu_size = msg->data.size();
        
        if (nalu_size > 4 && 
            !(nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 0 && nalu_data[3] == 1) &&
            !(nalu_data[0] == 0 && nalu_data[1] == 0 && nalu_data[2] == 1)) {
            data.reserve(nalu_size + 4);
            data.push_back(0);
            data.push_back(0);
            data.push_back(0);
            data.push_back(1);
            data.insert(data.end(), msg->data.begin(), msg->data.end());
            
            packet_->data = data.data();
            packet_->size = data.size();
        } else {
            packet_->data = const_cast<uint8_t*>(nalu_data);
            packet_->size = nalu_size;
        }

        int ret = avcodec_send_packet(codec_ctx_, packet_);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending a packet for decoding");
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            } else if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error during decoding");
                return;
            }

            if (!sws_ctx_) {
                // 目标分辨率设定为解码出的原始分辨率的 1/4 (长宽各缩放一半)
                int target_width = frame_->width / 2;
                int target_height = frame_->height / 2;

                // 保证宽和高为偶数
                target_width = target_width - (target_width % 2);
                target_height = target_height - (target_height % 2);

                sws_ctx_ = sws_getContext(
                    frame_->width, frame_->height, codec_ctx_->pix_fmt,
                    target_width, target_height, AV_PIX_FMT_BGR24,
                    SWS_FAST_BILINEAR, NULL, NULL, NULL
                );
                
                target_width_ = target_width;
                target_height_ = target_height;
            }

            auto out_msg = std::make_shared<sensor_msgs::msg::Image>();
            out_msg->header.stamp = msg->timestamp;
            out_msg->header.frame_id = msg->frame_id;
            out_msg->height = target_height_;
            out_msg->width = target_width_;
            out_msg->encoding = "bgr8";
            out_msg->is_bigendian = false;
            out_msg->step = target_width_ * 3;
            
            size_t size = out_msg->step * out_msg->height;
            out_msg->data.resize(size);

            uint8_t *dest[4] = { out_msg->data.data(), NULL, NULL, NULL };
            int dest_linesize[4] = { static_cast<int>(out_msg->step), 0, 0, 0 };

            // 使用 sws_scale 进行缩放并转换色彩空间
            sws_scale(sws_ctx_, frame_->data, frame_->linesize, 0, frame_->height,
                      dest, dest_linesize);

            publisher_->publish(*out_msg);
        }
    }
#endif

    AVCodecContext *codec_ctx_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *packet_ = nullptr;
    struct SwsContext *sws_ctx_ = nullptr;
    int target_width_ = 0;
    int target_height_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
#ifdef FOXGLOVE_MSGS_FOUND
    rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr subscription_;
#endif
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<H265ToImageNode>());
    rclcpp::shutdown();
    return 0;
}
