#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <csignal>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

#include "BS_thread_pool.hpp"

#include "h265_decoder.h"
#include "stereonet_process.h"
#include "camera_intrinsic.h"
#include "img_convert_utils.h"
#include "feature_epipolar_align.h"
#include "calib_parser.h"

namespace fs = std::filesystem;

// ========================================================================================
// Helpers
// ========================================================================================

static bool readCameraIntrinsicFromFile(const std::string &file_path,
                                        stereonet::CameraIntrinsic &intrinsic) {
  std::ifstream infile(file_path);
  if (!infile.is_open()) return false;

  std::vector<double> values;
  std::string line;
  while (std::getline(infile, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    double v;
    while (ss >> v) values.push_back(v);
  }

  // format 1: fx fy cx cy baseline
  if (values.size() == 5) {
    intrinsic.fx = values[0];
    intrinsic.fy = values[1];
    intrinsic.cx = values[2];
    intrinsic.cy = values[3];
    intrinsic.baseline = values[4];
    return intrinsic.is_valid();
  }
  // format 2: 3x3 K + baseline
  if (values.size() == 10) {
    intrinsic.fx = values[0];
    intrinsic.cx = values[2];
    intrinsic.fy = values[4];
    intrinsic.cy = values[5];
    intrinsic.baseline = values[9];
    return intrinsic.is_valid();
  }
  return false;
}

static void saveCameraIntrinsic(const std::string &dir,
                                const stereonet::CameraIntrinsic &intr) {
  fs::create_directories(dir);
  {
    std::ofstream f(dir + "/camera_intrinsic.txt");
    f << "# fx fy cx cy baseline(m)\n";
    f << intr.fx << " " << intr.fy << " " << intr.cx << " " << intr.cy << " "
      << intr.baseline << std::endl;
  }
  {
    std::ofstream f(dir + "/K.txt");
    f << intr.fx << " 0.0 " << intr.cx << " 0.0 " << intr.fy << " "
      << intr.cy << " 0.0 0.0 1.0\n";
    f << intr.baseline << std::endl;
  }
}


static int64_t stamp_to_ns(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

static bool cv_mat3x3_finite(const cv::Mat &m) {
  if (m.empty() || m.rows != 3 || m.cols != 3 || m.type() != CV_64F)
    return false;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      if (!std::isfinite(m.at<double>(r, c))) return false;
  return true;
}

/** Scale pinhole K: independent scale_w / scale_h so (cx,cy) stay inside curr image when aspect matches calib. */
static cv::Mat scaled_camera_matrix_wh(const cv::Mat &K_calib, double scale_w, double scale_h) {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = K_calib.at<double>(0, 0) * scale_w;
  K.at<double>(1, 1) = K_calib.at<double>(1, 1) * scale_h;
  K.at<double>(0, 2) = K_calib.at<double>(0, 2) * scale_w;
  K.at<double>(1, 2) = K_calib.at<double>(1, 2) * scale_h;
  return K;
}

/**
 * initUndistortRectifyMap 在部分 OpenCV 版本上，对「8 参有理畸变 + 极大 k1」会在 C++ 层直接 SIGSEGV。
 * 默认只用前 N 项（Brown k1,k2,p1,p2,k3），几何略差于完整 8 参，但能稳定建图。
 * max_coeffs <= 0：使用 YAML 中的完整 D。
 */
static cv::Mat slice_distortion_for_undistort(const cv::Mat &d_in, int max_coeffs) {
  if (d_in.empty()) return d_in;
  const int total = static_cast<int>(d_in.total());
  int use = total;
  if (max_coeffs > 0 && max_coeffs < total)
    use = max_coeffs;
  if (use == total)
    return d_in;
  cv::Mat col = d_in.reshape(1, total);
  return col.rowRange(0, use).clone().reshape(1, use);
}

// ========================================================================================
// Node
// ========================================================================================

class StereoH265Node : public rclcpp::Node {
public:
  explicit StereoH265Node(const rclcpp::NodeOptions &options)
      : Node("stereo_h265_node", options) {
    load_parameters();
    init();
  }

  ~StereoH265Node() override {
    shutdown_ = true;
    sync_cv_.notify_all();
    for (auto &t : process_threads_) {
      if (t.joinable()) t.join();
    }
    if (postprocess_thread_pool_ptr_) {
      postprocess_thread_pool_ptr_->wait();
      postprocess_thread_pool_ptr_.reset();
    }
    if (save_thread_pool_ptr_) {
      save_thread_pool_ptr_->wait();
      save_thread_pool_ptr_.reset();
    }
    stereonet_process_.reset();
    RCLCPP_INFO(get_logger(), "Node shutdown complete, processed %d frames",
                frame_count_.load());
  }

private:
  // ====================== parameter declaration / loading =======================

  template <typename T>
  T get_or_declare_parameter(const std::string &name, const T &default_value) {
    if (has_parameter(name)) {
      return get_parameter(name).get_value<T>();
    }
    return declare_parameter<T>(name, default_value);
  }

  void load_parameters() {
    model_path_ = get_or_declare_parameter<std::string>("model_path", "./model/DStereoV2.4_int16.bin");
    post_version_ = get_or_declare_parameter<std::string>("post_version", "auto");
    left_topic_ = get_or_declare_parameter<std::string>("left_topic", "/image_left_raw/h265");
    right_topic_ = get_or_declare_parameter<std::string>("right_topic", "/image_right_raw/h265");
    uncertainty_th_ = get_or_declare_parameter<double>("uncertainty_th", -0.10);
    result_dir_ = get_or_declare_parameter<std::string>("result_dir", "./result");
    sync_tolerance_ns_ = static_cast<int64_t>(get_or_declare_parameter<double>("sync_tolerance_ms", 10.0) * 1e6);
    max_buffer_size_ = get_or_declare_parameter<int>("max_buffer_size", 10);
    infer_thread_num_ = get_or_declare_parameter<int>("infer_thread_num", 1);
    save_freq_ = get_or_declare_parameter<int>("save_freq", 1);
    max_frames_ = get_or_declare_parameter<int>("max_frames", -1);
    save_results_ = get_or_declare_parameter<bool>("save_results", true);
    save_visual_ = get_or_declare_parameter<bool>("save_visual", true);
    save_pcd_ = get_or_declare_parameter<bool>("save_pcd", true);
    save_disp_ = get_or_declare_parameter<bool>("save_disp", true);
    save_depth_ = get_or_declare_parameter<bool>("save_depth", true);
    save_uncert_ = get_or_declare_parameter<bool>("save_uncert", false);
    save_epipolar_ = get_or_declare_parameter<bool>("save_epipolar", false);

    publish_depth_ = get_or_declare_parameter<bool>("publish_depth", true);
    publish_pointcloud_ = get_or_declare_parameter<bool>("publish_pointcloud", true);
    publish_visual_ = get_or_declare_parameter<bool>("publish_visual", false);
    publish_disp_ = get_or_declare_parameter<bool>("publish_disp", false);
    depth_topic_ = get_or_declare_parameter<std::string>("depth_topic", "~/stereonet_depth");
    pointcloud_topic_ = get_or_declare_parameter<std::string>("pointcloud_topic", "~/stereonet_pointcloud2");
    visual_topic_ = get_or_declare_parameter<std::string>("visual_topic", "~/stereonet_visual");
    disp_topic_ = get_or_declare_parameter<std::string>("disp_topic", "~/stereonet_disp");
    frame_id_ = get_or_declare_parameter<std::string>("frame_id", "camera_link");
    pc_downsample_step_ = get_or_declare_parameter<int>("pointcloud_downsample_step", 2);
    pc_depth_max_ = get_or_declare_parameter<double>("pointcloud_depth_max", 5.0);
    undistort_use_stereo_rectify_ =
        get_or_declare_parameter<bool>("undistort_use_stereo_rectify", false);
    // 0 = use full D from YAML; 5 = Brown 5 项，规避极端 8 参有理模型在 initUndistortRectifyMap 内崩溃
    undistort_dist_coeff_count_ =
        get_or_declare_parameter<int>("undistort_dist_coeff_count", 5);
    undistort_force_no_distortion_ =
        get_or_declare_parameter<bool>("undistort_force_no_distortion", false);
    // <=0：不启用；任一眼 |k1| 超过阈值则左右眼均改用零 D 建图（避免极大 k1 在 OpenCV 内仍 SIGSEGV）
    undistort_extreme_k1_threshold_ =
        get_or_declare_parameter<double>("undistort_extreme_k1_threshold", 2.0);

    // Camera intrinsic loading (priority: calib_yaml > intrinsic_file > params)
    bool intrinsic_loaded = false;

    std::string calib_yaml = get_or_declare_parameter<std::string>("calib_yaml_file", "");
    
    // Attempt to resolve relative paths using package share directory
    if (!calib_yaml.empty() && !fs::exists(calib_yaml)) {
      try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("ros2_h265_stereonet");
        std::string resolved_path = pkg_share + "/" + calib_yaml;
        if (fs::exists(resolved_path)) {
          calib_yaml = resolved_path;
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Failed to resolve package share directory: %s", e.what());
      }
    }

    if (!calib_yaml.empty() && fs::exists(calib_yaml)) {
      CameraCalib left_cam, right_cam;
      if (parseFullStereoCalibYaml(calib_yaml, left_cam, right_cam)) {
        // Extr_B_C: camera → body. Relative pose left → right:
        // P_right = R_R^T R_L P_left + R_R^T (t_L - t_R)
        cv::Mat R = right_cam.R_BC.t() * left_cam.R_BC;
        cv::Mat T = right_cam.R_BC.t() * cv::Mat(left_cam.T_BC - right_cam.T_BC);
        T.convertTo(T, CV_64F);
        const double baseline_from_T = cv::norm(T);
        const double tx_abs = std::abs(T.at<double>(0, 0));
        stereo_baseline_m_ = (tx_abs > 1e-4) ? tx_abs : baseline_from_T;

        calib_ref_w_ = left_cam.width > 0 ? left_cam.width : 1920;
        calib_ref_h_ = left_cam.height > 0 ? left_cam.height : 1080;
        calib_K_left_ = left_cam.K.clone();
        calib_D_left_ = left_cam.D.clone();
        calib_K_right_ = right_cam.K.clone();
        calib_D_right_ = right_cam.D.clone();
        undistort_maps_cache_w_ = 0;
        undistort_maps_cache_h_ = 0;
        left_map1_.release();
        left_map2_.release();
        right_map1_.release();
        right_map2_.release();

        if (!undistort_use_stereo_rectify_) {
          // Per-eye undistort: scale K by curr_width vs calib width, adjust cy for vertical
          // center crop; D unchanged; initUndistortRectifyMap(K_scaled, D, I, K_scaled, size).
          undistort_scaled_k_mode_ = true;
          use_rectification_ = true;
          camera_intrinsic_.fx = calib_K_left_.at<double>(0, 0);
          camera_intrinsic_.fy = calib_K_left_.at<double>(1, 1);
          camera_intrinsic_.cx = calib_K_left_.at<double>(0, 2);
          camera_intrinsic_.cy = calib_K_left_.at<double>(1, 2);
          camera_intrinsic_.baseline = stereo_baseline_m_;
          RCLCPP_INFO(get_logger(),
                      "Calib YAML: scaled-K undistort (see undistort tutorial). "
                      "Ref %dx%d; remap maps match first decoded frame size. baseline=%.5f m. "
                      "undistort_dist_coeff_count=%d (0=full D from YAML); "
                      "extreme_k1_thresh=%.3f force_no_dist=%s",
                      calib_ref_w_, calib_ref_h_, stereo_baseline_m_, undistort_dist_coeff_count_,
                      undistort_extreme_k1_threshold_, undistort_force_no_distortion_ ? "true" : "false");
          intrinsic_loaded = true;
        } else {
          undistort_scaled_k_mode_ = false;
          cv::Mat R1, R2, P1, P2, Q;
          cv::Size img_size(calib_ref_w_, calib_ref_h_);
          cv::Mat Dl_sr = slice_distortion_for_undistort(left_cam.D, undistort_dist_coeff_count_);
          cv::Mat Dr_sr = slice_distortion_for_undistort(right_cam.D, undistort_dist_coeff_count_);
          if (Dl_sr.total() != left_cam.D.total() || Dr_sr.total() != right_cam.D.total()) {
            RCLCPP_WARN(get_logger(),
                        "stereoRectify path: using truncated D (undistort_dist_coeff_count=%d) for stability",
                        undistort_dist_coeff_count_);
          }
          applyUndistortDistortionSafety(Dl_sr, Dr_sr, "stereoRectify path");

          cv::stereoRectify(left_cam.K, Dl_sr, right_cam.K, Dr_sr,
                            img_size, R, T, R1, R2, P1, P2, Q,
                            cv::CALIB_ZERO_DISPARITY, 0, img_size);

          bool stereo_ok =
              cv_mat3x3_finite(P1(cv::Rect(0, 0, 3, 3))) &&
              std::isfinite(P1.at<double>(0, 2)) && std::isfinite(P1.at<double>(1, 2)) &&
              std::isfinite(P2.at<double>(0, 3)) && std::isfinite(P1.at<double>(0, 0)) &&
              std::abs(P1.at<double>(0, 0)) > 1e-6;

          if (!stereo_ok) {
            cv::stereoRectify(left_cam.K, Dl_sr, right_cam.K, Dr_sr,
                              img_size, R, T, R1, R2, P1, P2, Q,
                              cv::CALIB_ZERO_DISPARITY, 1, img_size);
            stereo_ok =
                cv_mat3x3_finite(P1(cv::Rect(0, 0, 3, 3))) &&
                std::isfinite(P1.at<double>(0, 2)) && std::isfinite(P1.at<double>(1, 2)) &&
                std::isfinite(P2.at<double>(0, 3)) && std::isfinite(P1.at<double>(0, 0)) &&
                std::abs(P1.at<double>(0, 0)) > 1e-6;
          }

          if (stereo_ok) {
            RCLCPP_INFO(get_logger(),
                        "Stereo rectification OK (stereoRectify). baseline≈%.5f m from P2",
                        std::abs(P2.at<double>(0, 3) / P1.at<double>(0, 0)));
          } else {
            RCLCPP_WARN(get_logger(),
                        "stereoRectify failed; falling back to undistort at calib resolution.");
            R1 = cv::Mat::eye(3, 3, CV_64F);
            R2 = cv::Mat::eye(3, 3, CV_64F);
            P1 = left_cam.K.clone();
            P2 = right_cam.K.clone();
            cv::Rect roi1, roi2;
            cv::Mat new1 = cv::getOptimalNewCameraMatrix(
                left_cam.K, Dl_sr, img_size, 1.0, img_size, &roi1);
            cv::Mat new2 = cv::getOptimalNewCameraMatrix(
                right_cam.K, Dr_sr, img_size, 1.0, img_size, &roi2);
            if (cv_mat3x3_finite(new1) && cv_mat3x3_finite(new2) &&
                std::isfinite(new1.at<double>(0, 2)) &&
                std::isfinite(new1.at<double>(1, 2))) {
              P1 = new1;
              P2 = new2;
            }
            camera_intrinsic_.baseline =
                (tx_abs > 1e-4) ? tx_abs : baseline_from_T;
          }

          // CV_32FC1：map1=map_x map2=map_y；强畸变/8 参 D 下 CV_16SC2 建图易数值溢出导致崩溃
          bool stereo_maps_built = false;
          {
            const int prev_ocv_threads = cv::getNumThreads();
            cv::setNumThreads(1);
            try {
              cv::initUndistortRectifyMap(left_cam.K, Dl_sr, R1, P1, img_size, CV_32FC1,
                                          left_map1_, left_map2_);
              cv::initUndistortRectifyMap(right_cam.K, Dr_sr, R2, P2, img_size, CV_32FC1,
                                          right_map1_, right_map2_);
              stereo_maps_built = true;
            } catch (const cv::Exception &e) {
              RCLCPP_ERROR(get_logger(), "initUndistortRectifyMap (stereoRectify path) failed: %s",
                           e.what());
              left_map1_.release();
              left_map2_.release();
              right_map1_.release();
              right_map2_.release();
            }
            cv::setNumThreads(prev_ocv_threads);
          }

          if (stereo_maps_built) {
            use_rectification_ = true;

            camera_intrinsic_.fx = P1.at<double>(0, 0);
            camera_intrinsic_.fy = P1.at<double>(1, 1);
            camera_intrinsic_.cx = P1.at<double>(0, 2);
            camera_intrinsic_.cy = P1.at<double>(1, 2);
            if (stereo_ok && std::isfinite(P2.at<double>(0, 3)) &&
                std::abs(P1.at<double>(0, 0)) > 1e-6) {
              camera_intrinsic_.baseline =
                  std::abs(P2.at<double>(0, 3) / P1.at<double>(0, 0));
            }
            if (!camera_intrinsic_.is_valid() || !std::isfinite(camera_intrinsic_.fx)) {
              camera_intrinsic_.fx = left_cam.K.at<double>(0, 0);
              camera_intrinsic_.fy = left_cam.K.at<double>(1, 1);
              camera_intrinsic_.cx = left_cam.K.at<double>(0, 2);
              camera_intrinsic_.cy = left_cam.K.at<double>(1, 2);
              camera_intrinsic_.baseline = baseline_from_T;
            }

            RCLCPP_INFO(get_logger(),
                        "Loaded stereo calibration YAML & stereoRectify path: %s  →  "
                        "[fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, baseline=%.5f m]",
                        calib_yaml.c_str(), camera_intrinsic_.fx,
                        camera_intrinsic_.fy, camera_intrinsic_.cx,
                        camera_intrinsic_.cy, camera_intrinsic_.baseline);
            intrinsic_loaded = true;
          } else {
            use_rectification_ = false;
            RCLCPP_ERROR(get_logger(),
                         "Stereo rectify maps not built; will fall back to intrinsic_file / fx,fy,cx,cy,baseline "
                         "params if provided.");
          }
        }
      } else {
        RCLCPP_ERROR(get_logger(),
                     "Failed to parse calib YAML: %s  (check format)",
                     calib_yaml.c_str());
      }
    }

    std::string intrinsic_file = get_or_declare_parameter<std::string>("intrinsic_file", "");
    if (!intrinsic_loaded) {
      if (!intrinsic_file.empty() && fs::exists(intrinsic_file)) {
        if (readCameraIntrinsicFromFile(intrinsic_file, camera_intrinsic_)) {
          intrinsic_loaded = true;
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to read intrinsic file: %s",
                       intrinsic_file.c_str());
        }
      }
    }

    double fx = get_or_declare_parameter<double>("fx", 0.0);
    double fy = get_or_declare_parameter<double>("fy", 0.0);
    double cx = get_or_declare_parameter<double>("cx", 0.0);
    double cy = get_or_declare_parameter<double>("cy", 0.0);
    double baseline = get_or_declare_parameter<double>("baseline", 0.0);

    if (!intrinsic_loaded) {
      camera_intrinsic_.fx = fx;
      camera_intrinsic_.fy = fy;
      camera_intrinsic_.cx = cx;
      camera_intrinsic_.cy = cy;
      camera_intrinsic_.baseline = baseline;
    }

    // HEVC(FFmpeg)：与 demo 硬件解码类似，默认丢弃 GOP 中途的 AU，直到出现 VPS/SPS/PPS 或 IRAP
    hevc_require_entry_point_ =
        get_or_declare_parameter<bool>("hevc_require_entry_point", true);
  }

  // ====================== initialisation ========================================

  void init() {
    if (save_results_) fs::create_directories(result_dir_);

    // H.265 decoders (one per camera)
    left_decoder_ = std::make_unique<H265Decoder>();
    right_decoder_ = std::make_unique<H265Decoder>();
    if (!left_decoder_->init() || !right_decoder_->init()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialise H.265 decoder");
      throw std::runtime_error("H.265 decoder init failed");
    }
    left_decoder_->set_require_hevc_entry_point(hevc_require_entry_point_);
    right_decoder_->set_require_hevc_entry_point(hevc_require_entry_point_);
    RCLCPP_INFO(get_logger(),
                "H265 FFmpeg decoder: hevc_require_entry_point=%s "
                "(wait for VPS/SPS/PPS or IRAP NALs before feeding; reduces POC ref errors)",
                hevc_require_entry_point_ ? "true" : "false");

    // StereoNet inference engine
    stereonet_process_ =
        std::make_shared<stereonet::StereonetProcess>(get_logger());
    if (stereonet_process_->init(model_path_, post_version_) != 0) {
      RCLCPP_FATAL(get_logger(), "Failed to load model: %s",
                   model_path_.c_str());
      throw std::runtime_error("StereoNet model init failed");
    }
    // Get model input size and fix potential width/height swap.
    // Some models/snapshots report (h,w) but the node assumes (w,h) when
    // calling cv::resize and when computing NV12 size.
    int raw_model_w = 0;
    int raw_model_h = 0;
    stereonet_process_->get_model_input_size(raw_model_w, raw_model_h);

    // If the model input is expected to be landscape (w > h) but we got it
    // swapped, fix it. (e.g. expected 640x352 vs reported 352x640)
    if (raw_model_w > 0 && raw_model_h > 0 && raw_model_w < raw_model_h) {
      std::swap(raw_model_w, raw_model_h);
    }

    // Safety: avoid negative/overflow dimensions creating huge allocations.
    if (raw_model_w <= 0 || raw_model_h <= 0 || raw_model_w > 10000 || raw_model_h > 10000) {
      RCLCPP_FATAL(get_logger(),
                   "Invalid model input size after normalization (w=%d, h=%d). Abort to avoid huge allocations.",
                   raw_model_w, raw_model_h);
      throw std::runtime_error("Invalid model input size");
    }
    model_input_w_ = raw_model_w;
    model_input_h_ = raw_model_h;
    RCLCPP_INFO(get_logger(), "Model input size: %d x %d", model_input_w_, model_input_h_);

    if (camera_intrinsic_.is_valid()) {
      RCLCPP_INFO(get_logger(),
                  "Camera intrinsic [fx,fy,cx,cy,baseline]: %.2f, %.2f, "
                  "%.2f, %.2f, %.4f",
                  camera_intrinsic_.fx, camera_intrinsic_.fy,
                  camera_intrinsic_.cx, camera_intrinsic_.cy,
                  camera_intrinsic_.baseline);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Camera intrinsic INVALID (fx=%.2f fy=%.2f cx=%.2f "
                   "cy=%.2f baseline=%.4f). "
                   "depth / visual / pointcloud will ALL be SKIPPED! "
                   "Set calib_yaml_file, intrinsic_file, or fx/fy/cx/cy/baseline params.",
                   camera_intrinsic_.fx, camera_intrinsic_.fy,
                   camera_intrinsic_.cx, camera_intrinsic_.cy,
                   camera_intrinsic_.baseline);
    }

    // ROS publishers
    if (publish_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, 5);
      RCLCPP_INFO(get_logger(), "Publishing depth on: %s", depth_topic_.c_str());
    }
    if (publish_pointcloud_) {
      pointcloud_pub_ =
          create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 5);
      RCLCPP_INFO(get_logger(), "Publishing pointcloud on: %s",
                  pointcloud_topic_.c_str());
    }
    if (publish_visual_) {
      visual_pub_ =
          create_publisher<sensor_msgs::msg::Image>(visual_topic_, 5);
      RCLCPP_INFO(get_logger(), "Publishing visual on: %s",
                  visual_topic_.c_str());
    }
    if (publish_disp_) {
      disp_pub_ = create_publisher<sensor_msgs::msg::Image>(disp_topic_, 5);
      RCLCPP_INFO(get_logger(), "Publishing disp on: %s", disp_topic_.c_str());
    }

    // Processing threads (keeps callbacks non-blocking)
    for (int i = 0; i < infer_thread_num_; ++i) {
      process_threads_.emplace_back(&StereoH265Node::process_loop, this);
    }
    postprocess_thread_pool_ptr_ = std::make_unique<BS::thread_pool<>>(1);
    save_thread_pool_ptr_ = std::make_unique<BS::thread_pool<>>(1);

    // ROS subscriptions
    auto qos = rclcpp::SensorDataQoS();
    left_sub_ = create_subscription<foxglove_msgs::msg::CompressedVideo>(
        left_topic_, qos,
        std::bind(&StereoH265Node::left_callback, this,
                  std::placeholders::_1));
    right_sub_ = create_subscription<foxglove_msgs::msg::CompressedVideo>(
        right_topic_, qos,
        std::bind(&StereoH265Node::right_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to left: %s", left_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to right: %s", right_topic_.c_str());
  }

  // ====================== subscription callbacks ================================

  void left_callback(
      const foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
    cv::Mat bgr;
    const size_t paylen = msg->data.size();
    if (!left_decoder_->decode(msg->data.data(), paylen, bgr)) {
      H265DecodeStats st = left_decoder_->get_stats();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Left H265: no BGR this packet (payload=%zu B). "
          "sync_established=%s skipped_gate=%lu total_pkts=%lu bgr_ok=%lu "
          "send_fail=%lu recv_fail=%lu last_err=[%s]",
          paylen, st.sync_established ? "yes" : "no",
          static_cast<unsigned long>(st.packets_skipped_no_entry),
          static_cast<unsigned long>(st.packets_total),
          static_cast<unsigned long>(st.frames_out),
          static_cast<unsigned long>(st.send_fail),
          static_cast<unsigned long>(st.recv_fail),
          st.last_err[0] ? st.last_err : "-");
      return;
    }
    int64_t ts = stamp_to_ns(msg->timestamp);
    {
      std::lock_guard<std::mutex> lk(sync_mtx_);
      left_frames_[ts] = bgr;
      evict_old_locked(left_frames_);
    }
    sync_cv_.notify_one();
  }

  void right_callback(
      const foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
    cv::Mat bgr;
    const size_t paylen = msg->data.size();
    if (!right_decoder_->decode(msg->data.data(), paylen, bgr)) {
      H265DecodeStats st = right_decoder_->get_stats();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Right H265: no BGR this packet (payload=%zu B). "
          "sync_established=%s skipped_gate=%lu total_pkts=%lu bgr_ok=%lu "
          "send_fail=%lu recv_fail=%lu last_err=[%s]",
          paylen, st.sync_established ? "yes" : "no",
          static_cast<unsigned long>(st.packets_skipped_no_entry),
          static_cast<unsigned long>(st.packets_total),
          static_cast<unsigned long>(st.frames_out),
          static_cast<unsigned long>(st.send_fail),
          static_cast<unsigned long>(st.recv_fail),
          st.last_err[0] ? st.last_err : "-");
      return;
    }
    int64_t ts = stamp_to_ns(msg->timestamp);
    {
      std::lock_guard<std::mutex> lk(sync_mtx_);
      right_frames_[ts] = bgr;
      evict_old_locked(right_frames_);
    }
    sync_cv_.notify_one();
  }

  // ====================== frame synchronisation =================================

  void evict_old_locked(std::map<int64_t, cv::Mat> &buf) {
    while (static_cast<int>(buf.size()) > max_buffer_size_)
      buf.erase(buf.begin());
  }

  /**
   * Try to find a matching left-right pair whose timestamps are within
   * sync_tolerance_ns_.  On success the two frames are moved into out_left /
   * out_right and the matched entries (and all older ones) are erased.
   */
  bool try_match_locked(cv::Mat &out_left, cv::Mat &out_right,
                        int64_t &out_ts) {
    for (auto li = left_frames_.begin(); li != left_frames_.end(); ++li) {
      for (auto ri = right_frames_.begin(); ri != right_frames_.end(); ++ri) {
        if (std::abs(li->first - ri->first) <= sync_tolerance_ns_) {
          out_left = li->second;
          out_right = ri->second;
          out_ts = li->first;
          // erase matched and all older entries
          left_frames_.erase(left_frames_.begin(), std::next(li));
          right_frames_.erase(right_frames_.begin(), std::next(ri));
          return true;
        }
      }
    }
    return false;
  }

  /**
   * 极大 k1（如左目 ~8）在部分 OpenCV 上即使用 5 项 D，initUndistortRectifyMap 仍会 SIGSEGV。
   * 强制左右眼使用相同的 D 策略，避免立体输入一边去畸变一边不去。
   */
  void applyUndistortDistortionSafety(cv::Mat &Dl, cv::Mat &Dr, const char *ctx) {
    if (undistort_force_no_distortion_) {
      RCLCPP_WARN(get_logger(), "%s: undistort_force_no_distortion=true → zero D (5×1) for both eyes.", ctx);
      Dl = cv::Mat::zeros(5, 1, CV_64F);
      Dr = cv::Mat::zeros(5, 1, CV_64F);
      return;
    }
    if (undistort_extreme_k1_threshold_ <= 0.0)
      return;
    const auto k1_abs = [](const cv::Mat &d) -> double {
      if (d.empty()) return 0.0;
      return std::abs(d.at<double>(0, 0));
    };
    const bool el = k1_abs(Dl) > undistort_extreme_k1_threshold_;
    const bool er = k1_abs(Dr) > undistort_extreme_k1_threshold_;
    if (el || er) {
      RCLCPP_WARN(get_logger(),
                  "%s: |k1|>%.3f on left=%s right=%s (k1 L=%.4f R=%.4f) → zero D for BOTH eyes "
                  "(avoid OpenCV initUndistortRectifyMap crash). Set undistort_extreme_k1_threshold=0 to "
                  "disable this fallback.",
                  ctx, undistort_extreme_k1_threshold_, el ? "yes" : "no", er ? "yes" : "no", k1_abs(Dl),
                  k1_abs(Dr));
      Dl = cv::Mat::zeros(5, 1, CV_64F);
      Dr = cv::Mat::zeros(5, 1, CV_64F);
    }
  }

  /** Build left/right remap for current decoded resolution (scaled K + fixed D). Caller holds undistort_mtx_. */
  void updateScaledUndistortMapsUnlocked(int curr_w, int curr_h) {
    if (!undistort_scaled_k_mode_)
      return;
    if (curr_w <= 0 || curr_h <= 0) {
      RCLCPP_ERROR(get_logger(),
                   "updateScaledUndistortMapsUnlocked: skip invalid curr size %dx%d",
                   curr_w, curr_h);
      return;
    }
    if (calib_ref_w_ <= 0 || calib_ref_h_ <= 0) {
      RCLCPP_ERROR(get_logger(),
                   "updateScaledUndistortMapsUnlocked: skip invalid calib_ref %dx%d",
                   calib_ref_w_, calib_ref_h_);
      return;
    }
    if (curr_w == undistort_maps_cache_w_ && curr_h == undistort_maps_cache_h_ &&
        !left_map1_.empty()) {
      RCLCPP_DEBUG(get_logger(),
                   "updateScaledUndistortMapsUnlocked: reuse cached maps %dx%d",
                   curr_w, curr_h);
      return;
    }

    const double scale_w = static_cast<double>(curr_w) / static_cast<double>(calib_ref_w_);
    const double scale_h = static_cast<double>(curr_h) / static_cast<double>(calib_ref_h_);
    cv::Mat Kl = scaled_camera_matrix_wh(calib_K_left_, scale_w, scale_h);
    cv::Mat Kr = scaled_camera_matrix_wh(calib_K_right_, scale_w, scale_h);

    const cv::Size img_size(curr_w, curr_h);
    cv::Mat Dl = slice_distortion_for_undistort(calib_D_left_, undistort_dist_coeff_count_);
    cv::Mat Dr = slice_distortion_for_undistort(calib_D_right_, undistort_dist_coeff_count_);
    applyUndistortDistortionSafety(Dl, Dr, "scaled-K undistort");
    const cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);

    RCLCPP_INFO(get_logger(),
                "initUndistortRectifyMap: %dx%d CV_32FC1; D eff. left %zu right %zu "
                "(slice_count=%d, 0=full from YAML)",
                curr_w, curr_h, Dl.total(), Dr.total(), undistort_dist_coeff_count_);

    const int prev_ocv_threads = cv::getNumThreads();
    cv::setNumThreads(1);
    try {
      cv::initUndistortRectifyMap(Kl, Dl, R_eye, Kl, img_size, CV_32FC1, left_map1_, left_map2_);
    } catch (const cv::Exception &e) {
      cv::setNumThreads(prev_ocv_threads);
      RCLCPP_ERROR(get_logger(), "initUndistortRectifyMap LEFT failed: %s", e.what());
      left_map1_.release();
      left_map2_.release();
      return;
    }
    RCLCPP_INFO(get_logger(), "initUndistortRectifyMap: left maps map1 %dx%d ch=%d map2 %dx%d ch=%d",
                left_map1_.rows, left_map1_.cols, left_map1_.channels(),
                left_map2_.rows, left_map2_.cols, left_map2_.channels());
    try {
      cv::initUndistortRectifyMap(Kr, Dr, R_eye, Kr, img_size, CV_32FC1, right_map1_, right_map2_);
    } catch (const cv::Exception &e) {
      cv::setNumThreads(prev_ocv_threads);
      RCLCPP_ERROR(get_logger(), "initUndistortRectifyMap RIGHT failed: %s", e.what());
      left_map1_.release();
      left_map2_.release();
      right_map1_.release();
      right_map2_.release();
      return;
    }
    cv::setNumThreads(prev_ocv_threads);
    RCLCPP_INFO(get_logger(), "initUndistortRectifyMap: right maps map1 %dx%d ch=%d map2 %dx%d ch=%d",
                right_map1_.rows, right_map1_.cols, right_map1_.channels(),
                right_map2_.rows, right_map2_.cols, right_map2_.channels());

    undistort_maps_cache_w_ = curr_w;
    undistort_maps_cache_h_ = curr_h;

    camera_intrinsic_.fx = Kl.at<double>(0, 0);
    camera_intrinsic_.fy = Kl.at<double>(1, 1);
    camera_intrinsic_.cx = Kl.at<double>(0, 2);
    camera_intrinsic_.cy = Kl.at<double>(1, 2);
    camera_intrinsic_.baseline = stereo_baseline_m_;

    RCLCPP_INFO(get_logger(),
                "Undistort maps: image %dx%d (calib %dx%d) scale_w=%.4f scale_h=%.4f  "
                "K_left fx=%.2f cx=%.2f cy=%.2f",
                curr_w, curr_h, calib_ref_w_, calib_ref_h_, scale_w, scale_h,
                camera_intrinsic_.fx, camera_intrinsic_.cx, camera_intrinsic_.cy);
  }

  // ====================== processing thread =====================================

  void process_loop() {
    while (!shutdown_) {
      cv::Mat left_bgr, right_bgr;
      int64_t ts_ns = 0;

      {
        std::unique_lock<std::mutex> lk(sync_mtx_);
        sync_cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
          return shutdown_.load() || (!left_frames_.empty() &&
                                      !right_frames_.empty());
        });
        if (shutdown_) break;
        if (!try_match_locked(left_bgr, right_bgr, ts_ns)) {
          if (!left_frames_.empty() && !right_frames_.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Stereo sync: both sides decoding but no pair (left_buf=%zu right_buf=%zu "
                "tolerance_ns=%ld). Check timestamp skew or wait for keyframe on both.",
                left_frames_.size(), right_frames_.size(),
                static_cast<long>(sync_tolerance_ns_));
          }
          continue;
        }
      }

      // Check max_frames limit
      if (max_frames_ > 0 && frame_count_ >= max_frames_) {
        RCLCPP_INFO(get_logger(), "Reached max_frames (%d), stopping",
                    max_frames_);
        rclcpp::shutdown();
        return;
      }

      process_stereo_pair(left_bgr, right_bgr, ts_ns);
    }
  }

  // ====================== stereonet inference ===================================

  void process_stereo_pair(cv::Mat &left_bgr, cv::Mat &right_bgr,
                           int64_t ts_ns) {
    const int pre_fc = frame_count_.load();
    const bool trace = (pre_fc < 16);
    RCLCPP_INFO(get_logger(), "Processing frame #%d  ts=%ld",
                pre_fc, static_cast<long>(ts_ns));

    // ---- resize to model input if necessary --------------------------------
    if (model_input_w_ <= 0 || model_input_h_ <= 0) {
      RCLCPP_ERROR(get_logger(),
                   "Skip frame ts=%ld due to invalid model input size (%d x %d).",
                   ts_ns, model_input_w_, model_input_h_);
      return;
    }
    if (left_bgr.empty() || right_bgr.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Skip frame ts=%ld: left_bgr.empty=%d right_bgr.empty=%d",
                   ts_ns, left_bgr.empty() ? 1 : 0, right_bgr.empty() ? 1 : 0);
      return;
    }
    if (left_bgr.cols < 8 || left_bgr.rows < 8 || right_bgr.cols < 8 ||
        right_bgr.rows < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Skip frame: decoded image too small (left %dx%d right %dx%d)",
                           left_bgr.cols, left_bgr.rows, right_bgr.cols, right_bgr.rows);
      return;
    }
    if (!left_bgr.data || !right_bgr.data) {
      RCLCPP_ERROR(get_logger(),
                   "Skip frame ts=%ld: null data ptr (left=%p right=%p)",
                   ts_ns, static_cast<void *>(left_bgr.data),
                   static_cast<void *>(right_bgr.data));
      return;
    }
    if (trace) {
      RCLCPP_INFO(get_logger(),
                  "[trace pre_fc=%d] decoded BGR: left %dx%d type=%d c=%d | right %dx%d type=%d c=%d",
                  pre_fc, left_bgr.cols, left_bgr.rows, left_bgr.type(),
                  left_bgr.isContinuous() ? 1 : 0, right_bgr.cols, right_bgr.rows,
                  right_bgr.type(), right_bgr.isContinuous() ? 1 : 0);
    }
    int orig_w = left_bgr.cols, orig_h = left_bgr.rows;
    if (orig_w <= 0 || orig_h <= 0) {
      RCLCPP_ERROR(get_logger(),
                   "Skip frame ts=%ld due to invalid input image size (%d x %d).",
                   ts_ns, orig_w, orig_h);
      return;
    }

    cv::Mat left_rect, right_rect;
    if (use_rectification_) {
      if (trace) {
        RCLCPP_INFO(get_logger(),
                    "[trace pre_fc=%d] undistort: scaled_k=%s use_rectification_=true orig=%dx%d",
                    pre_fc, undistort_scaled_k_mode_ ? "yes" : "no", orig_w, orig_h);
      }
      if (undistort_scaled_k_mode_) {
        std::lock_guard<std::mutex> ulk(undistort_mtx_);
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] calling updateScaledUndistortMapsUnlocked", pre_fc);
        updateScaledUndistortMapsUnlocked(orig_w, orig_h);
        if (left_map1_.empty() || right_map1_.empty() ||
            left_map1_.size() != left_bgr.size() ||
            right_map1_.size() != right_bgr.size()) {
          RCLCPP_ERROR(get_logger(),
                       "remap/map mismatch: src %dx%d map1 %dx%d (right map %dx%d). Skip frame.",
                       left_bgr.cols, left_bgr.rows,
                       left_map1_.empty() ? -1 : left_map1_.cols,
                       left_map1_.empty() ? -1 : left_map1_.rows,
                       right_map1_.empty() ? -1 : right_map1_.cols,
                       right_map1_.empty() ? -1 : right_map1_.rows);
          return;
        }
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] cv::remap LEFT start", pre_fc);
        cv::remap(left_bgr, left_rect, left_map1_, left_map2_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] cv::remap LEFT done -> %dx%d", pre_fc,
                      left_rect.cols, left_rect.rows);
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] cv::remap RIGHT start", pre_fc);
        cv::remap(right_bgr, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] cv::remap RIGHT done -> %dx%d", pre_fc,
                      right_rect.cols, right_rect.rows);
      } else {
        if (left_map1_.empty() || left_map1_.size() != left_bgr.size()) {
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "stereoRectify maps size %dx%d != decoded %dx%d; skip remap (fix calib or resolution).",
              left_map1_.empty() ? 0 : left_map1_.cols,
              left_map1_.empty() ? 0 : left_map1_.rows, left_bgr.cols, left_bgr.rows);
          return;
        }
        if (trace)
          RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] stereoRectify path: remap L/R", pre_fc);
        cv::remap(left_bgr, left_rect, left_map1_, left_map2_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        cv::remap(right_bgr, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
      }
      if (trace) {
        RCLCPP_INFO(get_logger(),
                    "[trace pre_fc=%d] undistort/remap finished: left_rect %dx%d c=%d right_rect %dx%d c=%d",
                    pre_fc, left_rect.cols, left_rect.rows, left_rect.isContinuous() ? 1 : 0,
                    right_rect.cols, right_rect.rows, right_rect.isContinuous() ? 1 : 0);
      }
    } else {
      left_rect = left_bgr;
      right_rect = right_bgr;
      if (trace)
        RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] no rectification, pass-through BGR", pre_fc);
    }

    stereonet::CameraIntrinsic cam = camera_intrinsic_; // copy (scaled-K path: fx,cy match undistorted frame)
    cv::Mat left_img_resize, right_img_resize;

    if (orig_w != model_input_w_ || orig_h != model_input_h_) {
      if (trace)
        RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] cv::resize %dx%d -> %dx%d", pre_fc, orig_w,
                    orig_h, model_input_w_, model_input_h_);
      cv::resize(left_rect, left_img_resize,
                 cv::Size(model_input_w_, model_input_h_));
      cv::resize(right_rect, right_img_resize,
                 cv::Size(model_input_w_, model_input_h_));
      if (trace) {
        RCLCPP_INFO(get_logger(),
                    "[trace pre_fc=%d] resize done L %dx%d c=%d R %dx%d c=%d",
                    pre_fc, left_img_resize.cols, left_img_resize.rows,
                    left_img_resize.isContinuous() ? 1 : 0, right_img_resize.cols,
                    right_img_resize.rows, right_img_resize.isContinuous() ? 1 : 0);
      }
      if (cam.is_valid()) {
        cam.fx = cam.fx * model_input_w_ / orig_w;
        cam.fy = cam.fy * model_input_h_ / orig_h;
        cam.cx = cam.cx * model_input_w_ / orig_w;
        cam.cy = cam.cy * model_input_h_ / orig_h;
        if (trace || pre_fc == 0) {
          RCLCPP_INFO(get_logger(),
                      "Resized %dx%d -> %dx%d, scaled intrinsic [fx,fy,cx,cy]: %.2f, %.2f, %.2f, %.2f",
                      orig_w, orig_h, model_input_w_, model_input_h_, cam.fx, cam.fy, cam.cx, cam.cy);
        }
      }
    } else {
      left_img_resize = left_rect;
      right_img_resize = right_rect;
      if (trace)
        RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] no resize (already model size)", pre_fc);
    }

    // ---- BGR → NV12 --------------------------------------------------------
    const int64_t nv12_size64 =
        static_cast<int64_t>(model_input_w_) * static_cast<int64_t>(model_input_h_) * 3 / 2;
    // Safety: cap NV12 allocation to prevent "负数/溢出 -> 几十TB" failures.
    static constexpr int64_t kMaxNv12Bytes = 1024LL * 1024LL * 1024LL; // 1GB
    if (nv12_size64 <= 0 || nv12_size64 > kMaxNv12Bytes) {
      RCLCPP_ERROR(get_logger(),
                   "Skip frame ts=%ld due to unreasonable NV12 size (nv12_size64=%ld, w=%d, h=%d).",
                   ts_ns, static_cast<long>(nv12_size64), model_input_w_, model_input_h_);
      return;
    }
    // size_t cast is safe due to the max check above.
    size_t nv12_size = static_cast<size_t>(nv12_size64);
    std::vector<uint8_t> left_nv12(nv12_size), right_nv12(nv12_size);
    if (trace) {
      RCLCPP_INFO(get_logger(),
                  "[trace pre_fc=%d] bgr_mat_to_nv12: L %dx%d t=%d c=%d | R %dx%d t=%d c=%d",
                  pre_fc, left_img_resize.cols, left_img_resize.rows, left_img_resize.type(),
                  left_img_resize.isContinuous() ? 1 : 0, right_img_resize.cols,
                  right_img_resize.rows, right_img_resize.type(),
                  right_img_resize.isContinuous() ? 1 : 0);
    }
    if (!ImgConvertUtils::bgr_mat_to_nv12(left_img_resize, left_nv12.data())) {
      RCLCPP_ERROR(get_logger(),
                   "bgr_mat_to_nv12 FAILED (left) ts=%ld size=%dx%d type=%d continuous=%d",
                   ts_ns, left_img_resize.cols, left_img_resize.rows, left_img_resize.type(),
                   left_img_resize.isContinuous() ? 1 : 0);
      return;
    }
    if (!ImgConvertUtils::bgr_mat_to_nv12(right_img_resize, right_nv12.data())) {
      RCLCPP_ERROR(get_logger(),
                   "bgr_mat_to_nv12 FAILED (right) ts=%ld size=%dx%d type=%d continuous=%d",
                   ts_ns, right_img_resize.cols, right_img_resize.rows, right_img_resize.type(),
                   right_img_resize.isContinuous() ? 1 : 0);
      return;
    }
    if (trace || pre_fc == 0)
      RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] BGR->NV12 OK, nv12_size=%zu", pre_fc, nv12_size);

    // ---- BPU inference -----------------------------------------------------
    cv::Mat disp, uncert;
    if (trace)
      RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] forward_sync start", pre_fc);
    int ret = stereonet_process_->forward_sync(left_nv12, right_nv12, uncertainty_th_,
                                               disp, uncert);
    if (trace || pre_fc == 0)
      RCLCPP_INFO(get_logger(), "[trace pre_fc=%d] forward_sync returned %d disp empty=%d type=%d",
                  pre_fc, ret, disp.empty() ? 1 : 0, disp.type());
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "Inference failed ts=%ld forward_sync ret=%d", ts_ns, ret);
      return;
    }
    if (disp.empty() || disp.type() != CV_32FC1) {
      RCLCPP_ERROR(get_logger(),
                   "Invalid disparity ts=%ld: empty=%d type=%d (expect CV_32FC1=%d ch=%d)",
                   ts_ns, disp.empty() ? 1 : 0, disp.type(), CV_32FC1, disp.channels());
      return;
    }

    int count = frame_count_.fetch_add(1);

    if (trace || count <= 5)
      RCLCPP_INFO(get_logger(), "[trace count=%d] cloning Mats for postprocess thread", count);

    // Deep copies: inference thread may reuse Mats before async tasks run.
    cv::Mat left_rect_safe = left_rect.clone();
    cv::Mat right_rect_safe = right_rect.clone();
    cv::Mat left_rs_safe = left_img_resize.clone();
    cv::Mat right_rs_safe = right_img_resize.clone();
    cv::Mat disp_safe = disp.clone();
    cv::Mat uncert_safe = uncert.clone();

    if (trace || count <= 5)
      RCLCPP_INFO(get_logger(), "[trace count=%d] detach_task(postprocess) ...", count);

    // ---- Dispatch post-processing and saving to thread pool ----------------
    postprocess_thread_pool_ptr_->detach_task(
        [this, left_rect_safe, right_rect_safe, left_rs_safe, right_rs_safe, disp_safe,
         uncert_safe, cam, ts_ns, count]() {
          if (count < 12) {
            RCLCPP_INFO(get_logger(),
                        "[trace post count=%d] task begin cam_valid=%d disp %dx%d type=%d",
                        count, cam.is_valid() ? 1 : 0, disp_safe.cols, disp_safe.rows,
                        disp_safe.type());
          }
          // ---- depth (requires valid intrinsic) ----------------------------------
          cv::Mat depth;
          if (cam.is_valid()) {
            stereonet::StereonetProcess::disp_to_depth(disp_safe, depth, cam);
          } else if (count < 12) {
            RCLCPP_WARN(get_logger(),
                        "[trace post count=%d] skip disp_to_depth: cam not valid", count);
          }
          if (count < 12) {
            RCLCPP_INFO(get_logger(),
                        "[trace post count=%d] after disp_to_depth: depth empty=%d %dx%d",
                        count, depth.empty() ? 1 : 0, depth.cols, depth.rows);
          }

          // ---- build ROS header for publishers ------------------------------------
          std_msgs::msg::Header header;
          header.stamp.sec = static_cast<int32_t>(ts_ns / 1000000000LL);
          header.stamp.nanosec = static_cast<uint32_t>(ts_ns % 1000000000LL);
          header.frame_id = frame_id_;

          // ---- publish topics -----------------------------------------------------
          if (publish_depth_ && !depth.empty()) {
            publish_depth_image(depth, header);
          }
          if (publish_pointcloud_ && !depth.empty() && cam.is_valid()) {
            publish_pointcloud2(depth, left_rs_safe, cam, header);
          }
          if (publish_visual_ && !depth.empty() && cam.is_valid()) {
            cv::Mat visual;
            stereonet::StereonetProcess::convert_visual_img(left_rs_safe, disp_safe, depth,
                                                            cam, visual);
            publish_image(visual_pub_, visual, "bgr8", header);
          }
          if (publish_disp_ && !disp_safe.empty()) {
            publish_disp_image(disp_safe, header);
          }
          if (count < 12) {
            RCLCPP_INFO(get_logger(),
                        "[trace post count=%d] publish done (flags: depth=%d pc=%d visual=%d disp=%d)",
                        count, publish_depth_ ? 1 : 0, publish_pointcloud_ ? 1 : 0,
                        publish_visual_ ? 1 : 0, publish_disp_ ? 1 : 0);
          }

          // ---- save results ------------------------------------------------------
          if (save_results_ && (count % save_freq_ == 0)) {
            cv::Mat depth_copy = depth.clone();
            save_thread_pool_ptr_->detach_task(
                [this, left_rect_safe, right_rect_safe, left_rs_safe, right_rs_safe,
                 disp_safe, uncert_safe, depth_copy, cam, ts_ns, count]() {
                  save_frame_results(left_rect_safe, right_rect_safe, left_rs_safe,
                                     right_rs_safe, disp_safe, uncert_safe, depth_copy,
                                     cam, ts_ns, count);
                });
          }
        });
  }

  // ====================== result persistence ====================================

  void save_frame_results(const cv::Mat &left_bgr, const cv::Mat &right_bgr,
                          const cv::Mat &left_img_resize, const cv::Mat &right_img_resize,
                          const cv::Mat &disp, const cv::Mat &uncert,
                          const cv::Mat &depth,
                          const stereonet::CameraIntrinsic &cam,
                          int64_t ts_ns, int count) {
    std::string prefix = std::to_string(ts_ns);

    cv::imwrite(result_dir_ + "/left_" + prefix + ".png", left_bgr);
    cv::imwrite(result_dir_ + "/right_" + prefix + ".png", right_bgr);

    if (save_disp_ && !disp.empty())
      cv::imwrite(result_dir_ + "/disp_" + prefix + ".pfm", disp);

    if (save_uncert_ && !uncert.empty())
      cv::imwrite(result_dir_ + "/uncert_" + prefix + ".pfm", uncert);

    if (save_depth_ && !depth.empty())
      cv::imwrite(result_dir_ + "/depth_" + prefix + ".png", depth);

    if (save_visual_ && !depth.empty() && cam.is_valid()) {
      cv::Mat visual;
      stereonet::StereonetProcess::convert_visual_img(left_img_resize, disp, depth,
                                                      cam, visual);
      cv::imwrite(result_dir_ + "/visual_" + prefix + ".png", visual);
    }

    if (save_pcd_ && !depth.empty() && cam.is_valid()) {
      std::vector<stereonet::PointXYZRGB> pc;
      stereonet::StereonetProcess::depth_to_pointcloud_rgb(depth, left_img_resize,
                                                           cam, pc);
      stereonet::StereonetProcess::dump_pcd_file_rgb(
          result_dir_ + "/pointcloud_" + prefix + ".pcd", pc);
    }

    if (save_epipolar_) {
      auto cam_ptr = std::make_shared<stereonet::CameraIntrinsic>(cam);
      cv::Mat epi_vis;
      FeatureEpipolarAlign::check_epipolar_alignment(left_img_resize, right_img_resize,
                                                     cam_ptr, epi_vis);
      cv::imwrite(result_dir_ + "/epipolar_" + prefix + ".png", epi_vis);
    }

    if (cam.is_valid() && count == 0) {
      saveCameraIntrinsic(result_dir_, cam);
    }

    RCLCPP_INFO(get_logger(), "Saved results for ts=%ld to %s", ts_ns,
                result_dir_.c_str());
  }

  // ====================== topic publish functions ================================

  void publish_depth_image(const cv::Mat &depth,
                           const std_msgs::msg::Header &header) {
    // depth is CV_16UC1 (mm)
    cv::Mat cont = depth.isContinuous() ? depth : depth.clone();
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = cont.rows;
    msg->width = cont.cols;
    msg->encoding = "mono16";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(cont.step[0]);
    size_t data_size = msg->step * msg->height;
    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), cont.data, data_size);
    depth_pub_->publish(std::move(msg));
  }

  void publish_disp_image(const cv::Mat &disp,
                          const std_msgs::msg::Header &header) {
    // disp is CV_32FC1
    cv::Mat cont = disp.isContinuous() ? disp : disp.clone();
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = cont.rows;
    msg->width = cont.cols;
    msg->encoding = "32FC1";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(cont.step[0]);
    size_t data_size = msg->step * msg->height;
    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), cont.data, data_size);
    disp_pub_->publish(std::move(msg));
  }

  void publish_image(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub,
                     const cv::Mat &img, const std::string &encoding,
                     const std_msgs::msg::Header &header) {
    cv::Mat cont = img.isContinuous() ? img : img.clone();
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = cont.rows;
    msg->width = cont.cols;
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(cont.step[0]);
    size_t data_size = msg->step * msg->height;
    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), cont.data, data_size);
    pub->publish(std::move(msg));
  }

  void publish_pointcloud2(const cv::Mat &depth, const cv::Mat &rgb,
                           const stereonet::CameraIntrinsic &cam,
                           const std_msgs::msg::Header &header) {
    if (depth.type() != CV_16UC1 || rgb.type() != CV_8UC3 ||
        depth.size() != rgb.size()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "publish_pointcloud2: skip (depth %dx%d type=%d vs rgb %dx%d type=%d)",
          depth.cols, depth.rows, depth.type(), rgb.cols, rgb.rows, rgb.type());
      return;
    }
    int step = std::max(1, pc_downsample_step_);
    float max_depth_mm = static_cast<float>(pc_depth_max_ * 1000.0);

    // pre-count valid points
    int point_count = 0;
    for (int v = 0; v < depth.rows; v += step) {
      const uint16_t *row = depth.ptr<uint16_t>(v);
      for (int u = 0; u < depth.cols; u += step) {
        uint16_t d = row[u];
        if (d > 0 && d < static_cast<uint16_t>(max_depth_mm)) ++point_count;
      }
    }
    if (point_count == 0) return;

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header = header;
    msg->height = 1;
    msg->width = point_count;
    msg->is_dense = true;
    msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");

    float inv_fx = 1.0f / static_cast<float>(cam.fx);
    float inv_fy = 1.0f / static_cast<float>(cam.fy);
    float cx = static_cast<float>(cam.cx);
    float cy = static_cast<float>(cam.cy);

    for (int v = 0; v < depth.rows; v += step) {
      const uint16_t *d_row = depth.ptr<uint16_t>(v);
      const cv::Vec3b *c_row = rgb.ptr<cv::Vec3b>(v);
      for (int u = 0; u < depth.cols; u += step) {
        uint16_t d = d_row[u];
        if (d == 0 || d >= static_cast<uint16_t>(max_depth_mm)) continue;

        float z = d * 0.001f; // mm → m
        *iter_x = (u - cx) * z * inv_fx;
        *iter_y = (v - cy) * z * inv_fy;
        *iter_z = z;

        const cv::Vec3b &bgr = c_row[u];
        *iter_r = bgr[2];
        *iter_g = bgr[1];
        *iter_b = bgr[0];

        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
      }
    }

    pointcloud_pub_->publish(std::move(msg));
  }

  // ====================== member variables ======================================

  // -- params
  std::string model_path_;
  std::string post_version_;
  std::string left_topic_;
  std::string right_topic_;
  double uncertainty_th_ = -0.10;
  std::string result_dir_ = "./result";
  int64_t sync_tolerance_ns_ = 10000000; // 10 ms default
  int max_buffer_size_ = 10;
  int save_freq_ = 1;
  int max_frames_ = -1;
  bool save_results_ = true;
  bool save_visual_ = true;
  bool save_pcd_ = true;
  bool save_disp_ = true;
  bool save_depth_ = true;
  bool save_uncert_ = false;
  bool save_epipolar_ = false;
  stereonet::CameraIntrinsic camera_intrinsic_;

  // -- publish params
  bool publish_depth_ = true;
  bool publish_pointcloud_ = true;
  bool publish_visual_ = false;
  bool publish_disp_ = false;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string visual_topic_;
  std::string disp_topic_;
  std::string frame_id_ = "camera_link";
  int pc_downsample_step_ = 2;
  double pc_depth_max_ = 5.0;

  // -- model
  std::shared_ptr<stereonet::StereonetProcess> stereonet_process_;
  int model_input_w_ = 0;
  int model_input_h_ = 0;

  // -- decoders
  bool hevc_require_entry_point_ = true;
  std::unique_ptr<H265Decoder> left_decoder_;
  std::unique_ptr<H265Decoder> right_decoder_;

  // -- subscriptions
  rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr
      left_sub_;
  rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr
      right_sub_;

  // -- publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visual_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_pub_;

  // -- sync buffer & processing threads
  std::mutex sync_mtx_;
  std::condition_variable sync_cv_;
  std::map<int64_t, cv::Mat> left_frames_;
  std::map<int64_t, cv::Mat> right_frames_;
  std::vector<std::thread> process_threads_;
  int infer_thread_num_ = 1;
  std::atomic<bool> shutdown_{false};
  std::atomic<int> frame_count_{0};

  // -- thread pools for post-processing and saving
  std::unique_ptr<BS::thread_pool<>> postprocess_thread_pool_ptr_ = nullptr;
  std::unique_ptr<BS::thread_pool<>> save_thread_pool_ptr_ = nullptr;

  // -- rectification / undistort
  bool use_rectification_ = false;
  bool undistort_use_stereo_rectify_ = false;
  bool undistort_scaled_k_mode_ = false;
  int undistort_dist_coeff_count_ = 5;
  bool undistort_force_no_distortion_ = false;
  double undistort_extreme_k1_threshold_ = 2.0;
  int calib_ref_w_ = 0;
  int calib_ref_h_ = 0;
  int undistort_maps_cache_w_ = 0;
  int undistort_maps_cache_h_ = 0;
  double stereo_baseline_m_ = 0.0;
  cv::Mat calib_K_left_, calib_D_left_, calib_K_right_, calib_D_right_;
  std::mutex undistort_mtx_;
  cv::Mat left_map1_, left_map2_;
  cv::Mat right_map1_, right_map2_;
};

// ========================================================================================
// main
// ========================================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node =
      std::make_shared<StereoH265Node>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
