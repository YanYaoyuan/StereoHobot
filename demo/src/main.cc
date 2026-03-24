#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "depth_anything_v2.hpp"

int main(int argc, char** argv) {
    // 简单参数：argv[1] 模型路径，argv[2] 测试图片路径
    std::string model_path = (argc > 1) ? argv[1] : "depth_any.hbm";
    std::string image_path = (argc > 2) ? argv[2] : "test.jpg";

    std::cout << "[DepthAnything] model: " << model_path
              << ", image: " << image_path << std::endl;

    // 1. 读取测试图片（BGR）
    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return -1;
    }

    try {
        // 2. 初始化模型
        DepthAnythingV2 model(model_path);

        // 3. 预处理
        model.pre_process(bgr);

        // 4. 推理
        model.infer();

        // 5. 后处理并保存结果
        cv::Mat depth_color;
        model.post_process(bgr.cols, bgr.rows, depth_color);

        const std::string out_path = "res_depth_cpp.png";
        cv::imwrite(out_path, depth_color);
        std::cout << "Success! Depth map saved to " << out_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

