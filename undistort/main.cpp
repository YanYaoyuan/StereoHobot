#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 1. 原始参数 (1080P)
    double orig_w = 1920.0;
    double orig_h = 1080.0;
    
    // 2. 当前图像参数
    double curr_w = 640.0;
    double curr_h = 325.0;

    // 关键：统一使用宽度的缩放比例，确保物体肥瘦不变
    double scale = curr_w / orig_w; 

    // 缩放内参
    double fx = 805.299072 * scale;
    double fy = 805.879883 * scale;
    double cx = 959.196655 * scale;
    
    // 修正 cy：考虑到 325 可能是从 1080P 等比缩放后的 360 像素中裁剪出来的
    // 假设是中心裁剪，cy 需要减去顶部被裁掉的高度
    double equivalent_h = orig_h * scale; // 360 像素
    double top_cut = (equivalent_h - curr_h) / 2.0; 
    double cy = (538.812378 * scale) - top_cut;

    cv::Mat K = (cv::Mat_<double>(3, 3) << 
                 fx, 0,  cx,
                 0,  fy, cy,
                 0,  0,  1);

    // 3. 8个畸变参数 (保持不变)
    cv::Mat D = (cv::Mat_<double>(1, 8) << 
                 7.99858522, 3.08304286, -8.16457978e-05, 0.000179127994, 
                 0.0763648525, 8.38899517, 5.9210844, 0.559656382);

    cv::Mat img = cv::imread("input.jpg");
    if (img.empty()) return -1;

    // 4. 解决“变窄”的关键：
    // 不要让 getOptimalNewCameraMatrix 自由缩放 fx/fy
    // 我们直接使用缩放后的 K 作为新内参，或者将 alpha 设为 0 并手动微调
    cv::Mat output;
    
    // 如果使用 alpha=0 依然觉得瘦，说明 new_K 改变了横纵比
    // 这里我们尝试直接用原来的 K（缩放后）来去畸变，观察比例
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), K, img.size(), CV_16SC2, map1, map2);
    cv::remap(img, output, map1, map2, cv::INTER_LINEAR);

    // 5. 保存结果
    cv::imwrite("rectified_fixed_aspect.jpg", output);
    
    std::cout << "校正完成。如果边缘仍有黑边，请尝试在 getOptimalNewCameraMatrix 中使用 alpha=0。" << std::endl;

    return 0;
}