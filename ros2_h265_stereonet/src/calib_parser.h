#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

struct CameraCalib {
    int width = 0;
    int height = 0;
    cv::Mat K; // 3x3
    cv::Mat D; // 1xN
    cv::Mat R_BC; // 3x3
    cv::Vec3d T_BC; // 3x1
};

struct LidarCalib {
    cv::Mat R_BL;   // 3x3  body→lidar rotation
    cv::Vec3d t_BL; // 3x1  body→lidar translation
};

inline cv::Mat quatToMat(double qx, double qy, double qz, double qw) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    double sqw = qw*qw;
    double sqx = qx*qx;
    double sqy = qy*qy;
    double sqz = qz*qz;

    double invs = 1.0 / (sqx + sqy + sqz + sqw);
    double m00 = ( sqx - sqy - sqz + sqw)*invs;
    double m11 = (-sqx + sqy - sqz + sqw)*invs;
    double m22 = (-sqx - sqy + sqz + sqw)*invs;

    double tmp1 = qx*qy;
    double tmp2 = qz*qw;
    double m10 = 2.0 * (tmp1 + tmp2)*invs;
    double m01 = 2.0 * (tmp1 - tmp2)*invs;

    double tmp3 = qx*qz;
    double tmp4 = qy*qw;
    double m20 = 2.0 * (tmp3 - tmp4)*invs;
    double m02 = 2.0 * (tmp3 + tmp4)*invs;

    double tmp5 = qy*qz;
    double tmp6 = qx*qw;
    double m21 = 2.0 * (tmp5 + tmp6)*invs;
    double m12 = 2.0 * (tmp5 - tmp6)*invs;

    R.at<double>(0,0) = m00; R.at<double>(0,1) = m01; R.at<double>(0,2) = m02;
    R.at<double>(1,0) = m10; R.at<double>(1,1) = m11; R.at<double>(1,2) = m12;
    R.at<double>(2,0) = m20; R.at<double>(2,1) = m21; R.at<double>(2,2) = m22;
    return R;
}

inline bool parseFullStereoCalibYaml(const std::string &file_path, CameraCalib &left, CameraCalib &right) {
    std::ifstream infile(file_path);
    if (!infile.is_open()) return false;

    std::string current_label;
    std::string current_section;

    auto parseArray = [](const std::string &s) -> std::vector<double> {
        auto p0 = s.find('[');
        auto p1 = s.find(']');
        if (p0 == std::string::npos || p1 == std::string::npos) return {};
        std::string inner = s.substr(p0 + 1, p1 - p0 - 1);
        for (char &c : inner) { if (c == ',') c = ' '; }
        std::istringstream ss(inner);
        std::vector<double> vals;
        double v;
        while (ss >> v) { vals.push_back(v); }
        return vals;
    };

    auto trim = [](const std::string &s) -> std::string {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    };

    std::string line;
    while (std::getline(infile, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;

        if (t.find("label:") != std::string::npos) {
            current_label = trim(t.substr(t.find(':') + 1));
            current_section.clear();
            std::cout << "[CalibParser] Found label: " << current_label << std::endl;
            continue;
        }

        CameraCalib* cam = nullptr;
        if (current_label == "stereo_left") cam = &left;
        else if (current_label == "stereo_right") cam = &right;
        
        if (!cam) continue;

        if (t.find("image_width:") != std::string::npos) {
            cam->width = std::stoi(trim(t.substr(t.find(':') + 1)));
            std::cout << "[CalibParser] " << current_label << " width: " << cam->width << std::endl;
            continue;
        }
        if (t.find("image_height:") != std::string::npos) {
            cam->height = std::stoi(trim(t.substr(t.find(':') + 1)));
            std::cout << "[CalibParser] " << current_label << " height: " << cam->height << std::endl;
            continue;
        }

        if (t.find("intrinsics:") != std::string::npos && t.find("data:") == std::string::npos) { current_section = "intrinsics"; continue; }
        if (t.find("distortion:") != std::string::npos) { current_section = "distortion"; continue; }
        if (t.find("Extr_B_C:") != std::string::npos)   { current_section = "extr"; continue; }

        if (current_section == "intrinsics" && t.find("data:") != std::string::npos) {
            auto vals = parseArray(t);
            if (vals.size() == 4) {
                cam->K = cv::Mat::eye(3, 3, CV_64F);
                cam->K.at<double>(0,0) = vals[0]; // fx
                cam->K.at<double>(1,1) = vals[1]; // fy
                cam->K.at<double>(0,2) = vals[2]; // cx
                cam->K.at<double>(1,2) = vals[3]; // cy
                std::cout << "[CalibParser] " << current_label << " K: fx=" << vals[0] << " fy=" << vals[1] << " cx=" << vals[2] << " cy=" << vals[3] << std::endl;
            }
            current_section.clear();
            continue;
        }

        if (current_section == "distortion" && t.find("data:") != std::string::npos) {
            auto vals = parseArray(t);
            cam->D = cv::Mat(1, vals.size(), CV_64F);
            std::cout << "[CalibParser] " << current_label << " D: [";
            for (size_t i = 0; i < vals.size(); ++i) {
                cam->D.at<double>(0, i) = vals[i];
                std::cout << vals[i] << (i == vals.size()-1 ? "" : ", ");
            }
            std::cout << "]" << std::endl;
            current_section.clear();
            continue;
        }

        if (current_section == "extr" && t.find("orientation:") != std::string::npos) {
            auto vals = parseArray(t);
            if (vals.size() == 4) {
                // D-Robotics YAML format is [w, x, y, z]
                // Our quatToMat expects (x, y, z, w)
                cam->R_BC = quatToMat(vals[1], vals[2], vals[3], vals[0]);
                std::cout << "[CalibParser] " << current_label << " R_BC parsed from quat." << std::endl;
            }
            continue;
        }
        if (current_section == "extr" && t.find("translation:") != std::string::npos) {
            auto vals = parseArray(t);
            if (vals.size() == 3) {
                cam->T_BC = cv::Vec3d(vals[0], vals[1], vals[2]);
                std::cout << "[CalibParser] " << current_label << " T_BC: [" << vals[0] << ", " << vals[1] << ", " << vals[2] << "]" << std::endl;
            }
            continue;
        }
    }

    return !left.K.empty() && !right.K.empty();
}

// Parse Extr_B_L from the "lidar" label section of a vita_calib.yaml
inline bool parseLidarCalibYaml(const std::string &file_path, LidarCalib &lidar) {
    std::ifstream infile(file_path);
    if (!infile.is_open()) return false;

    auto parseArray = [](const std::string &s) -> std::vector<double> {
        auto p0 = s.find('[');
        auto p1 = s.find(']');
        if (p0 == std::string::npos || p1 == std::string::npos) return {};
        std::string inner = s.substr(p0 + 1, p1 - p0 - 1);
        for (char &c : inner) { if (c == ',') c = ' '; }
        std::istringstream ss(inner);
        std::vector<double> vals;
        double v;
        while (ss >> v) vals.push_back(v);
        return vals;
    };

    auto trim = [](const std::string &s) -> std::string {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    };

    bool in_lidar = false;
    bool in_extr  = false;
    std::string line;
    while (std::getline(infile, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;

        if (t.find("label:") != std::string::npos) {
            std::string lbl = trim(t.substr(t.find(':') + 1));
            in_lidar = (lbl == "lidar");
            in_extr  = false;
            continue;
        }
        if (!in_lidar) continue;

        if (t.find("Extr_B_L:") != std::string::npos) { in_extr = true; continue; }
        if (!in_extr) continue;

        if (t.find("orientation:") != std::string::npos) {
            auto vals = parseArray(t);
            if (vals.size() == 4) {
                // vita_calib format: [w, x, y, z]
                lidar.R_BL = quatToMat(vals[1], vals[2], vals[3], vals[0]);
                std::cout << "[CalibParser] lidar R_BL parsed from quat." << std::endl;
            }
            continue;
        }
        if (t.find("translation:") != std::string::npos) {
            auto vals = parseArray(t);
            if (vals.size() == 3) {
                lidar.t_BL = cv::Vec3d(vals[0], vals[1], vals[2]);
                std::cout << "[CalibParser] lidar t_BL: ["
                          << vals[0] << ", " << vals[1] << ", " << vals[2] << "]" << std::endl;
            }
            // once we have both orientation+translation we can stop
            if (!lidar.R_BL.empty()) break;
            continue;
        }
    }

    return !lidar.R_BL.empty();
}
