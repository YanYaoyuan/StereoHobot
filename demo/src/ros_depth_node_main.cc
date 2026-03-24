#include "rclcpp/rclcpp.hpp"
#include "depth_node.hpp"

/**
 * @brief DepthAnything ROS2 节点入口
 *
 * 功能：
 *  - 启动 DepthInferenceNode
 *  - 通过 ROS2 Spin 处理图像 Topic 与深度图发布
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthInferenceNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

