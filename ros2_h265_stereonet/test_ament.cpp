#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
int main() {
    std::cout << ament_index_cpp::get_package_share_directory("ros2_h265_stereonet") << std::endl;
    return 0;
}
