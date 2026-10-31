#include <rclcpp/rclcpp.hpp>
#include "fiducial_detect/aruco_detect_component.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<fiducial_detect::ArucoDetectComponent>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
