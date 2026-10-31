#include "fiducial_detect/aruco_detect_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace fiducial_detect {

ArucoDetectComponent::ArucoDetectComponent(const rclcpp::NodeOptions& options)
    : FiducialNodeBase<ArucoDetectComponent, fiducial_core::ArucoDetector>("aruco_detector", options)
{
    this->declare_parameter<int>("dictionary", cv::aruco::DICT_6X6_250);
    this->declare_parameter<int>("adaptiveThreshWinSizeMin", 7);
    this->declare_parameter<int>("adaptiveThreshWinSizeMax", 23);
    this->declare_parameter<int>("adaptiveThreshWinSizeStep", 8);
    this->declare_parameter<double>("adaptiveThreshConstant", 7.0);
    this->declare_parameter<double>("minMarkerPerimeterRate", 0.08);
    this->declare_parameter<bool>("doCornerRefinement", true);

    fiducial_core::ArucoDetectorConfig config;
    config.dictionary_id = this->get_parameter("dictionary").as_int();
    config.default_marker_length = this->get_parameter("fiducial_len").as_double();
    config.adaptiveThreshWinSizeMin = this->get_parameter("adaptiveThreshWinSizeMin").as_int();
    config.adaptiveThreshWinSizeMax = this->get_parameter("adaptiveThreshWinSizeMax").as_int();
    config.adaptiveThreshWinSizeStep = this->get_parameter("adaptiveThreshWinSizeStep").as_int();
    config.adaptiveThreshConstant = this->get_parameter("adaptiveThreshConstant").as_double();
    config.minMarkerPerimeterRate = this->get_parameter("minMarkerPerimeterRate").as_double();
    config.doCornerRefinement = this->get_parameter("doCornerRefinement").as_bool();

    detector_.updateConfig(config);
}

} // namespace fiducial_detect

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_detect::ArucoDetectComponent)
