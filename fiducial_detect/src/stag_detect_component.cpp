#include "fiducial_detect/stag_detect_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace fiducial_detect {

StagDetectComponent::StagDetectComponent(const rclcpp::NodeOptions& options)
    : FiducialNodeBase<StagDetectComponent, fiducial_core::StagDetector>("stag_detector", options)
{
    this->declare_parameter<int>("stag_library", 15);
    this->declare_parameter<int>("error_correction", 7);

    fiducial_core::StagDetectorConfig config;
    config.library_hd = this->get_parameter("stag_library").as_int();
    config.error_correction = this->get_parameter("error_correction").as_int();
    config.default_marker_length = this->get_parameter("fiducial_len").as_double();

    detector_.updateConfig(config);
}

} // namespace fiducial_detect

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_detect::StagDetectComponent)
