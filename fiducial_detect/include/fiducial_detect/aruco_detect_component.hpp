#pragma once

#include "fiducial_detect/fiducial_node_base.hpp"
#include "fiducial_core/aruco_detector.hpp"

namespace fiducial_detect {

class ArucoDetectComponent : public FiducialNodeBase<ArucoDetectComponent, fiducial_core::ArucoDetector> {
public:
    explicit ArucoDetectComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

} // namespace fiducial_detect
