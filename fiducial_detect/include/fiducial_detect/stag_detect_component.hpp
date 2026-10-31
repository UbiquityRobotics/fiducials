#pragma once

#include "fiducial_detect/fiducial_node_base.hpp"
#include "fiducial_core/stag_detector.hpp"

namespace fiducial_detect {

class StagDetectComponent : public FiducialNodeBase<StagDetectComponent, fiducial_core::StagDetector> {
public:
    explicit StagDetectComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

} // namespace fiducial_detect
