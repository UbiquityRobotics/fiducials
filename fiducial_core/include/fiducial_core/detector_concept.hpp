#pragma once

#include <concepts>
#include <vector>
#include <opencv2/core.hpp>
#include "fiducial_core/observation.hpp"

namespace fiducial_core {

/**
 * @brief C++20 Concept defining a Fiducial Detector implementation.
 */
template <typename T>
concept DetectorConcept = requires(
    T detector,
    const cv::Mat& img,
    const cv::Mat& cam_mat,
    const cv::Mat& dist,
    std::vector<FiducialObservation>& obs,
    int id,
    double length)
{
    { detector.detect(img, cam_mat, dist, obs) } -> std::same_as<size_t>;
    { detector.setMarkerLengthOverride(id, length) } -> std::same_as<void>;
    { detector.getMarkerLength(id) } -> std::convertible_to<double>;
};

} // namespace fiducial_core
