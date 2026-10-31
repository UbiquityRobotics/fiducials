#pragma once

#include <vector>
#include <array>
#include <opencv2/core.hpp>

namespace fiducial_core {

/**
 * @brief Represents a single detected fiducial observation with pose and uncertainty.
 */
struct FiducialObservation {
    int id{0};
    std::vector<cv::Point2f> corners;        // 4 corners in image space (TL, TR, BR, BL)
    cv::Vec3d rvec{0.0, 0.0, 0.0};           // Rodrigues rotation vector relative to camera frame
    cv::Vec3d tvec{0.0, 0.0, 0.0};           // Translation vector [X, Y, Z] in meters relative to camera frame
    std::array<double, 36> covariance{};     // Analytical 6x6 pose covariance matrix (row-major)
    double reprojection_error{0.0};          // RMS reprojection error in pixels
    double score{1.0};                       // Confidence score [0.0 .. 1.0]
    double object_error{0.0};                // Normalized object error
    double marker_length{0.0};               // Physical side length in meters
};

} // namespace fiducial_core
