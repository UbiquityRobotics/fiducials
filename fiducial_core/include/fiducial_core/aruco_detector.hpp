#pragma once

#include <vector>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include "fiducial_core/observation.hpp"

namespace fiducial_core {

struct ArucoDetectorConfig {
    int dictionary_id{cv::aruco::DICT_6X6_250};
    double default_marker_length{0.14}; // meters
    // Guarded adaptive window sweep parameters
    int adaptiveThreshWinSizeMin{7};
    int adaptiveThreshWinSizeMax{23};
    int adaptiveThreshWinSizeStep{8};
    double adaptiveThreshConstant{7.0};
    double minMarkerPerimeterRate{0.08};
    double maxMarkerPerimeterRate{4.0};
    double polygonalApproxAccuracyRate{0.05};
    double minCornerDistanceRate{0.05};
    int minDistanceToBorder{3};
    bool doCornerRefinement{true};
    int cornerRefinementWinSize{5};
    int cornerRefinementMaxIterations{30};
    double cornerRefinementMinAccuracy{0.1};
};

class ArucoDetector {
public:
    explicit ArucoDetector(const ArucoDetectorConfig& config = ArucoDetectorConfig());
    ~ArucoDetector() = default;

    void updateConfig(const ArucoDetectorConfig& config);
    void setMarkerLengthOverride(int id, double length_meters);
    void clearMarkerLengthOverrides();
    double getMarkerLength(int id) const;

    /**
     * @brief Detects ArUco markers in an image and computes accurate 3D poses using IPPE.
     * @param image Input image (single-channel CV_8UC1 or BGR CV_8UC3).
     * @param camera_matrix 3x3 camera intrinsic matrix.
     * @param dist_coeffs Distortion coefficients.
     * @param observations Output vector of detected observations.
     * @return Number of detected markers.
     */
    size_t detect(
        const cv::Mat& image,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        std::vector<FiducialObservation>& observations);

private:
    ArucoDetectorConfig config_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::map<int, double> length_overrides_;

    // Pre-allocated internal buffers to eliminate heap churn
    std::vector<int> cached_ids_;
    std::vector<std::vector<cv::Point2f>> cached_corners_;
    std::vector<std::vector<cv::Point2f>> cached_rejected_;
};

} // namespace fiducial_core
