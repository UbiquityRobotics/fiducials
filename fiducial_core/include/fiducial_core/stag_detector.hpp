#pragma once

#include <vector>
#include <map>
#include <memory>
#include <opencv2/core.hpp>
#include "fiducial_core/observation.hpp"

class Stag;

namespace fiducial_core {

struct StagDetectorConfig {
    int library_hd{15};
    int error_correction{7};
    double default_marker_length{0.14}; // meters
};

class StagDetector {
public:
    explicit StagDetector(const StagDetectorConfig& config = StagDetectorConfig());
    ~StagDetector();

    void updateConfig(const StagDetectorConfig& config);
    void setMarkerLengthOverride(int id, double length_meters);
    void clearMarkerLengthOverrides();
    double getMarkerLength(int id) const;

    /**
     * @brief Detects STag markers and computes 3D poses using IPPE.
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
    StagDetectorConfig config_;
    std::unique_ptr<Stag> stag_;
    std::map<int, double> length_overrides_;
};

} // namespace fiducial_core
