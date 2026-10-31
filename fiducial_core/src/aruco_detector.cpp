#include "fiducial_core/aruco_detector.hpp"
#include "fiducial_core/ippe_solver.hpp"
#include <opencv2/imgproc.hpp>

namespace fiducial_core {

ArucoDetector::ArucoDetector(const ArucoDetectorConfig& config)
{
    updateConfig(config);
}

void ArucoDetector::updateConfig(const ArucoDetectorConfig& config)
{
    config_ = config;
    dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(config_.dictionary_id));

    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->adaptiveThreshWinSizeMin = config_.adaptiveThreshWinSizeMin;
    detector_params_->adaptiveThreshWinSizeMax = config_.adaptiveThreshWinSizeMax;
    detector_params_->adaptiveThreshWinSizeStep = config_.adaptiveThreshWinSizeStep;
    detector_params_->adaptiveThreshConstant = config_.adaptiveThreshConstant;
    detector_params_->minMarkerPerimeterRate = config_.minMarkerPerimeterRate;
    detector_params_->maxMarkerPerimeterRate = config_.maxMarkerPerimeterRate;
    detector_params_->polygonalApproxAccuracyRate = config_.polygonalApproxAccuracyRate;
    detector_params_->minCornerDistanceRate = config_.minCornerDistanceRate;
    detector_params_->minDistanceToBorder = config_.minDistanceToBorder;

    if (config_.doCornerRefinement) {
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = config_.cornerRefinementWinSize;
        detector_params_->cornerRefinementMaxIterations = config_.cornerRefinementMaxIterations;
        detector_params_->cornerRefinementMinAccuracy = config_.cornerRefinementMinAccuracy;
    } else {
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
}

void ArucoDetector::setMarkerLengthOverride(int id, double length_meters)
{
    length_overrides_[id] = length_meters;
}

void ArucoDetector::clearMarkerLengthOverrides()
{
    length_overrides_.clear();
}

double ArucoDetector::getMarkerLength(int id) const
{
    auto it = length_overrides_.find(id);
    if (it != length_overrides_.end()) {
        return it->second;
    }
    return config_.default_marker_length;
}

size_t ArucoDetector::detect(
    const cv::Mat& image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<FiducialObservation>& observations)
{
    observations.clear();
    cached_ids_.clear();
    cached_corners_.clear();
    cached_rejected_.clear();

    if (image.empty()) {
        return 0;
    }

    // Ensure grayscale mono8 image
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Run ArUco marker corner detection
    cv::aruco::detectMarkers(
        gray, dictionary_, cached_corners_, cached_ids_,
        detector_params_, cached_rejected_);

    if (cached_ids_.empty()) {
        return 0;
    }

    observations.reserve(cached_ids_.size());

    for (size_t i = 0; i < cached_ids_.size(); ++i) {
        FiducialObservation obs;
        obs.id = cached_ids_[i];
        double marker_len = getMarkerLength(obs.id);

        if (!camera_matrix.empty()) {
            if (IppeSolver::solveSquare(cached_corners_[i], marker_len, camera_matrix, dist_coeffs, obs)) {
                observations.push_back(obs);
            }
        } else {
            // Intrinsics not yet available: return corner geometry only
            obs.corners = cached_corners_[i];
            obs.marker_length = marker_len;
            obs.score = 1.0;
            observations.push_back(obs);
        }
    }

    return observations.size();
}

} // namespace fiducial_core
