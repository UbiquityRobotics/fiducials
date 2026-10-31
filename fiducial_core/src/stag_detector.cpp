#include "fiducial_core/stag_detector.hpp"
#include "fiducial_core/ippe_solver.hpp"
#include "stag/Stag.h"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace fiducial_core {

StagDetector::StagDetector(const StagDetectorConfig& config)
{
    updateConfig(config);
}

StagDetector::~StagDetector() = default;

void StagDetector::updateConfig(const StagDetectorConfig& config)
{
    config_ = config;
    stag_ = std::make_unique<Stag>(config_.library_hd, config_.error_correction, false);
}

void StagDetector::setMarkerLengthOverride(int id, double length_meters)
{
    length_overrides_[id] = length_meters;
}

void StagDetector::clearMarkerLengthOverrides()
{
    length_overrides_.clear();
}

double StagDetector::getMarkerLength(int id) const
{
    auto it = length_overrides_.find(id);
    if (it != length_overrides_.end()) {
        return it->second;
    }
    return config_.default_marker_length;
}

size_t StagDetector::detect(
    const cv::Mat& image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<FiducialObservation>& observations)
{
    observations.clear();
    if (image.empty() || !stag_) {
        return 0;
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    stag_->detectMarkers(gray);
    const auto& markers = stag_->getMarkerList();
    if (markers.empty()) {
        return 0;
    }

    observations.reserve(markers.size());

    for (const auto& marker : markers) {
        FiducialObservation obs;
        obs.id = marker.id;
        double marker_len = getMarkerLength(obs.id);
        const double half_len = marker_len / 2.0;

        obs.corners.clear();
        obs.corners.reserve(4);
        for (size_t ci = 0; ci < 4 && ci < marker.corners.size(); ++ci) {
            obs.corners.emplace_back(
                static_cast<float>(marker.corners[ci].x),
                static_cast<float>(marker.corners[ci].y));
        }

        if (!camera_matrix.empty() && obs.corners.size() == 4) {
            // Setup 5 coplanar points (center + 4 corners) for planar IPPE
            std::vector<cv::Point2f> tag_image;
            tag_image.reserve(5);
            tag_image.emplace_back(
                static_cast<float>(marker.center.x),
                static_cast<float>(marker.center.y));
            for (const auto& pt : obs.corners) {
                tag_image.push_back(pt);
            }

            const float half_len_f = static_cast<float>(half_len);
            std::vector<cv::Point3f> tag_world;
            tag_world.reserve(5);
            tag_world.emplace_back(0.0f, 0.0f, 0.0f);
            tag_world.emplace_back(-half_len_f,  half_len_f, 0.0f);
            tag_world.emplace_back( half_len_f,  half_len_f, 0.0f);
            tag_world.emplace_back( half_len_f, -half_len_f, 0.0f);
            tag_world.emplace_back(-half_len_f, -half_len_f, 0.0f);

            if (IppeSolver::solvePlanar(tag_image, tag_world, camera_matrix, dist_coeffs,
                                       obs.rvec, obs.tvec, obs.reprojection_error))
            {
                double diag_dx = obs.corners[0].x - obs.corners[2].x;
                double diag_dy = obs.corners[0].y - obs.corners[2].y;
                double diag_px = std::sqrt(diag_dx * diag_dx + diag_dy * diag_dy);
                if (diag_px < 1.0) diag_px = 1.0;

                double tvec_norm = cv::norm(obs.tvec);
                obs.object_error = (obs.reprojection_error / diag_px) * (tvec_norm / marker_len);
                obs.score = std::exp(-2.0 * obs.object_error);
                obs.marker_length = marker_len;

                obs.covariance = IppeSolver::computeAnalyticalCovariance(
                    obs.tvec, obs.rvec, marker_len,
                    obs.reprojection_error, obs.object_error, camera_matrix);

                observations.push_back(obs);
            }
        } else {
            obs.marker_length = marker_len;
            obs.score = 1.0;
            observations.push_back(obs);
        }
    }

    return observations.size();
}

} // namespace fiducial_core
