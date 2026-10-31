#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include "fiducial_core/observation.hpp"

namespace fiducial_core {

/**
 * @brief Closed-form Infinitesimal Plane-Based Pose Estimation (IPPE) solver.
 *
 * Replaces iterative Levenberg-Marquardt solvePnP with analytical closed-form
 * planar estimation, eliminating 180-degree flip ambiguities and calculating
 * analytical 6x6 spatial covariance.
 */
class IppeSolver {
public:
    /**
     * @brief Generates standard 3D planar object points for a square marker centered at origin.
     * @param marker_length Physical side length in meters.
     * @param obj_points Output vector of 4 3D points.
     */
    static void getSquareObjectPoints(float marker_length, std::vector<cv::Point3f>& obj_points);

    static bool solveSquare(
        const std::vector<cv::Point2f>& corners,
        double marker_length,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        FiducialObservation& obs);

    static bool solvePlanar(
        const std::vector<cv::Point2f>& img_points,
        const std::vector<cv::Point3f>& obj_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        cv::Vec3d& rvec,
        cv::Vec3d& tvec,
        double& reprojection_error);

    /**
     * @brief Computes analytical 6x6 spatial covariance for a planar marker observation.
     */
    static std::array<double, 36> computeAnalyticalCovariance(
        const cv::Vec3d& tvec,
        const cv::Vec3d& rvec,
        double marker_length,
        double reprojection_error,
        double object_error,
        const cv::Mat& camera_matrix);
};

} // namespace fiducial_core
