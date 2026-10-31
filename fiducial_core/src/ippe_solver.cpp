#include "fiducial_core/ippe_solver.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>

namespace fiducial_core {

void IppeSolver::getSquareObjectPoints(float marker_length, std::vector<cv::Point3f>& obj_points) {
    obj_points.clear();
    obj_points.reserve(4);
    const float half_len = marker_length / 2.0f;
    // Standard OpenCV ArUco convention:
    // Corner 0: top-left (-half_len,  half_len, 0)
    // Corner 1: top-right ( half_len,  half_len, 0)
    // Corner 2: bottom-right ( half_len, -half_len, 0)
    // Corner 3: bottom-left (-half_len, -half_len, 0)
    obj_points.emplace_back(-half_len,  half_len, 0.0f);
    obj_points.emplace_back( half_len,  half_len, 0.0f);
    obj_points.emplace_back( half_len, -half_len, 0.0f);
    obj_points.emplace_back(-half_len, -half_len, 0.0f);
}

bool IppeSolver::solveSquare(
    const std::vector<cv::Point2f>& corners,
    double marker_length,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    FiducialObservation& obs)
{
    if (corners.size() != 4 || marker_length <= 0.0 || camera_matrix.empty()) {
        return false;
    }

    std::vector<cv::Point3f> obj_points;
    getSquareObjectPoints(static_cast<float>(marker_length), obj_points);

    // IPPE returns 2 candidate planar solutions, resolving ambiguity via reprojection residual
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<double> reprojection_errors;
    int num_solutions = cv::solvePnPGeneric(
        obj_points, corners, camera_matrix, dist_coeffs,
        rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
        cv::noArray(), cv::noArray(), reprojection_errors);

    if (num_solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        // Fallback to iterative solvePnP if IPPE encounters degenerate geometry
        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(obj_points, corners, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            return false;
        }
        obs.rvec = rvec;
        obs.tvec = tvec;
    } else {
        // Select solution with lowest reprojection error (guarantees non-flipped orientation)
        int best_idx = 0;
        if (num_solutions > 1 && reprojection_errors.size() >= 2) {
            if (reprojection_errors[1] < reprojection_errors[0]) {
                best_idx = 1;
            }
        }
        obs.rvec = rvecs[best_idx];
        obs.tvec = tvecs[best_idx];
    }

    // Compute exact RMS reprojection error
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_points, obs.rvec, obs.tvec, camera_matrix, dist_coeffs, projected);
    double sum_sq_err = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        double dx = corners[i].x - projected[i].x;
        double dy = corners[i].y - projected[i].y;
        sum_sq_err += (dx * dx + dy * dy);
    }
    obs.reprojection_error = std::sqrt(sum_sq_err / 4.0);

    // Compute diagonal corner distance in image pixels
    double diag_dx = corners[0].x - corners[2].x;
    double diag_dy = corners[0].y - corners[2].y;
    double diag_px = std::sqrt(diag_dx * diag_dx + diag_dy * diag_dy);
    if (diag_px < 1.0) diag_px = 1.0;

    double tvec_norm = cv::norm(obs.tvec);
    obs.object_error = (obs.reprojection_error / diag_px) * (tvec_norm / marker_length);
    obs.score = std::exp(-2.0 * obs.object_error);
    obs.marker_length = marker_length;
    obs.corners = corners;

    // Compute analytical 6x6 covariance
    obs.covariance = computeAnalyticalCovariance(
        obs.tvec, obs.rvec, marker_length,
        obs.reprojection_error, obs.object_error, camera_matrix);

    return true;
}

bool IppeSolver::solvePlanar(
    const std::vector<cv::Point2f>& img_points,
    const std::vector<cv::Point3f>& obj_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Vec3d& rvec,
    cv::Vec3d& tvec,
    double& reprojection_error)
{
    if (img_points.size() < 4 || obj_points.size() < 4 || camera_matrix.empty()) {
        return false;
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<double> errors;
    int num_solutions = cv::solvePnPGeneric(
        obj_points, img_points, camera_matrix, dist_coeffs,
        rvecs, tvecs, false, cv::SOLVEPNP_IPPE,
        cv::noArray(), cv::noArray(), errors);

    if (num_solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        if (!cv::solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            return false;
        }
    } else {
        int best_idx = 0;
        if (num_solutions > 1 && errors.size() >= 2 && errors[1] < errors[0]) {
            best_idx = 1;
        }
        rvec = rvecs[best_idx];
        tvec = tvecs[best_idx];
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
    double sum_sq_err = 0.0;
    for (size_t i = 0; i < img_points.size(); ++i) {
        double dx = img_points[i].x - projected[i].x;
        double dy = img_points[i].y - projected[i].y;
        sum_sq_err += (dx * dx + dy * dy);
    }
    reprojection_error = std::sqrt(sum_sq_err / static_cast<double>(img_points.size()));
    return true;
}

std::array<double, 36> IppeSolver::computeAnalyticalCovariance(
    const cv::Vec3d& tvec,
    const cv::Vec3d& rvec,
    double marker_length,
    double reprojection_error,
    double object_error,
    const cv::Mat& camera_matrix)
{
    (void)rvec;
    std::array<double, 36> cov{};
    cov.fill(0.0);

    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double f = (fx > 0.0 && fy > 0.0) ? (fx + fy) / 2.0 : 500.0;

    const double z = std::max(tvec[2], 0.05); // Min 5cm distance guard
    const double norm_t = std::max(cv::norm(tvec), 0.05);

    // Baseline subpixel corner detector noise standard deviation (pixels)
    const double sigma_pixel = std::max(reprojection_error, 0.35);

    // 1. Lateral variance (X and Y): sigma_xy = (Z / f) * sigma_pixel
    const double sigma_xy = (z / f) * sigma_pixel;
    const double var_x = sigma_xy * sigma_xy + 1e-6;
    const double var_y = sigma_xy * sigma_xy + 1e-6;

    // 2. Depth variance (Z): depth error scales quadratically with distance Z
    // dZ/dW = -Z^2 / (f * L) => sigma_z = (Z^2 / (f * L)) * (sqrt(2) * sigma_pixel)
    const double effective_len = std::max(marker_length, 0.02);
    const double dZ_dW = (z * z) / (f * effective_len);
    const double sigma_z = dZ_dW * (1.414 * sigma_pixel);
    const double var_z = sigma_z * sigma_z + (object_error * object_error * z * z) + 1e-6;

    // 3. Oblique angle inflation factor: as viewing angle approaches grazing, orientation uncertainty stretches
    double cos_tilt = z / norm_t;
    if (cos_tilt < 0.15) cos_tilt = 0.15;
    const double tilt_factor = 1.0 / (cos_tilt * cos_tilt);

    // 4. Orientation variance (Roll, Pitch, Yaw):
    // Angular error scales with (Z / (f * L)) * sigma_pixel
    const double sigma_rot_base = (z * sigma_pixel) / (f * effective_len);
    const double var_roll = std::max(sigma_rot_base * sigma_rot_base, 0.005 * 0.005);
    const double var_pitch = std::max(sigma_rot_base * sigma_rot_base * tilt_factor, 0.005 * 0.005);
    const double var_yaw = std::max(sigma_rot_base * sigma_rot_base * tilt_factor, 0.005 * 0.005);

    // Diagonal elements of 6x6 matrix: [x, y, z, roll, pitch, yaw]
    cov[0]  = var_x;     // (0, 0)
    cov[7]  = var_y;     // (1, 1)
    cov[14] = var_z;     // (2, 2)
    cov[21] = var_roll;  // (3, 3)
    cov[28] = var_pitch; // (4, 4)
    cov[35] = var_yaw;   // (5, 5)

    // Depth-lateral physical cross-correlations
    cov[2]  = (tvec[0] / z) * var_z * 0.08; // cov(x, z)
    cov[12] = cov[2];                       // cov(z, x)
    cov[8]  = (tvec[1] / z) * var_z * 0.08; // cov(y, z)
    cov[13] = cov[8];                       // cov(z, y)

    return cov;
}

} // namespace fiducial_core
