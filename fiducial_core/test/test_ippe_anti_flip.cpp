#include <gtest/gtest.h>
#include "fiducial_core/ippe_solver.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

TEST(FiducialCoreTest, IppeAntiFlipObliqueAngle) {
    const double fx = 800.0, fy = 800.0;
    const double cx = 320.0, cy = 240.0;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    const double marker_len = 0.14;
    // 65 degree tilt around Y axis for marker facing camera
    const double tilt_angle = 65.0 * M_PI / 180.0;
    cv::Mat R_base = (cv::Mat_<double>(3, 3) <<
        1.0,  0.0,  0.0,
        0.0, -1.0,  0.0,
        0.0,  0.0, -1.0);
    cv::Mat R_tilt = (cv::Mat_<double>(3, 3) <<
        std::cos(tilt_angle), 0.0, std::sin(tilt_angle),
        0.0, 1.0, 0.0,
        -std::sin(tilt_angle), 0.0, std::cos(tilt_angle));
    cv::Mat R_gt = R_base * R_tilt;
    cv::Vec3d gt_rvec;
    cv::Rodrigues(R_gt, gt_rvec);
    const cv::Vec3d gt_tvec(0.0, 0.0, 1.20);

    std::vector<cv::Point3f> obj_points;
    fiducial_core::IppeSolver::getSquareObjectPoints(static_cast<float>(marker_len), obj_points);

    std::vector<cv::Point2f> img_corners;
    cv::projectPoints(obj_points, gt_rvec, gt_tvec, camera_matrix, dist_coeffs, img_corners);

    fiducial_core::FiducialObservation obs;
    bool success = fiducial_core::IppeSolver::solveSquare(
        img_corners, marker_len, camera_matrix, dist_coeffs, obs);

    ASSERT_TRUE(success);

    // 1. Verify rotation error does not suffer a 180-degree flip
    double rot_err = cv::norm(obs.rvec - gt_rvec);
    EXPECT_LT(rot_err, 0.05) << "IPPE rotation error was " << rot_err << " rad (no flip)";

    // 2. Verify normal vector points towards the camera
    cv::Mat R;
    cv::Rodrigues(obs.rvec, R);
    cv::Vec3d normal(R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));

    // Marker Z normal in camera frame should have a negative component along the line of sight (facing camera)
    EXPECT_LT(normal[2], 0.0) << "Marker normal must point towards camera, not flipped backwards";

    // 3. Translation error must be sub-centimeter
    double trans_err = cv::norm(obs.tvec - gt_tvec);
    EXPECT_LT(trans_err, 0.005);
}
