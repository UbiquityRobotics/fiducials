#include <gtest/gtest.h>
#include "fiducial_core/aruco_detector.hpp"
#include "fiducial_core/ippe_solver.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

TEST(FiducialCoreTest, PinholeGroundTruthAccuracy) {
    // 1. Setup camera intrinsics
    const double fx = 800.0, fy = 800.0;
    const double cx = 320.0, cy = 240.0;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    // 2. Setup ground-truth 3D pose of marker facing the camera (Rx(pi) aligns marker Y-up with camera Y-down)
    const double marker_len = 0.14; // 14 cm
    const cv::Vec3d gt_tvec(0.15, -0.10, 1.50); // X=15cm, Y=-10cm, Z=1.5m
    const cv::Vec3d gt_rvec(M_PI + 0.05, -0.08, 0.20); // Marker facing camera with small perturbation

    // 3. Project 3D marker corners into camera 2D image plane
    std::vector<cv::Point3f> obj_points;
    fiducial_core::IppeSolver::getSquareObjectPoints(static_cast<float>(marker_len), obj_points);

    std::vector<cv::Point2f> img_corners;
    cv::projectPoints(obj_points, gt_rvec, gt_tvec, camera_matrix, dist_coeffs, img_corners);

    // 4. Create synthetic 640x480 image and warp the marker pattern onto the projected corners
    cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC1);
    canvas.setTo(200); // Gray background

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat marker_src;
    cv::aruco::drawMarker(dict, 42, 200, marker_src, 1);

    std::vector<cv::Point2f> src_corners = {
        cv::Point2f(0.f, 0.f),
        cv::Point2f(199.f, 0.f),
        cv::Point2f(199.f, 199.f),
        cv::Point2f(0.f, 199.f)
    };

    cv::Mat H = cv::getPerspectiveTransform(src_corners, img_corners);
    cv::Mat warped;
    cv::warpPerspective(marker_src, warped, H, canvas.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(200));

    // Combine warped marker onto canvas
    cv::Mat mask;
    cv::threshold(warped, mask, 254, 255, cv::THRESH_BINARY_INV);
    warped.copyTo(canvas, mask);

    // 5. Run detector
    fiducial_core::ArucoDetectorConfig config;
    config.dictionary_id = cv::aruco::DICT_6X6_250;
    config.default_marker_length = marker_len;
    fiducial_core::ArucoDetector detector(config);

    std::vector<fiducial_core::FiducialObservation> observations;
    size_t count = detector.detect(canvas, camera_matrix, dist_coeffs, observations);

    ASSERT_EQ(count, 1u) << "Expected exactly 1 detected marker";
    const auto& obs = observations[0];
    EXPECT_EQ(obs.id, 42);

    // 6. Verify translation error <= 1 cm at 1.5m range
    double trans_err = cv::norm(obs.tvec - gt_tvec);
    EXPECT_LT(trans_err, 0.010) << "Translation error was " << trans_err << " m (must be < 1 cm)";

    // 7. Verify SO(3) geodesic rotation error <= 0.03 rad (~1.7 deg)
    cv::Mat R_gt, R_obs;
    cv::Rodrigues(gt_rvec, R_gt);
    cv::Rodrigues(obs.rvec, R_obs);
    cv::Mat R_diff = R_gt.t() * R_obs;
    cv::Vec3d r_diff;
    cv::Rodrigues(R_diff, r_diff);
    double rot_err = cv::norm(r_diff);
    EXPECT_LT(rot_err, 0.03) << "SO(3) geodesic rotation error was " << rot_err << " rad (must be < 0.03 rad)";

    // 8. Verify covariance properties
    EXPECT_GT(obs.covariance[0], 0.0);  // var(x) > 0
    EXPECT_GT(obs.covariance[7], 0.0);  // var(y) > 0
    EXPECT_GT(obs.covariance[14], 0.0); // var(z) > 0
    // Depth variance should exceed lateral variance
    EXPECT_GT(obs.covariance[14], obs.covariance[0]);
    EXPECT_GT(obs.score, 0.5);
}
