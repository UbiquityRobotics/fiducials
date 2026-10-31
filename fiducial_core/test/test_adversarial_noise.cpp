#include <gtest/gtest.h>
#include "fiducial_core/aruco_detector.hpp"
#include <opencv2/core.hpp>

TEST(FiducialCoreTest, AdversarialNoiseRejection) {
    const double fx = 800.0, fy = 800.0;
    const double cx = 320.0, cy = 240.0;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    fiducial_core::ArucoDetectorConfig config;
    config.dictionary_id = cv::aruco::DICT_6X6_250;
    fiducial_core::ArucoDetector detector(config);

    // Test 1: Pure Gaussian sensor noise (mean=128, std=40)
    cv::RNG rng(12345);
    for (int frame = 0; frame < 20; ++frame) {
        cv::Mat noise(480, 640, CV_8UC1);
        rng.fill(noise, cv::RNG::NORMAL, 128, 40);

        std::vector<fiducial_core::FiducialObservation> obs;
        size_t count = detector.detect(noise, camera_matrix, dist_coeffs, obs);
        EXPECT_EQ(count, 0u) << "False positive detected on Gaussian noise frame " << frame;
    }

    // Test 2: Uniform random noise (high entropy)
    for (int frame = 0; frame < 20; ++frame) {
        cv::Mat rand_img(480, 640, CV_8UC1);
        rng.fill(rand_img, cv::RNG::UNIFORM, 0, 256);

        std::vector<fiducial_core::FiducialObservation> obs;
        size_t count = detector.detect(rand_img, camera_matrix, dist_coeffs, obs);
        EXPECT_EQ(count, 0u) << "False positive detected on uniform noise frame " << frame;
    }

    // Test 3: Blank solid frames
    cv::Mat black = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat white = cv::Mat::ones(480, 640, CV_8UC1) * 255;
    std::vector<fiducial_core::FiducialObservation> obs;
    EXPECT_EQ(detector.detect(black, camera_matrix, dist_coeffs, obs), 0u);
    EXPECT_EQ(detector.detect(white, camera_matrix, dist_coeffs, obs), 0u);
}
