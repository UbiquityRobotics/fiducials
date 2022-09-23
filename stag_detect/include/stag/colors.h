#ifndef COLORS_H
#define COLORS_H

#include <vector>

#include <opencv2/opencv.hpp>

using std::vector;

static vector<cv::Scalar> colors = {
    cv::Scalar(81, 112, 215),  cv::Scalar(255, 0, 13),
    cv::Scalar(154, 14, 234),  cv::Scalar(21, 176, 26),
    cv::Scalar(255, 129, 192),

    cv::Scalar(4, 217, 255),   cv::Scalar(249, 115, 6),
    cv::Scalar(237, 13, 217),  cv::Scalar(19, 234, 201),
    cv::Scalar(255, 255, 20),

    cv::Scalar(1, 101, 252),   cv::Scalar(143, 20, 2),
    cv::Scalar(112, 59, 231),  cv::Scalar(33, 252, 13),
    cv::Scalar(255, 186, 205),

    cv::Scalar(130, 202, 252), cv::Scalar(255, 176, 124),
    cv::Scalar(194, 0, 120),   cv::Scalar(107, 139, 164),
    cv::Scalar(219, 180, 120),

    cv::Scalar(4, 133, 209),   cv::Scalar(190, 1, 25),
    cv::Scalar(133, 103, 152), cv::Scalar(2, 147, 134),
    cv::Scalar(255, 2, 141),

    cv::Scalar(162, 191, 254), cv::Scalar(252, 90, 80),
    cv::Scalar(188, 19, 254),  cv::Scalar(205, 253, 2),
    cv::Scalar(205, 197, 10),

    cv::Scalar(30, 72, 143),   cv::Scalar(173, 129, 80),
    cv::Scalar(108, 52, 97),   cv::Scalar(10, 136, 138),
    cv::Scalar(192, 115, 122),

    cv::Scalar(152, 239, 249), cv::Scalar(127, 94, 0),
    cv::Scalar(191, 119, 246), cv::Scalar(176, 221, 22),
    cv::Scalar(252, 192, 6),

    cv::Scalar(3, 113, 156),   cv::Scalar(169, 86, 30),
    cv::Scalar(110, 117, 14),  cv::Scalar(4, 92, 90),
    cv::Scalar(146, 149, 145),

    cv::Scalar(6, 154, 243),   cv::Scalar(255, 167, 86),
    cv::Scalar(135, 174, 115), cv::Scalar(150, 249, 123),
    cv::Scalar(255, 255, 194)};

#endif