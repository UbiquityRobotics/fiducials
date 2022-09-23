#pragma once

namespace stag_ros {

// Read image from msg and convert it to grayscale, checks provided for rgb8
// and bgr8, default to mono8
inline bool msgToGray(const sensor_msgs::ImageConstPtr &msg, cv::Mat &gray) {
  if (msg->encoding.compare("bgr8") == 0) {
    cv::Mat src = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    return true;
  } else if (msg->encoding.compare("rgb8") == 0) {
    cv::Mat src = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::cvtColor(src, gray, CV_RGB2GRAY);
    return true;
  } else if (msg->encoding.compare("mono8") == 0) {
    gray = cv_bridge::toCvShare(msg, msg->encoding)->image;
    return true;
  }
  return false;
}

}  // namespace stag_ros