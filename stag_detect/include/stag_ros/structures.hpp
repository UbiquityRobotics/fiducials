#pragma once
#include <opencv2/core/types.hpp>

namespace stag_ros {

struct Tag {
  int id;
  std::string frame_id;
  cv::Point3d corners[4];
  cv::Point3d center;
};

struct Bundle {
  std::string frame_id;
  std::vector<Tag> tags;
};

}  // namespace stag_ros