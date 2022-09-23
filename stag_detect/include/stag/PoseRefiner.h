#ifndef POSEREFINER_H
#define POSEREFINER_H

#include "stag/EDInterface.h"
#include "stag/Marker.h"
#include "stag/EDInterface.h"

class PoseRefiner {
  bool checkIfPointInQuad(const Marker& marker, const cv::Point2d& p);
  cv::Point2d projectPoint(cv::Point2d p, cv::Mat H);

 public:
  void refineMarkerPose(EDInterface* edInterface, Marker& marker);
};

#endif