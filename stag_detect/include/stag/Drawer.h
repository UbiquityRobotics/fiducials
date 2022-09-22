#ifndef DRAWER_H
#define DRAWER_H

#include <string>
#include <opencv2/opencv.hpp>

#include "stag/ED/EDLines.h"
#include "stag/ED/EdgeMap.h"
#include "stag/QuadDetector.h"
#include "stag/Marker.h"

using std::string;

class Drawer {
  void colorAPixel(cv::Mat& img, int x, int y, cv::Scalar color, int dotWidth);

 public:
  // draws edge segments
  void drawEdgeMap(const string& path, cv::Mat image, EdgeMap* edgeMap);

  // draws line segments
  void drawLines(const string& path, cv::Mat image, EDLines* edLines);

  // draws corners (intersections of line segments)
  void drawCorners(const string& path, cv::Mat image,
                   const vector<vector<Corner>>& cornerGroups);

  // draws quads
  void drawQuads(const string& path, cv::Mat image, const vector<Quad>& quads);

  // draws markers
  cv::Mat drawMarkers(const string& path, cv::Mat image,
                      const vector<Marker>& markers);

  // draws refined markers and their ellipses
  cv::Mat drawEllipses(const string& path, cv::Mat image,
                       const vector<Marker>& markers);
};

#endif