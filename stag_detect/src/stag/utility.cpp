#include "stag/utility.h"

using cv::Mat;
using cv::Point2i;
using cv::Point2d;

unsigned char readPixelUnsafe(const cv::Mat& image, const Point2i& p) {
  return image.ptr<unsigned char>(p.y)[p.x];
}

unsigned char readPixelSafe(const cv::Mat& image, const Point2i& p) {
  if ((p.x >= 0) && (p.x < image.size().width) && (p.y >= 0) &&
      (p.y < image.size().height))
    return image.ptr<unsigned char>(p.y)[p.x];
  else
    return (unsigned char)128;  // if the point to be read is outside image
                                // boundaries, return 128
}

unsigned char readPixelSafeBilinear(const cv::Mat& image, const Point2d& p) {
  if ((p.x >= 0) && (p.x <= image.size().width - 1) && (p.y >= 0) &&
      (p.y <= image.size().height - 1)) {
    int x1 = floor(p.x);
    int x2 = ceil(p.x);
    int y1 = floor(p.y);
    int y2 = ceil(p.y);

    double dist1 = sqrt((x1 - p.x) * (x1 - p.x) + (y1 - p.y) * (y1 - p.y));
    double dist2 = sqrt((x1 - p.x) * (x1 - p.x) + (y2 - p.y) * (y2 - p.y));
    double dist3 = sqrt((x2 - p.x) * (x2 - p.x) + (y1 - p.y) * (y1 - p.y));
    double dist4 = sqrt((x2 - p.x) * (x2 - p.x) + (y2 - p.y) * (y2 - p.y));
    double totDist = dist1 + dist2 + dist3 + dist4;

    double totalRead = 0;
    unsigned char reading;

    reading = image.ptr<unsigned char>(y1)[x1];
    ;
    totalRead += reading * dist1;

    reading = image.ptr<unsigned char>(y2)[x1];
    ;
    totalRead += reading * dist2;

    reading = image.ptr<unsigned char>(y1)[x2];
    totalRead += reading * dist3;

    reading = image.ptr<unsigned char>(y2)[x2];
    totalRead += reading * dist4;

    return totalRead / totDist;
  } else
    return (unsigned char)128;  // if the point to be read is outside image
                                // boundaries, return 128
}

double crossProduct(const Point2d& p1, const Point2d& p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

double squaredDistance(const Point2d& p1, const Point2d& p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}