#include "stag/EDInterface.h"
#include "stag/utility.h"

#include <algorithm>
#include <vector>

using std::vector;
using std::max;
using std::min;
using cv::Point2i;
using cv::Point2d;

void EDInterface::runEDPFandEDLines(const cv::Mat& image) {
  if (edLines != NULL) delete edLines;
  if (edgeMap != NULL) delete edgeMap;

  edLines = DetectLinesByEDPF(edgeMap, image.data, image.size().width,
                              image.size().height, false, 0);
}

EdgeMap* EDInterface::getEdgeMap() { return edgeMap; }

EDLines* EDInterface::getEDLines() { return edLines; }

void EDInterface::correctLineDirection(const cv::Mat& image, LineSegment& ls) {
  // if invert == 0, the line is longer in x-axis
  // if invert == 1, the line is longer in y-axis
  // the number of pixels along the longer axis is the number of pixels to be
  // sampled (sampling may be done sparsely if it's a bottleneck)
  int pointsToSample;
  int minX, maxX, minY, maxY;
  if (ls.invert == 0) {
    minX = (int)min(ls.sx, ls.ex);
    maxX = (int)(max(ls.sx, ls.ex) + 0.5);
    pointsToSample = maxX - minX + 1;
  } else {
    minY = (int)min(ls.sy, ls.ey);
    maxY = (int)(max(ls.sy, ls.ey) + 0.5);
    pointsToSample = maxY - minY + 1;
  }

  // line's left and right sides are sampled with an offset
  double offset = 1;

  // create vectors to hold the pixels to be sampled
  // right holds the right-hand side pixels when going from start to end, vice
  // versa

  // the sampling is done using nearest neighborhood, hence the int type
  // these are the pixels to be sampled for each side
  vector<Point2i> right(pointsToSample);
  vector<Point2i> left(pointsToSample);

  // these are the pixels on the line
  double neutX, neutY;

  if (ls.invert == 0)  // if the line is horizontal
  {
    if (ls.sx < ls.ex)  // if the line goes left to right
    {
      for (int i = 0; i < pointsToSample; i++) {
        neutX = minX + i;
        neutY = ls.b * neutX + ls.a;

        right[i] = Point2i((int)neutX, (int)round(neutY + offset));
        left[i] = Point2i((int)neutX, (int)round(neutY - offset));
      }
    } else  // if the line goes right to left
    {
      for (int i = 0; i < pointsToSample; i++) {
        neutX = minX + i;
        neutY = ls.b * neutX + ls.a;

        right[i] = Point2i((int)neutX, (int)round(neutY - offset));
        left[i] = Point2i((int)neutX, (int)round(neutY + offset));
      }
    }
  }

  else  // if the line is vertical
  {
    if (ls.sy < ls.ey)  // if the line goes up to down
    {
      for (int i = 0; i < pointsToSample; i++) {
        neutY = minY + i;
        neutX = ls.b * neutY + ls.a;

        right[i] = Point2i((int)round(neutX - offset), (int)neutY);
        left[i] = Point2i((int)round(neutX + offset), (int)neutY);
      }
    } else  // if the line goes down to up
    {
      for (int i = 0; i < pointsToSample; i++) {
        neutY = minY + i;
        neutX = ls.b * neutY + ls.a;

        right[i] = Point2i((int)round(neutX + offset), (int)neutY);
        left[i] = Point2i((int)round(neutX - offset), (int)neutY);
      }
    }
  }

  // read image and accumulate brightness levels from the pixels

  // first check if a pixel outside the borders is meant to be read
  // if so, use safe read
  minX = min(min(right[0].x, right.back().x), min(left[0].x, left.back().x));
  maxX = max(max(right[0].x, right.back().x), max(left[0].x, left.back().x));
  minY = min(min(right[0].y, right.back().y), min(left[0].y, left.back().y));
  maxY = max(max(right[0].y, right.back().y), max(left[0].y, left.back().y));

  bool useSafeRead = false;
  if ((minX < 0) || (maxX >= image.size().width) || (minY < 0) ||
      (maxY >= image.size().height))
    useSafeRead = true;

  // now read each pixel and accumulate brightness levels
  unsigned int accRight = 0, accLeft = 0;

  if (useSafeRead) {
    for (int i = 0; i < pointsToSample; i++) {
      accRight += readPixelSafe(image, right[i]);
      accLeft += readPixelSafe(image, left[i]);
    }
  } else {
    for (int i = 0; i < pointsToSample; i++) {
      accRight += readPixelUnsafe(image, right[i]);
      accLeft += readPixelUnsafe(image, left[i]);
    }
  }

  // when going from start to end, right-hand side should have been darker =>
  // lesser accumulation if this is not the case, replace sx-sy with ex-ey
  if (accLeft < accRight) {
    double t1 = ls.sx;
    double t2 = ls.sy;
    ls.sx = ls.ex;
    ls.sy = ls.ey;
    ls.ex = t1;
    ls.ey = t2;
  }
}

Point2d EDInterface::intersectionOfLineSegments(const LineSegment& line1,
                                                const LineSegment& line2) {
  Point2d inters;

  double aL1, bL1, aL2, bL2;
  if (line1.invert == 0) {
    aL1 = line1.b;
    bL1 = line1.a;
  } else {
    aL1 = 1 / line1.b;
    bL1 = -line1.a / line1.b;
  }
  if (line2.invert == 0) {
    aL2 = line2.b;
    bL2 = line2.a;
  } else {
    aL2 = 1 / line2.b;
    bL2 = -line2.a / line2.b;
  }
  inters.x = (bL2 - bL1) / (aL1 - aL2);
  inters.y = aL1 * inters.x + bL1;

  if ((line1.invert == 1) && (line1.b == 0)) {
    if (line2.invert == 0) {
      inters.y = line2.a + line2.b * line1.a;
      inters.x = line1.a;
    } else {
      inters.y = (line1.a - line2.a) / line2.b;
      inters.x = line1.a;
    }
  } else if ((line2.invert == 1) && (line2.b == 0)) {
    if (line1.invert == 0) {
      inters.y = line1.a + line1.b * line2.a;
      inters.x = line2.a;
    } else {
      inters.y = (line2.a - line1.a) / line1.b;
      inters.x = line2.a;
    }
  }
  return inters;
}