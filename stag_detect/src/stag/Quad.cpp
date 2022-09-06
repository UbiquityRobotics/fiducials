#include "stag/Quad.h"
#include "stag/utility.h"

using cv::Point2d;
using cv::Point3d;
using cv::Mat;

Quad::Quad(vector<Point2d> inCorners) {
  corners = inCorners;
  calculateLineAtInfinity();
  calculateProjectiveDistortion();
}

void Quad::estimateHomography() {
  // apply affine rectification to the corners
  vector<Point2d> affineCorners = vector<Point2d>(4);

  for (int i = 0; i < 4; i++)
    affineCorners[i] = Point2d(
        corners[i].x /
            (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z),
        corners[i].y /
            (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z));

  Mat HarInv = Mat::eye(3, 3, CV_64FC1);
  Mat Haffsim = Mat::eye(3, 3, CV_64FC1);

  // inverse of affine rectification
  HarInv.at<double>(2, 0) = -lineInf.x / lineInf.z;
  HarInv.at<double>(2, 1) = -lineInf.y / lineInf.z;
  HarInv.at<double>(2, 2) = 1 / lineInf.z;

  // find the affine transformation from square to affine rectified quad
  Haffsim.at<double>(0, 0) = affineCorners[1].x - affineCorners[0].x;
  Haffsim.at<double>(0, 1) = affineCorners[3].x - affineCorners[0].x;
  Haffsim.at<double>(0, 2) = affineCorners[0].x;
  Haffsim.at<double>(1, 0) = affineCorners[1].y - affineCorners[0].y;
  Haffsim.at<double>(1, 1) = affineCorners[3].y - affineCorners[0].y;
  Haffsim.at<double>(1, 2) = affineCorners[0].y;

  // product of these transformations is the homography
  H = HarInv * Haffsim;

  // locate the projection of the center of the marker
  Mat origCenter(3, 1, CV_64FC1);
  origCenter.at<double>(0) = 0.5;
  origCenter.at<double>(1) = 0.5;
  origCenter.at<double>(2) = 1;

  origCenter = H * origCenter;
  center.x = origCenter.at<double>(0) / origCenter.at<double>(2);
  center.y = origCenter.at<double>(1) / origCenter.at<double>(2);
}

void Quad::calculateLineAtInfinity() {
  // intersection points at the vanishing line
  Point2d inters1, inters2;

  // cross products of corners (i.e. lines representing the edges)
  double cross14 = crossProduct(corners[0], corners[3]);
  double cross23 = crossProduct(corners[1], corners[2]);
  double cross12 = crossProduct(corners[0], corners[1]);
  double cross34 = crossProduct(corners[2], corners[3]);

  // vectors going from one corner to another
  Point2d vec23(corners[1].x - corners[2].x, corners[1].y - corners[2].y);
  Point2d vec14(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
  Point2d vec34(corners[2].x - corners[3].x, corners[2].y - corners[3].y);
  Point2d vec12(corners[0].x - corners[1].x, corners[0].y - corners[1].y);

  // if both edge pairs are parallel
  if ((crossProduct(vec14, vec23) == 0) &&
      (crossProduct(vec12, vec34) == 0))  // lines are parallel
  {
    lineInf = Point3d(0, 0, 1);
    return;
  }
  // if one edge pair is parallel
  else if (crossProduct(vec14, vec23) == 0) {
    inters2.x = (cross12 * vec34.x - vec12.x * cross34) /
                (vec12.x * vec34.y - vec12.y * vec34.x);
    inters2.y = (cross12 * vec34.y - vec12.y * cross34) /
                (vec12.x * vec34.y - vec12.y * vec34.x);

    // this intersection is not real. doing this to find the equation of the
    // line with only one point.
    inters1.x = inters2.x + vec14.x;
    inters1.y = inters2.y + vec14.y;
  }
  // if the other edge pair is parallel
  else if (crossProduct(vec12, vec34) == 0) {
    inters1.x = (cross14 * vec23.x - vec14.x * cross23) /
                (vec14.x * vec23.y - vec14.y * vec23.x);
    inters1.y = (cross14 * vec23.y - vec14.y * cross23) /
                (vec14.x * vec23.y - vec14.y * vec23.x);

    // this intersection is not real. doing this to find the equation of the
    // line with only one point.
    inters2.x = inters1.x + vec12.x;
    inters2.y = inters1.y + vec12.y;
  }
  // if neither pairs are parallel
  else {
    inters1.x = (cross14 * vec23.x - vec14.x * cross23) /
                (vec14.x * vec23.y - vec14.y * vec23.x);
    inters1.y = (cross14 * vec23.y - vec14.y * cross23) /
                (vec14.x * vec23.y - vec14.y * vec23.x);

    inters2.x = (cross12 * vec34.x - vec12.x * cross34) /
                (vec12.x * vec34.y - vec12.y * vec34.x);
    inters2.y = (cross12 * vec34.y - vec12.y * cross34) /
                (vec12.x * vec34.y - vec12.y * vec34.x);
  }

  // find the vanishing line in homogeneous coordinates
  // l = P1 x P2
  double l1 = inters1.y - inters2.y;
  double l2 = inters2.x - inters1.x;
  double l3 = inters1.x * inters2.y - inters2.x * inters1.y;

  // normalize using
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
  // (13)
  double normalizer = sqrt(l1 * l1 + l2 * l2);
  l1 /= normalizer;
  l2 /= normalizer;
  l3 /= normalizer;

  lineInf = Point3d(l1, l2, l3);
}

void Quad::calculateProjectiveDistortion() {
  // find the minimum and maximum distance from corners to the vanishing line
  // projective distortion = maxDist / minDist
  double curDist =
      abs(lineInf.x * corners[0].x + lineInf.y * corners[0].y + lineInf.z);

  double minDist = curDist;
  double maxDist = curDist;

  for (int i = 1; i < 4; i++) {
    curDist =
        abs(lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z);
    if (curDist < minDist) minDist = curDist;
    if (curDist > maxDist) maxDist = curDist;
  }
  projectiveDistortion = maxDist / minDist;
}

Quad::Quad(const Quad &q) {
  corners = q.corners;
  lineInf = q.lineInf;
  projectiveDistortion = q.projectiveDistortion;
  H = q.H.clone();
  center = q.center;
}