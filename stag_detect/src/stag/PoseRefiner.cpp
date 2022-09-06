#include <numeric>
#include "stag/PoseRefiner.h"
#include "stag/Ellipse.h"
#include "stag/utility.h"

using cv::Point2d;
using cv::Mat;

Mat optEllipse;
class Refine;

void PoseRefiner::refineMarkerPose(EDInterface* edInterface, Marker& marker) {
  // we find an edge segment loop that belongs to the circular border, and fit
  // an ellipse to it the edge segment loop needs pass closely to the circular
  // border when backprojected to test this, we accumulate the min distances
  // from the backprojected edge segment to 36 uniformly sampled points on the
  // circular border

  // these are the points to which the distances are to be sampled. left as is
  // for documentation purposes
  static const double sinVals[36] = {
      0.000000,  0.173648,  0.342020,  0.500000,  0.642788,  0.766044,
      0.866025,  0.939693,  0.984808,  1.000000,  0.984808,  0.939693,
      0.866025,  0.766044,  0.642788,  0.500000,  0.342020,  0.173648,
      0.000000,  -0.173648, -0.342020, -0.500000, -0.642788, -0.766044,
      -0.866025, -0.939693, -0.984808, -1.000000, -0.984808, -0.939693,
      -0.866025, -0.766044, -0.642788, -0.500000, -0.342020, -0.173648};
  vector<Point2d> samplePoints(36);
  for (int i = 0; i < 36; i++)
    samplePoints[i] =
        Point2d(0.5 + 0.4 * sinVals[(i + 9) % 36], 0.5 + 0.4 * sinVals[i]);

  // find the edge segment loop that is most likely to belong to the ellipse
  EdgeMap* edgeMap = edInterface->getEdgeMap();
  int indChosenSegment = -1;
  double minAccError = INFINITY;

  // the edge segment we are looking for is at least minEdgeSegmentLength long
  int minEdgeSegmentLength = 20;
  // it is a loop, the distance between the start and end points is below
  // loopThres
  double loopThres = 7;
  // we sample the distances from the circular border on the marker plane to the
  // backprojected edge segment this distances will be referred to as error the
  // edge segment we are looking for is at most errorThres distant from the
  // circular border
  double errorThres = 0.1;
  // the accumulated distances from the 36 points to the edge segment is at most
  // accErrorThres
  double accErrorThres = 36 * 0.05;

  for (int indEdgeSegment = 0; indEdgeSegment < edgeMap->noSegments;
       indEdgeSegment++) {
    // skip if the edge segment is too short
    if (edgeMap->segments[indEdgeSegment].noPixels < minEdgeSegmentLength)
      continue;

    // skip if the edge segment is not a loop
    if (squaredDistance(
            Point2d(edgeMap->segments[indEdgeSegment].pixels[0].c,
                    edgeMap->segments[indEdgeSegment].pixels[0].r),
            Point2d(edgeMap->segments[indEdgeSegment]
                        .pixels[edgeMap->segments[indEdgeSegment].noPixels - 1]
                        .c,
                    edgeMap->segments[indEdgeSegment]
                        .pixels[edgeMap->segments[indEdgeSegment].noPixels - 1]
                        .r)) > loopThres * loopThres)
      continue;

    // sample the edge segment, skip if any parts are outside the marker
    bool skipBecauseOutside = false;
    for (int indEdgePix = 0;
         indEdgePix < edgeMap->segments[indEdgeSegment].noPixels;
         indEdgePix += minEdgeSegmentLength) {
      if (!checkIfPointInQuad(
              marker,
              Point2d(
                  edgeMap->segments[indEdgeSegment].pixels[indEdgePix].c,
                  edgeMap->segments[indEdgeSegment].pixels[indEdgePix].r))) {
        skipBecauseOutside = true;
        break;
      }
    }
    if (skipBecauseOutside) continue;

    // project this edge segment to marker plane
    Mat Hinv = marker.H.inv();
    vector<Point2d> projPixels(edgeMap->segments[indEdgeSegment].noPixels);
    vector<double> sampleErrors(
        36, INFINITY);  // min distance from the 36 points to the edge segment
    vector<double> pixelErrors(
        edgeMap->segments[indEdgeSegment].noPixels,
        INFINITY);  // min distance from edge segment individual edge segment
                    // pixels to the 36 points
    for (int indEdgePix = 0;
         indEdgePix < edgeMap->segments[indEdgeSegment].noPixels;
         indEdgePix++) {
      Mat edgePix =
          (cv::Mat_<double>(3, 1)
               << edgeMap->segments[indEdgeSegment].pixels[indEdgePix].c,
           edgeMap->segments[indEdgeSegment].pixels[indEdgePix].r, 1);
      Mat projEdgePix = Hinv * edgePix;

      projPixels[indEdgePix] =
          Point2d(projEdgePix.at<double>(0) / projEdgePix.at<double>(2),
                  projEdgePix.at<double>(1) / projEdgePix.at<double>(2));
      for (int sampleInd = 0; sampleInd < 36; sampleInd++) {
        double dist =
            sqrt((projPixels[indEdgePix].x - samplePoints[sampleInd].x) *
                     (projPixels[indEdgePix].x - samplePoints[sampleInd].x) +
                 (projPixels[indEdgePix].y - samplePoints[sampleInd].y) *
                     (projPixels[indEdgePix].y - samplePoints[sampleInd].y));

        if (dist < sampleErrors[sampleInd]) sampleErrors[sampleInd] = dist;

        if (dist < pixelErrors[indEdgePix]) pixelErrors[indEdgePix] = dist;
      }
    }

    // skip if at least one projected pixel is far away from the expected circle
    bool atLeastOneBadPixel = false;
    for (int j = 0; j < pixelErrors.size(); j++) {
      if (pixelErrors[j] > errorThres) {
        atLeastOneBadPixel = true;
        break;
      }
    }
    if (atLeastOneBadPixel) continue;

    // update the chosen segment index if this segment has the lowest error
    double errorSum =
        std::accumulate(sampleErrors.begin(), sampleErrors.end(), (double)0);
    if ((errorSum < minAccError) && (errorSum < accErrorThres)) {
      minAccError = errorSum;
      indChosenSegment = indEdgeSegment;
    }
  }

  // do not refine the pose if none of the edge segments were satisfactory
  if (indChosenSegment == -1) return;

  // fit an ellipse to the edge pixels
  vector<pix> ellipseEdgePixels(edgeMap->segments[indChosenSegment].noPixels);
  for (int indEdgePix = 0; indEdgePix < ellipseEdgePixels.size();
       indEdgePix++) {
    pix p;
    p.x = edgeMap->segments[indChosenSegment].pixels[indEdgePix].c;
    p.y = edgeMap->segments[indChosenSegment].pixels[indEdgePix].r;
    ellipseEdgePixels[indEdgePix] = p;
  }
  customEllipse ellipse(ellipseEdgePixels.data(), ellipseEdgePixels.size());

  // form the conic matrix
  double coeffs[6];
  ellipse.GetCoefficients(coeffs);
  marker.C = Mat(3, 3, CV_64FC1);
  marker.C.at<double>(0, 0) = coeffs[0];
  marker.C.at<double>(0, 1) = marker.C.at<double>(1, 0) = -coeffs[1] / 2;
  marker.C.at<double>(1, 1) = coeffs[2];
  marker.C.at<double>(0, 2) = marker.C.at<double>(2, 0) = coeffs[3] / 2;
  marker.C.at<double>(1, 2) = marker.C.at<double>(2, 1) = -coeffs[4] / 2;
  marker.C.at<double>(2, 2) = coeffs[5];

  // refine the pose
  optEllipse = marker.C;
  cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
  cv::Ptr<cv::MinProblemSolver::Function> ptr_F = cv::makePtr<Refine>();
  solver->setFunction(ptr_F);
  cv::Mat x(1, 9, CV_64FC1);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      x.at<double>(i + j * 3) = marker.H.at<double>(i, j);
  cv::Mat step(9, 1, CV_64FC1);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      step.at<double>(i + j * 3) = abs(0.001 * x.at<double>(i + j * 3));
  solver->setInitStep(step);
  double res = solver->minimize(x);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      marker.H.at<double>(i, j) = x.at<double>(i + j * 3);

  // update points
  marker.center = projectPoint(Point2d(0.5, 0.5), marker.H);
  marker.corners[0] = projectPoint(Point2d(0, 0), marker.H);
  marker.corners[1] = projectPoint(Point2d(1, 0), marker.H);
  marker.corners[2] = projectPoint(Point2d(1, 1), marker.H);
  marker.corners[3] = projectPoint(Point2d(0, 1), marker.H);
}

Point2d PoseRefiner::projectPoint(Point2d p, Mat H) {
  Mat pMat = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
  Mat projPMat = H * pMat;
  return Point2d(projPMat.at<double>(0) / projPMat.at<double>(2),
                 projPMat.at<double>(1) / projPMat.at<double>(2));
}

bool PoseRefiner::checkIfPointInQuad(const Marker& marker, const Point2d& p) {
  // check if point p is in the fan of each corner
  Point2d c1c2, c1c4, c3c2, c3c4, c1p, c3p;

  c1c2 = Point2d(marker.corners[1].x - marker.corners[0].x,
                 marker.corners[1].y - marker.corners[0].y);
  c1c4 = Point2d(marker.corners[3].x - marker.corners[0].x,
                 marker.corners[3].y - marker.corners[0].y);
  c3c2 = Point2d(marker.corners[1].x - marker.corners[2].x,
                 marker.corners[1].y - marker.corners[2].y);
  c3c4 = Point2d(marker.corners[3].x - marker.corners[2].x,
                 marker.corners[3].y - marker.corners[2].y);

  c1p = Point2d(p.x - marker.corners[0].x, p.y - marker.corners[0].y);
  c3p = Point2d(p.x - marker.corners[2].x, p.y - marker.corners[2].y);

  if (crossProduct(c1p, c1c2) * crossProduct(c1p, c1c4) >= 0) return false;
  if (crossProduct(c1c2, c1p) * crossProduct(c1c2, c1c4) <= 0) return false;

  if (crossProduct(c3p, c3c2) * crossProduct(c3p, c3c4) >= 0) return false;
  if (crossProduct(c3c2, c3p) * crossProduct(c3c2, c3c4) <= 0) return false;

  return true;
}

class Refine : public cv::MinProblemSolver::Function {
 public:
  int getDims() const { return 9; }
  double calc(const double* x) const {
    Mat H, HT, Hi;
    H = Mat(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) H.at<double>(i, j) = x[i + j * 3];
    }
    cv::transpose(H, HT);
    Hi = H.inv();

    // project the detected ellipse using the current homography matrix
    Mat projCircle = HT * optEllipse * H;

    vector<double> projEllipseCoeff(6);
    projEllipseCoeff[0] = projCircle.at<double>(0, 0);
    projEllipseCoeff[1] = -projCircle.at<double>(0, 1) * 2;
    projEllipseCoeff[2] = projCircle.at<double>(1, 1);
    projEllipseCoeff[3] = projCircle.at<double>(0, 2) * 2;
    projEllipseCoeff[4] = -projCircle.at<double>(1, 2) * 2;
    projEllipseCoeff[5] = projCircle.at<double>(2, 2);

    // construct the ellipse using coefficients (to calculate semi major/minor
    // axes)
    customEllipse projEllipse(projEllipseCoeff.data());

    vector<double> errors;
    errors.push_back(abs(projEllipse.GetSemiMajorAxis() - 0.4));
    errors.push_back(abs(projEllipse.GetSemiMinorAxis() - 0.4));
    errors.push_back(abs(projEllipse.GetCenterX() - 0.5));
    errors.push_back(abs(projEllipse.GetCenterY() - 0.5));
    return std::accumulate(errors.begin(), errors.end(), (double)0);
  }
};