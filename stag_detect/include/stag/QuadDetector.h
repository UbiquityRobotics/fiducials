#ifndef QUADDETECTOR_H
#define QUADDETECTOR_H

#include <vector>
#include "stag/EDInterface.h"
#include "stag/Quad.h"

using std::vector;

class Corner {
 public:
  cv::Point2d loc;
  LineSegment l1, l2;
  Corner(cv::Point2d inLoc, LineSegment inL1, LineSegment inL2) {
    loc = inLoc;
    l1 = inL1;
    l2 = inL2;
  }
  Corner() {}
};

class QuadDetector {
  // if keepLogs is true, keep the intermediate results of the detection
  // algorithm in the memory, to be dumped when asked (default: false)
  bool keepLogs = false;

  // thresDist is used to check the following: (default: 7, may need to increase
  // it for larger images) if detected corners are the same with estimated
  // corners if detected corners are on edge segments
  double thresDist = 7;

  // the maximum value of (marker's max distance from the camera) / (marker's
  // min distance from the camera) (default: 1.5)
  double thresProjectiveDistortion = 1.5;

  vector<vector<Corner>> cornerGroups;
  vector<Quad> distortedQuads;
  vector<Quad> quads;

  // forms groups of line segments that may form a quad (line segments must be
  // >=4 and from the same edge segment) each line group is represented by a
  // vector<int>, and each line segment is represented by an int (its index)
  vector<vector<int>> groupLines(const cv::Mat& image,
                                 EDInterface* edInterface);

  // detects corners as intersections of consecutive line segments on an edge
  // segment (with the condition that the intersection lies on the same edge
  // segment)
  void detectCorners(EDInterface* edInterface,
                     const vector<vector<int>>& lineGroups);

  // returns true if the corners form a quad
  // will replace corners[1] or corners[3] with estimated corners if needed
  // orders corners in clockwise
  bool checkIfCornersFormQuad(vector<Corner>& corners,
                              EDInterface* edInterface);

  // checks if the quad is simple or intersects itself
  bool checkIfQuadIsSimple(const vector<Corner>& corners);

  // checks if two corners are facing each other
  bool checkIfTwoCornersFaceEachother(const Corner& c1, const Corner& c2);

 public:
  QuadDetector(bool inKeepLogs = false);

  void detectQuads(const cv::Mat& image, EDInterface* edInterface);

  const vector<vector<Corner>>& getCornerGroups();

  const vector<Quad>& getQuads() const;

  const vector<Quad>& getDistortedQuads() const;
};

#endif