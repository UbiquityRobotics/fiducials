#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

struct LineSegment {
  double a, b;  // y = a + bx (if invert = 0) || x = a + by (if invert = 1)
  int invert;

  double sx, sy;  // starting x & y coordinates
  double ex, ey;  // ending x & y coordinates

  int segmentNo;        // Edge segment that this line belongs to
  int firstPixelIndex;  // Index of the first pixel within the segment of pixels
  int len;              // No of pixels making up the line segment
};

/// Computes the round of a double number
int Round(double d);

/// Simply returns the length of a line segment
// double LineSegmentLength(LineSegment *ls);

/// Computes the minimum distance between the end points of two lines
/// "which" returns a value that designates which endpoints are closest
/// The meaning of this value is the following:
/// SS -- 00 - 0  (sx1, sy1) <--> (sx2, sy2)
/// SE -- 01 - 1  (sx1, sy1) <--> (ex2, ey2)
/// ES -- 10 - 2  (ex1, ey1) <--> (sx2, sy2)
/// EE -- 11 - 3  (ex1, ey1) <--> (ex2, ey2)
double ComputeMinDistanceBetweenTwoLines(LineSegment *ls1, LineSegment *ls2,
                                         int *pwhich = NULL);

/// Computes the angle between two line segments
/// Assumes that (ex, ey) of ls1 is closer to (sx, ey) of ls2
// double ComputeAngleBetweenTwoLines(LineSegment *ls1, LineSegment *ls2,
//                                    double *pMinDist = NULL);

/// Computes the angle (in degrees) between two line segments
// double ComputeAngleBetweenTwoLines2(LineSegment *ls1, LineSegment *ls2,
//                                     double *pMinDist = NULL,
//                                     int *pwhich = NULL);

/// Computes the intersection of two line segments
// void FindIntersectionPoint(LineSegment *ls1, LineSegment *ls2, double *xInt,
//                            double *yInt);

/// Given the start and end of a line segment, computes the points from (sx,
/// sy)-->(ex, ey) using the Bresenham's line trace algorithm
// void BresenhamLineTrace(int sx, int sy, int ex, int ey, int px[], int py[],
//                         int *noPoints);

/// Computes the closest point on the line to a given point (x1, y1)
void ComputeClosestPoint(double x1, double y1, double a, double b, int invert,
                         double *xOut, double *yOut);

/// Computes the closest distance of point (x1, y1) to a given line segment
double ComputeMinDistance(double x1, double y1, double a, double b, int invert);

/// Uses the two end points (sx, sy)----(ex, ey) of the line segment
/// and computes the line that passes through these points (a, b, invert)
void UpdateLineParameters(LineSegment *ls);

/// Checks if the given line segments are collinear & joins them if they are
/// In case of a join, ls1 is updated. ls2 is NOT changed
/// Returns true if join is successful, false otherwise
bool TryToJoinTwoLineSegments(LineSegment *ls1, LineSegment *ls2,
                              double MAX_DISTANCE_BETWEEN_TWO_LINES,
                              double MAX_ERROR);

/// Given two points (sx, sy)----(ex, ey), computes the line that passes through
/// these points Assumes that the points are not very close
// void ComputeLine(double sx, double sy, double ex, double ey, double *a,
//                  double *b, int *invert);

/// Fits a line to a given set of points. Returns the mean square error
void LineFit(double *x, double *y, int count, double *a, double *b, double *e,
             int *invert);

/// This line fit assumes that the direction of the line (invert) is known by a
/// previous computation
void LineFit(double *x, double *y, int count, double *a, double *b, int invert);

#endif