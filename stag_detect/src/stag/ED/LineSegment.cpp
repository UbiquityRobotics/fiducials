#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stag/ED/LineSegment.h"

///------------------------------------------------------------------
/// Rounds a double number to its closest integer part.
/// E.g., 4.24-->4, 4.78-->5
///
int Round(double d) { return (int)(d + 0.5); }  // end-Round

///-------------------------------------------------------------------------------
/// Simply returns the length of a line segment
///
// double LineSegmentLength(LineSegment *ls) {
//   double dx = ls->sx - ls->ex;
//   double dy = ls->sy - ls->ey;

//   return sqrt(dx * dx + dy * dy);
// }  // end-LineSegmentLength

///-------------------------------------------------------------------------------
/// Computes the angle between two line segments
/// Assumes that (ex, ey) of ls1 is closer to (sx, ey) of ls2
///
// double ComputeAngleBetweenTwoLines(LineSegment *ls1, LineSegment *ls2,
//                                    double *pMinDist) {
//   double vx1 = ls1->ex - ls1->sx;
//   double vy1 = ls1->ey - ls1->sy;
//   double v1Len = sqrt(vx1 * vx1 + vy1 * vy1);

//   double vx2 = ls2->ex - ls2->sx;
//   double vy2 = ls2->ey - ls2->sy;
//   double v2Len = sqrt(vx2 * vx2 + vy2 * vy2);

//   double dotProduct = vx1 * vx2 + vy1 * vy2;
//   double result = dotProduct / (v1Len * v2Len);
//   if (result < -1.0)
//     result = -1.0;
//   else if (result > 1.0)
//     result = 1.0;
//   double angle = acos(result);

// #define PI 3.14159
//   angle = (angle / PI) * 180;  // convert to degrees

//   if (pMinDist) {
//     //  Compute the distance between (ex, ey) of ls1 & (sx, sy) of ls2
//     double dx = ls1->ex - ls2->sx;
//     double dy = ls1->ey - ls2->sy;

//     *pMinDist = sqrt(dx * dx + dy * dy);
//   }  // end-if

//   return angle;
// }  // end-ComputeAngleBetweenTwoLines

///-------------------------------------------------------------------------------
/// Computes the angle (in degrees) between two line segments
/// Also returns the minimum distance between the end points of the two lines
/// Which returns a value that designates which endpoints are closer
/// The meaning of this value is the following:
/// SS -- 00 - 0  (sx1, sy1) <--> (sx2, sy2)
/// SE -- 01 - 1  (sx1, sy1) <--> (ex2, ey2)
/// ES -- 10 - 2  (ex1, ey1) <--> (sx2, sy2)
/// EE -- 11 - 3  (ex1, ey1) <--> (ex2, ey2)
///
#define SS 0
#define SE 1
#define ES 2
#define EE 3
// double ComputeAngleBetweenTwoLines2(LineSegment *ls1, LineSegment *ls2,
//                                     double *pMinDist, int *pwhich) {
//   double dx = ls1->sx - ls2->sx;
//   double dy = ls1->sy - ls2->sy;
//   double d = sqrt(dx * dx + dy * dy);
//   double min = d;
//   int which = SS;

//   dx = ls1->sx - ls2->ex;
//   dy = ls1->sy - ls2->ey;
//   d = sqrt(dx * dx + dy * dy);
//   if (d < min) {
//     min = d;
//     which = SE;
//   }

//   dx = ls1->ex - ls2->sx;
//   dy = ls1->ey - ls2->sy;
//   d = sqrt(dx * dx + dy * dy);
//   if (d < min) {
//     min = d;
//     which = ES;
//   }

//   dx = ls1->ex - ls2->ex;
//   dy = ls1->ey - ls2->ey;
//   d = sqrt(dx * dx + dy * dy);
//   if (d < min) {
//     min = d;
//     which = EE;
//   }

//   if (pMinDist) *pMinDist = min;
//   if (pwhich) *pwhich = which;

//   double vx1, vy1, vx2, vy2;

//   switch (which) {
//     case SS:
//       vx1 = ls1->ex - ls1->sx;
//       vy1 = ls1->ey - ls1->sy;

//       vx2 = ls2->ex - ls2->sx;
//       vy2 = ls2->ey - ls2->sy;
//       break;

//     case SE:
//       vx1 = ls1->ex - ls1->sx;
//       vy1 = ls1->ey - ls1->sy;

//       vx2 = ls2->sx - ls2->ex;
//       vy2 = ls2->sy - ls2->ey;
//       break;

//     case ES:
//       vx1 = ls1->sx - ls1->ex;
//       vy1 = ls1->sy - ls1->ey;

//       vx2 = ls2->ex - ls2->sx;
//       vy2 = ls2->ey - ls2->sy;
//       break;

//     case EE:
//       vx1 = ls1->sx - ls1->ex;
//       vy1 = ls1->sy - ls1->ey;

//       vx2 = ls2->sx - ls2->ex;
//       vy2 = ls2->sy - ls2->ey;
//       break;
//   }  // end-case

//   double v1Len = sqrt(vx1 * vx1 + vy1 * vy1);
//   double v2Len = sqrt(vx2 * vx2 + vy2 * vy2);

//   double dotProduct = vx1 * vx2 + vy1 * vy2;
//   double result = dotProduct / (v1Len * v2Len);
//   if (result < -1.0)
//     result = -1.0;
//   else if (result > 1.0)
//     result = 1.0;
//   double angle = acos(result);

// #define PI 3.14159
//   angle = (angle / PI) * 180;  // convert to degrees

//   // Return the angle
//   return angle;
// }  // end-ComputeAngleBetweenTwoLines2

///-------------------------------------------------------------------------------
/// Computes the minimum distance between the end points of two lines
///
double ComputeMinDistanceBetweenTwoLines(LineSegment *ls1, LineSegment *ls2,
                                         int *pwhich) {
  double dx = ls1->sx - ls2->sx;
  double dy = ls1->sy - ls2->sy;
  double d = sqrt(dx * dx + dy * dy);
  double min = d;
  int which = SS;

  dx = ls1->sx - ls2->ex;
  dy = ls1->sy - ls2->ey;
  d = sqrt(dx * dx + dy * dy);
  if (d < min) {
    min = d;
    which = SE;
  }

  dx = ls1->ex - ls2->sx;
  dy = ls1->ey - ls2->sy;
  d = sqrt(dx * dx + dy * dy);
  if (d < min) {
    min = d;
    which = ES;
  }

  dx = ls1->ex - ls2->ex;
  dy = ls1->ey - ls2->ey;
  d = sqrt(dx * dx + dy * dy);
  if (d < min) {
    min = d;
    which = EE;
  }

  if (pwhich) *pwhich = which;
  return min;
}  // end-ComputeMinDistanceBetweenTwoLines

///-------------------------------------------------------------------------------
/// Computes the intersection point of the two lines ls1 & ls2
/// Assumes that the lines are NOT collinear
///
// void FindIntersectionPoint(LineSegment *ls1, LineSegment *ls2, double *xInt,
//                            double *yInt) {
//   double x = 0.0;
//   double y = 0.0;

//   if (ls1->invert == 0 && ls2->invert == 0) {
//     // Both lines are of the form  y = bx + a
//     x = (ls2->a - ls1->a) / (ls1->b - ls2->b);
//     y = ls1->b * x + ls1->a;

//   } else if (ls1->invert == 0 && ls2->invert == 1) {
//     // LS1 is of the form y = bx + a, LS2 is of the form x = by + a
//     x = (ls2->b * ls1->a + ls2->a) / (1.0 - ls2->b * ls1->b);
//     y = ls1->b * x + ls1->a;

//   } else if (ls1->invert == 1 && ls2->invert == 0) {
//     // LS1 is of the form x = by + a and LS2 is of the form y = bx + a
//     y = (ls2->b * ls1->a + ls2->a) / (1.0 - ls1->b * ls2->b);
//     x = ls1->b * y + ls1->a;

//   } else {  // ls1->invert == 1 && ls2->invert == 1
//     // Both lines are of the form x = by + a
//     y = (ls1->a - ls2->a) / (ls2->b - ls1->b);
//     x = ls1->b * y + ls1->a;
//   }  // end-else

//   *xInt = x;
//   *yInt = y;
// }  // end-FindIntersectionPoint

///-----------------------------------------------------------------
/// Checks if the given line segments are collinear & joins them if they are
/// In case of a join, ls1 is updated. ls2 is NOT changed
/// Returns true if join is successful, false otherwise
///
bool TryToJoinTwoLineSegments(LineSegment *ls1, LineSegment *ls2,
                              double MAX_DISTANCE_BETWEEN_TWO_LINES,
                              double MAX_ERROR) {
  // Must belong to the same segment
  //  if (ls1->segmentNo != ls2->segmentNo) return false;

  int which;
  double dist = ComputeMinDistanceBetweenTwoLines(ls1, ls2, &which);
  if (dist > MAX_DISTANCE_BETWEEN_TWO_LINES) return false;

  // Compute line lengths. Use the longer one as the ground truth
  double dx = ls1->sx - ls1->ex;
  double dy = ls1->sy - ls1->ey;
  double prevLen = sqrt(dx * dx + dy * dy);

  dx = ls2->sx - ls2->ex;
  dy = ls2->sy - ls2->ey;
  double nextLen = sqrt(dx * dx + dy * dy);

  // Use the longer line as the ground truth
  LineSegment *shorter = ls1;
  LineSegment *longer = ls2;

  if (prevLen > nextLen) {
    shorter = ls2;
    longer = ls1;
  }

#if 0
  // Use 5 points to check for collinearity
#define POINT_COUNT 5
  double decr = 1.0/(POINT_COUNT-1);
  double alpha = 1.0;
  dist = 0.0;

  while (alpha >= 0.0){
    double px = alpha*shorter->sx + (1.0-alpha)*shorter->ex;
    double py = alpha*shorter->sy + (1.0-alpha)*shorter->ey;

    dist += ComputeMinDistance(px, py, longer->a, longer->b, longer->invert);

    alpha -= decr;
  } //end-while

  dist /= POINT_COUNT;

#undef POINT_COUNT

#else
  // Just use 3 points to check for collinearity
  dist = ComputeMinDistance(shorter->sx, shorter->sy, longer->a, longer->b,
                            longer->invert);
  dist += ComputeMinDistance((shorter->sx + shorter->ex) / 2.0,
                             (shorter->sy + shorter->ey) / 2.0, longer->a,
                             longer->b, longer->invert);
  dist += ComputeMinDistance(shorter->ex, shorter->ey, longer->a, longer->b,
                             longer->invert);

  dist /= 3.0;
#endif

  if (dist > MAX_ERROR) return false;

#if 0
  // Update the end points of ls1
  if (which == 0) {       // SS
    ls1->sx = ls2->ex;
    ls1->sy = ls2->ey;

  } else if (which == 1){ // SE
    ls1->sx = ls2->sx;
    ls1->sy = ls2->sy;

  } else if (which == 2){ // ES
    ls1->ex = ls2->ex;
    ls1->ey = ls2->ey;
    
  } else {                // EE
    ls1->ex = ls2->sx;
    ls1->ey = ls2->sy;
  } //end-else

#else
  /// 4 cases: 1:(s1, s2), 2:(s1, e2), 3:(e1, s2), 4:(e1, e2)

  /// case 1: (s1, s2)
  dx = fabs(ls1->sx - ls2->sx);
  dy = fabs(ls1->sy - ls2->sy);
  double d = dx + dy;
  double max = d;
  which = 1;

  /// case 2: (s1, e2)
  dx = fabs(ls1->sx - ls2->ex);
  dy = fabs(ls1->sy - ls2->ey);
  d = dx + dy;
  if (d > max) {
    max = d;
    which = 2;
  }  // end-if

  /// case 3: (e1, s2)
  dx = fabs(ls1->ex - ls2->sx);
  dy = fabs(ls1->ey - ls2->sy);
  d = dx + dy;
  if (d > max) {
    max = d;
    which = 3;
  }  // end-if

  /// case 4: (e1, e2)
  dx = fabs(ls1->ex - ls2->ex);
  dy = fabs(ls1->ey - ls2->ey);
  d = dx + dy;
  if (d > max) {
    max = d;
    which = 4;
  }  // end-if

  if (which == 1) {
    // (s1, s2)
    ls1->ex = ls2->sx;
    ls1->ey = ls2->sy;

  } else if (which == 2) {
    // (s1, e2)
    ls1->ex = ls2->ex;
    ls1->ey = ls2->ey;

  } else if (which == 3) {
    // (e1, s2)
    ls1->sx = ls2->sx;
    ls1->sy = ls2->sy;

  } else {
    // (e1, e2)
    ls1->sx = ls1->ex;
    ls1->sy = ls1->ey;

    ls1->ex = ls2->ex;
    ls1->ey = ls2->ey;
  }  // end-else

#endif

  // Update the first line's parameters
  if (ls1->firstPixelIndex + ls1->len + 5 >= ls2->firstPixelIndex)
    ls1->len += ls2->len;
  else if (ls2->len > ls1->len) {
    ls1->firstPixelIndex = ls2->firstPixelIndex;
    ls1->len = ls2->len;
  }  // end-if

  UpdateLineParameters(ls1);

  return true;
}  // end-TryToJoinTwoLineSegments

///-------------------------------------------------------------------
/// Traces the points on the line from (sx, sy) --> (ex, ey)
/// using only integer arithmetic using the famous Bresenham line tracking
/// algorithm The points on the line are filled into px and py arrays. No of
/// points are also returned
///
// void BresenhamLineTrace(int sx, int sy, int ex, int ey, int px[], int py[],
//                         int *noPoints) {
//   int dx = ex - sx;
//   int dy = ey - sy;
//   int count = 0;

//   if (abs(dx) >= abs(dy)) {
//     int xIncr = 1;
//     if (dx < 0) xIncr = -1;
//     int yIncr = 1;
//     if (dy < 0) yIncr = -1;

//     int incrE, incrNE, d, x, y;
//     d = abs(dy) * 2 - abs(dx);
//     incrE = abs(dy) * 2;
//     incrNE = (abs(dy) - abs(dx)) * 2;
//     x = sx;
//     y = sy;

//     dx = abs(dx);
//     for (int xx = 0; xx <= dx; xx++) {
//       px[count] = x;
//       py[count] = y;
//       count++;

//       if (d <= 0) {
//         d += incrE;
//         x += xIncr;

//       } else {
//         d += incrNE;
//         x += xIncr;
//         y += yIncr;
//       }  // end-else
//     }    // end-while

//   } else {
//     int xIncr = 1;
//     if (dx < 0) xIncr = -1;
//     int yIncr = 1;
//     if (dy < 0) yIncr = -1;

//     int incrE, incrNE, d, x, y;
//     d = abs(dx) * 2 - abs(dy);
//     incrE = abs(dx) * 2;
//     incrNE = (abs(dx) - abs(dy)) * 2;
//     x = sx;
//     y = sy;

//     dy = abs(dy);
//     for (int yy = 0; yy <= dy; yy++) {
//       px[count] = x;
//       py[count] = y;
//       count++;

//       if (d <= 0) {
//         d += incrE;
//         y += yIncr;

//       } else {
//         d += incrNE;
//         y += yIncr;
//         x += xIncr;
//       }  // end-else
//     }    // end-while
//   }      // end-else

//   *noPoints = count;
// }  // end-BresenhamLineTrace

// ///---------------------------------------------------------------------------------
// /// Given a point (x1, y1) and a line equation y=a+bx (invert=0) OR x=a+by
// /// (invert=1) Computes the (x2, y2) on the line that is closest to (x1, y1)
// ///
void ComputeClosestPoint(double x1, double y1, double a, double b, int invert,
                         double *xOut, double *yOut) {
  double x2, y2;

  if (invert == 0) {
    if (b == 0) {
      x2 = x1;
      y2 = a;

    } else {
      // Let the line passing through (x1, y1) that is perpendicular to a+bx be
      // c+dx
      double d = -1.0 / (b);
      double c = y1 - d * x1;

      x2 = (a - c) / (d - b);
      y2 = a + b * x2;
    }  // end-else

  } else {
    /// invert = 1
    if (b == 0) {
      x2 = a;
      y2 = y1;

    } else {
      // Let the line passing through (x1, y1) that is perpendicular to a+by be
      // c+dy
      double d = -1.0 / (b);
      double c = x1 - d * y1;

      y2 = (a - c) / (d - b);
      x2 = a + b * y2;
    }  // end-else
  }    // end-else

  *xOut = x2;
  *yOut = y2;
}  // end-ComputeClosestPoint

///---------------------------------------------------------------------------------
/// Given a point (x1, y1) and a line equation y=a+bx (invert=0) OR x=a+by
/// (invert=1) Computes the minimum distance of (x1, y1) to the line
///
double ComputeMinDistance(double x1, double y1, double a, double b,
                          int invert) {
  double x2, y2;

  if (invert == 0) {
    if (b == 0) {
      x2 = x1;
      y2 = a;

    } else {
      // Let the line passing through (x1, y1) that is perpendicular to a+bx be
      // c+dx
      double d = -1.0 / (b);
      double c = y1 - d * x1;

      x2 = (a - c) / (d - b);
      y2 = a + b * x2;
    }  // end-else

  } else {
    /// invert = 1
    if (b == 0) {
      x2 = a;
      y2 = y1;

    } else {
      // Let the line passing through (x1, y1) that is perpendicular to a+by be
      // c+dy
      double d = -1.0 / (b);
      double c = x1 - d * y1;

      y2 = (a - c) / (d - b);
      x2 = a + b * y2;
    }  // end-else
  }    // end-else

  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}  // end-ComputeMinDistance

///-----------------------------------------------------------------------------------
/// Uses the two end points (sx, sy)----(ex, ey) of the line segment
/// and computes the line that passes through these points (a, b, invert)
///
void UpdateLineParameters(LineSegment *ls) {
  double dx = ls->ex - ls->sx;
  double dy = ls->ey - ls->sy;

  if (fabs(dx) >= fabs(dy)) {
    /// Line will be of the form y = a + bx
    ls->invert = 0;
    if (fabs(dy) < 1e-3) {
      ls->b = 0;
      ls->a = (ls->sy + ls->ey) / 2;
    } else {
      ls->b = dy / dx;
      ls->a = ls->sy - (ls->b) * ls->sx;
    }  // end-else

  } else {
    /// Line will be of the form x = a + by
    ls->invert = 1;
    if (fabs(dx) < 1e-3) {
      ls->b = 0;
      ls->a = (ls->sx + ls->ex) / 2;
    } else {
      ls->b = dx / dy;
      ls->a = ls->sx - (ls->b) * ls->sy;
    }  // end-else
  }    // end-else
}  // end-UpdateLineParameters

///-----------------------------------------------------------------------------------
/// Given two points (sx, sy)----(ex, ey), computes the line that passes through
/// these points Assumes that the points are not very close
///
// void ComputeLine(double sx, double sy, double ex, double ey, double *a,
//                  double *b, int *invert) {
//   double dx = ex - sx;
//   double dy = ey - sy;

//   if (fabs(dx) >= fabs(dy)) {
//     /// Line will be of the form y = a + bx
//     *invert = 0;
//     if (fabs(dy) < 1e-3) {
//       *b = 0;
//       *a = (sy + ey) / 2;
//     } else {
//       *b = dy / dx;
//       *a = sy - (*b) * sx;
//     }  // end-else

//   } else {
//     /// Line will be of the form x = a + by
//     *invert = 1;
//     if (fabs(dx) < 1e-3) {
//       *b = 0;
//       *a = (sx + ex) / 2;
//     } else {
//       *b = dx / dy;
//       *a = sx - (*b) * sy;
//     }  // end-else
//   }    // end-else
// }  // end-ComputeLine

///-----------------------------------------------------------------------------------
/// Fits a line of the form y=a+bx (invert == 0) OR x=a+by (invert == 1)
///
void LineFit(double *x, double *y, int count, double *a, double *b, double *e,
             int *invert) {
  if (count < 2) return;

  double S = count, Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
  for (int i = 0; i < count; i++) {
    Sx += x[i];
    Sy += y[i];
  }  // end-for

  double mx = Sx / count;
  double my = Sy / count;

  double dx = 0.0;
  double dy = 0.0;
  for (int i = 0; i < count; i++) {
    dx += (x[i] - mx) * (x[i] - mx);
    dy += (y[i] - my) * (y[i] - my);
  }  // end-for

  if (dx < dy) {
    // Vertical line. Swap x & y, Sx & Sy
    *invert = 1;
    double *t = x;
    x = y;
    y = t;

    double d = Sx;
    Sx = Sy;
    Sy = d;

  } else {
    *invert = 0;
  }  // end-else

  // Now compute Sxx & Sxy
  for (int i = 0; i < count; i++) {
    Sxx += x[i] * x[i];
    Sxy += x[i] * y[i];
  }  // end-for

  double D = S * Sxx - Sx * Sx;
  *a = (Sxx * Sy - Sx * Sxy) / D;
  *b = (S * Sxy - Sx * Sy) / D;

  if (*b == 0.0) {
    // Vertical or horizontal line
    double error = 0.0;
    for (int i = 0; i < count; i++) {
      error += fabs((*a) - y[i]);
    }  // end-for
    *e = error / count;

  } else {
    double error = 0.0;
    for (int i = 0; i < count; i++) {
      // Let the line passing through (x[i], y[i]) that is perpendicular to a+bx
      // be c+dx
      double d = -1.0 / (*b);
      double c = y[i] - d * x[i];
      double x2 = ((*a) - c) / (d - (*b));
      double y2 = (*a) + (*b) * x2;

      double dist = (x[i] - x2) * (x[i] - x2) + (y[i] - y2) * (y[i] - y2);
      error += dist;
    }  // end-for

    *e = sqrt(error / count);
  }  // end-else
}  // end LineFit

///-----------------------------------------------------------------------------------
/// Fits a line of the form y=a+bx (invert == 0) OR x=a+by (invert == 1)
/// Assumes that the direction of the line is known by a previous computation
///
void LineFit(double *x, double *y, int count, double *a, double *b,
             int invert) {
  if (count < 2) return;

  double S = count, Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
  for (int i = 0; i < count; i++) {
    Sx += x[i];
    Sy += y[i];
  }  // end-for

  if (invert) {
    // Vertical line. Swap x & y, Sx & Sy
    double *t = x;
    x = y;
    y = t;

    double d = Sx;
    Sx = Sy;
    Sy = d;
  }  // end-if

  // Now compute Sxx & Sxy
  for (int i = 0; i < count; i++) {
    Sxx += x[i] * x[i];
    Sxy += x[i] * y[i];
  }  // end-for

  double D = S * Sxx - Sx * Sx;
  *a = (Sxx * Sy - Sx * Sxy) / D;
  *b = (S * Sxy - Sx * Sy) / D;
}  // end-LineFit
