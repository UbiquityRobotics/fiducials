// Burak - Periodical line fits addeed to SplitSegment2Lines

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "stag/ED/EDInternals.h"
#include "stag/ED/ImageSmooth.h"
#include "stag/ED/GradientOperators.h"

#include "stag/ED/ED.h"
#include "stag/ED/EDLines.h"
#include "stag/ED/LineSegment.h"
#include "stag/ED/NFA.h"
#include "stag/ED/MyMath.h"
// #include "Timer.h"

/** PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif /* !M_PI */

#ifndef FALSE
#define FALSE 0
#endif /* !FALSE */

#ifndef TRUE
#define TRUE 1
#endif /* !TRUE */

/// Goes over the original line segments and joins collinear lines that belong
/// to the same segment
void JoinCollinearLines(EDLines *lines,
                        double MAX_DISTANCE_BETWEEN_TWO_LINES = 6.0,
                        double MAX_ERROR = 1.5);

///----------------------------------------------------------------------------
/// Dumps lines to file
// ///
// void DumpLines2File(EDLines *lines, char *fname) {
// // Burak - suppresses _CRT_SECURE_NO_DEPRECATE warnings
// #pragma warning(disable : 4996)

//   FILE *fp = fopen(fname, "w");

// // Burak - restores _CRT_SECURE_NO_DEPRECATE warnings
// #pragma warning(default : 4996)

//   /// Dump lines to the file
//   fprintf(fp,
//           "+=======+===========+=======+=========+=== LINES "
//           "=====+========+========+========+========+========+========+\n");
//   fprintf(fp,
//           "| LineNo| SegmentNo |   a   |    b    |invert|   sx   |   sy   |   "
//           "ex   |   ey   |  Angle |Distance| Length |\n");
//   fprintf(fp,
//           "+=======+===========+=======+=========+===============+========+===="
//           "====+========+========+========+========+\n");

//   int i = 0;
//   while (i < lines->noLines) {
//     fprintf(fp,
//             "+-------+-----------+-------+---------+---------------+--------+--"
//             "------+--------+--------+--------+--------+\n");

//     int segmentNo = lines->lines[i].segmentNo;
//     int count = 1;
//     int firstLineIndex = i;
//     int lastLineIndex = i + 1;

//     while (lastLineIndex < lines->noLines &&
//            lines->lines[lastLineIndex].segmentNo == segmentNo) {
//       count++;
//       lastLineIndex++;
//     }  // end-while

//     i += count;
//     lastLineIndex--;

//     for (int j = firstLineIndex; j <= lastLineIndex; j++) {
//       double len = LineSegmentLength(&lines->lines[j]);
//       double dist = -1;
//       double angle = -1;
//       int next = j + 1;
//       if (next > lastLineIndex) next = firstLineIndex;

//       if (next != j) {
//         angle = ComputeAngleBetweenTwoLines(&lines->lines[j],
//                                             &lines->lines[next], &dist);
//       }  // end-if

//       fprintf(fp,
//               "|   %3d |    %3d    "
//               "|%7.1lf|%9.5lf|%6d|%8.1lf|%8.1lf|%8.1lf|%8.1lf|%8.1lf|%8.1lf|%8."
//               "1lf|\n",
//               j, lines->lines[j].segmentNo, lines->lines[j].a,
//               lines->lines[j].b, lines->lines[j].invert, lines->lines[j].sx,
//               lines->lines[j].sy, lines->lines[j].ex, lines->lines[j].ey, angle,
//               dist, len);
//     }  // end-for
//   }    // end-for
//   fprintf(fp,
//           "+=======+===========+=======+=========+===============+========+===="
//           "====+========+========+========+========+\n");

//   fclose(fp);
// }  // end-DumpLines2File

///------------------------------------------------------------------
/// Goes over the original line segments and combines collinear lines that
/// belong to the same segment
///
void JoinCollinearLines(EDLines *lines, double MAX_DISTANCE_BETWEEN_TWO_LINES,
                        double MAX_ERROR) {
  int lastLineIndex = -1;  // Index of the last line in the joined lines
  int i = 0;
  while (i < lines->noLines) {
    int segmentNo = lines->lines[i].segmentNo;

    lastLineIndex++;
    if (lastLineIndex != i)
      lines->lines[lastLineIndex] = lines->lines[i];  // copy the line
    int firstLineIndex =
        lastLineIndex;  // Index of the first line in this segment

    int count = 1;
    for (int j = i + 1; j < lines->noLines; j++) {
      if (lines->lines[j].segmentNo != segmentNo) break;

      // Try to combine this line with the previous line in this segment
      if (TryToJoinTwoLineSegments(
              &lines->lines[lastLineIndex], &lines->lines[j],
              MAX_DISTANCE_BETWEEN_TWO_LINES, MAX_ERROR) == false) {
        lastLineIndex++;
        if (lastLineIndex != j)
          lines->lines[lastLineIndex] = lines->lines[j];  // copy the line
      }                                                   // end-if

      count++;
    }  // end-for

    // Try to join the first & last line of this segment
    if (firstLineIndex != lastLineIndex) {
      if (TryToJoinTwoLineSegments(&lines->lines[firstLineIndex],
                                   &lines->lines[lastLineIndex],
                                   MAX_DISTANCE_BETWEEN_TWO_LINES, MAX_ERROR)) {
        lastLineIndex--;
      }  // end-if
    }    // end-if

    i += count;
  }  // end-while

  lines->noLines = lastLineIndex + 1;
}  // end-JoinCollinearLines

///-----------------------------------------------------------------
/// Given a full segment of pixels, splits the chain to lines
/// This code is used when we use the whole segment of pixels
///
void SplitSegment2Lines(double *x, double *y, int noPixels, int segmentNo,
                        EDLines *lines) {
  double LINE_ERROR = lines->LINE_ERROR;
  int MIN_LINE_LEN = lines->MIN_LINE_LEN;

  // First pixel of the line segment within the segment of points
  int firstPixelIndex = 0;

  while (noPixels >= MIN_LINE_LEN) {
    // Start by fitting a line to MIN_LINE_LEN pixels
    bool valid = false;
    double lastA, lastB, error;
    int lastInvert;

    while (noPixels >= MIN_LINE_LEN) {
      LineFit(x, y, MIN_LINE_LEN, &lastA, &lastB, &error, &lastInvert);
      if (error <= 0.5) {
        valid = true;
        break;
      }

#if 1
      noPixels -= 1;  // Go slowly
      x += 1;
      y += 1;
      firstPixelIndex += 1;
#else
      noPixels -= 2;  // Go faster (for speed)
      x += 2;
      y += 2;
      firstPixelIndex += 2;
#endif
    }  // end-while

    if (valid == false) return;

    // Now try to extend this line
    int index = MIN_LINE_LEN;
    int len = MIN_LINE_LEN;

    while (index < noPixels) {
      int startIndex = index;
      int lastGoodIndex = index - 1;
      int goodPixelCount = 0;
      int badPixelCount = 0;
      while (index < noPixels) {
        double d =
            ComputeMinDistance(x[index], y[index], lastA, lastB, lastInvert);

        if (d <= LINE_ERROR) {
          lastGoodIndex = index;
          goodPixelCount++;
          badPixelCount = 0;
        } else {
          badPixelCount++;
          if (badPixelCount >= 5) break;
        }  // end-if

        // Burak - Fits a new line every 10 good pixel
        if (goodPixelCount % 10 == 0)
          LineFit(x, y, lastGoodIndex - startIndex + len + 1, &lastA, &lastB,
                  lastInvert);
        // Burak - Fits a new line every 10 good pixel
        index++;
      }  // end-while

      if (goodPixelCount >= 2) {
        len += lastGoodIndex - startIndex + 1;
        LineFit(x, y, len, &lastA, &lastB, lastInvert);  // faster LineFit
        index = lastGoodIndex + 1;
      }  // end-if

      if (goodPixelCount < 2 || index >= noPixels) {
        // End of a line segment. Compute the end points
        double sx, sy, ex, ey;

        int index = 0;
        while (ComputeMinDistance(x[index], y[index], lastA, lastB,
                                  lastInvert) > LINE_ERROR)
          index++;
        ComputeClosestPoint(x[index], y[index], lastA, lastB, lastInvert, &sx,
                            &sy);
        int noSkippedPixels = index;

        index = lastGoodIndex;
        while (ComputeMinDistance(x[index], y[index], lastA, lastB,
                                  lastInvert) > LINE_ERROR)
          index--;
        ComputeClosestPoint(x[index], y[index], lastA, lastB, lastInvert, &ex,
                            &ey);

        // Add the line segment to lines
        lines->add(lastA, lastB, lastInvert, sx, sy, ex, ey, segmentNo,
                   firstPixelIndex + noSkippedPixels,
                   index - noSkippedPixels + 1);

        len = index + 1;
        break;
      }  // end-else
    }    // end-while

    noPixels -= len;
    x += len;
    y += len;
    firstPixelIndex += len;
  }  // end-while
}  // end-SplitSegment2Lines

///---------------------------------------------------------------------
/// Given a set of lines, validates them. This is the fast code
/// This code checks the directions of ONLY the pixels on the line
///
void ValidateLineSegments(EdgeMap *map, unsigned char *srcImg, EDLines *lines,
                          EDLines *invalidLines) {
  bool ValidateLineSegmentRect(unsigned char *srcImg, int width, int height,
                               int *x, int *y, LineSegment *ls, NFALUT *LUT);
  // Timer timer;

  int width = map->width;
  int height = map->height;

  int *x = new int[(width + height) * 4];
  int *y = new int[(width + height) * 4];

#define PRECISON_ANGLE 22.5
  double prec = (PRECISON_ANGLE / 180) * M_PI;
  double prob = 0.125;
#undef PRECISON_ANGLE

  double logNT = 2.0 * (log10((double)width) + log10((double)height));

  /// Compute LUT for NFA computation
  // timer.Start();

  NFALUT *LUT = new NFALUT((width + height) / 8, prob, logNT);

  // timer.Stop();
  lines->LUTComputationTime = 1;  // timer.ElapsedTime();

  if (invalidLines) invalidLines->clear();

  int noValidLines = 0;
  for (int i = 0; i < lines->noLines; i++) {
    LineSegment *ls = &lines->lines[i];

    // Compute Line's angle
    double lineAngle;

    if (ls->invert == 0) {
      // y = a + bx
      lineAngle = atan(ls->b);

    } else {
      // x = a + by
      lineAngle = atan(1.0 / ls->b);
    }  // end-else

    if (lineAngle < 0) lineAngle += M_PI;

    Pixel *pixels = &map->segments[ls->segmentNo].pixels[ls->firstPixelIndex];
    int noPixels = ls->len;

    bool valid = false;

    // Accept very long lines without testing. They are almost never
    // invalidated.
    if (ls->len >= 80) {
      valid = true;

      // Validate short line segments by a line support region rectangle having
      // width=2
    } else if (ls->len <= 25) {
      valid = ValidateLineSegmentRect(srcImg, width, height, x, y, ls, LUT);

    } else {
      // Longer line segments are first validated by a line support region
      // rectangle having width=1 (for speed) If the line segment is still
      // invalid, then a line support region rectangle having width=2 is tried
      // If the line segment fails both tests, it is discarded
      int aligned = 0;
      int count = 0;
      for (int j = 0; j < noPixels; j++) {
        int r = pixels[j].r;
        int c = pixels[j].c;

        if (r <= 0 || r >= height - 1 || c <= 0 || c >= width - 1) continue;

        count++;

        // compute gx & gy using the simple [-1 -1 -1]
        //                                  [ 1  1  1]  filter in both
        //                                  directions
        // Faster method below
        // A B C
        // D x E
        // F G H
        // gx = (C-A) + (E-D) + (H-F)
        // gy = (F-A) + (G-B) + (H-C)
        //
        // To make this faster:
        // com1 = (H-A)
        // com2 = (C-F)
        // Then: gx = com1 + com2 + (E-D) = (H-A) + (C-F) + (E-D) = (C-A) +
        // (E-D) + (H-F)
        //       gy = com2 - com1 + (G-B) = (H-A) - (C-F) + (G-B) = (F-A) +
        //       (G-B) + (H-C)
        //
        int com1 =
            srcImg[(r + 1) * width + c + 1] - srcImg[(r - 1) * width + c - 1];
        int com2 =
            srcImg[(r - 1) * width + c + 1] - srcImg[(r + 1) * width + c - 1];

        int gx =
            com1 + com2 + srcImg[r * width + c + 1] - srcImg[r * width + c - 1];
        int gy = com1 - com2 + srcImg[(r + 1) * width + c] -
                 srcImg[(r - 1) * width + c];

        double pixelAngle = myAtan2((double)gx, (double)-gy);
        double diff = fabs(lineAngle - pixelAngle);

        if (diff <= prec || diff >= M_PI - prec) aligned++;
      }  // end-for

      // Check validation by NFA computation (fast due to LUT)
      valid = checkValidationByNFA(count, aligned, LUT);
      if (valid == false)
        valid = ValidateLineSegmentRect(srcImg, width, height, x, y, ls, LUT);
    }  // end-else

    if (valid) {
      if (i != noValidLines) lines->lines[noValidLines] = lines->lines[i];
      noValidLines++;

    } else if (invalidLines) {
      invalidLines->add(&lines->lines[i]);
    }  // end-else
  }    // end-for

  lines->noLines = noValidLines;

  delete LUT;

  delete x;
  delete y;

  // timer.Stop();
  lines->lineValidationTime = 1;  // timer.ElapsedTime();
}  // end-ValidateLineSegments

//====================================================================================
/// The following rectangle enumeration code was taken from the LSD distribution
/// and adapted for our purposes. We hope that LSD guys are OK with this :-)
///
/// Enumerate the points within a rectangle of width 2 given its two end points
///
void EnumerateRectPoints(double sx, double sy, double ex, double ey, int ptsx[],
                         int ptsy[], int *pNoPoints) {
  double vxTmp[4], vyTmp[4];
  double vx[4], vy[4];
  int n, offset;

  double x1 = sx;
  double y1 = sy;
  double x2 = ex;
  double y2 = ey;
  double width = 2;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double vLen = sqrt(dx * dx + dy * dy);

  // make unit vector
  dx = dx / vLen;
  dy = dy / vLen;

  /* build list of rectangle corners ordered
     in a circular way around the rectangle */
  vxTmp[0] = x1 - dy * width / 2.0;
  vyTmp[0] = y1 + dx * width / 2.0;
  vxTmp[1] = x2 - dy * width / 2.0;
  vyTmp[1] = y2 + dx * width / 2.0;
  vxTmp[2] = x2 + dy * width / 2.0;
  vyTmp[2] = y2 - dx * width / 2.0;
  vxTmp[3] = x1 + dy * width / 2.0;
  vyTmp[3] = y1 - dx * width / 2.0;

  /* compute rotation of index of corners needed so that the first
     point has the smaller x.

     if one side is vertical, thus two corners have the same smaller x
     value, the one with the largest y value is selected as the first.
   */
  if (x1 < x2 && y1 <= y2)
    offset = 0;
  else if (x1 >= x2 && y1 < y2)
    offset = 1;
  else if (x1 > x2 && y1 >= y2)
    offset = 2;
  else
    offset = 3;

  /* apply rotation of index. */
  for (n = 0; n < 4; n++) {
    vx[n] = vxTmp[(offset + n) % 4];
    vy[n] = vyTmp[(offset + n) % 4];
  }  // end-for

  /* Set a initial condition.

     The values are set to values that will cause 'ri_inc' (that will
     be called immediately) to initialize correctly the first 'column'
     and compute the limits 'ys' and 'ye'.

     'y' is set to the integer value of vy[0], the starting corner.

     'ys' and 'ye' are set to very small values, so 'ri_inc' will
     notice that it needs to start a new 'column'.

     The smaller integer coordinate inside of the rectangle is
     'ceil(vx[0])'. The current 'x' value is set to that value minus
     one, so 'ri_inc' (that will increase x by one) will advance to
     the first 'column'.
   */
  int x = (int)ceil(vx[0]) - 1;
  int y = (int)ceil(vy[0]);
  double ys = -DBL_MAX, ye = -DBL_MAX;

  int noPoints = 0;
  int maxNoOfPoints = (int)(fabs(sx - ex) + fabs(sy - ey)) * 4;
  while (noPoints < maxNoOfPoints) {
    //  while (1){
    /* if not at end of exploration,
       increase y value for next pixel in the 'column' */
    y++;

    /* if the end of the current 'column' is reached,
       and it is not the end of exploration,
       advance to the next 'column' */
    while (y > ye && x <= vx[2]) {
      /* increase x, next 'column' */
      x++;

      /* if end of exploration, return */
      if (x > vx[2]) break;

      /* update lower y limit (start) for the new 'column'.

         We need to interpolate the y value that corresponds to the
         lower side of the rectangle. The first thing is to decide if
         the corresponding side is

           vx[0],vy[0] to vx[3],vy[3] or
           vx[3],vy[3] to vx[2],vy[2]

         Then, the side is interpolated for the x value of the
         'column'. But, if the side is vertical (as it could happen if
         the rectangle is vertical and we are dealing with the first
         or last 'columns') then we pick the lower value of the side
         by using 'inter_low'.
       */
      if ((double)x < vx[3]) {
        /* interpolation */
        if (fabs(vx[0] - vx[3]) <= 0.01) {
          if (vy[0] < vy[3])
            ys = vy[0];
          else if (vy[0] > vy[3])
            ys = vy[3];
          else
            ys = vy[0] + (x - vx[0]) * (vy[3] - vy[0]) / (vx[3] - vx[0]);
        } else
          ys = vy[0] + (x - vx[0]) * (vy[3] - vy[0]) / (vx[3] - vx[0]);

      } else {
        /* interpolation */
        if (fabs(vx[3] - vx[2]) <= 0.01) {
          if (vy[3] < vy[2])
            ys = vy[3];
          else if (vy[3] > vy[2])
            ys = vy[2];
          else
            ys = vy[3] + (x - vx[3]) * (y2 - vy[3]) / (vx[2] - vx[3]);
        } else
          ys = vy[3] + (x - vx[3]) * (vy[2] - vy[3]) / (vx[2] - vx[3]);
      }  // end-else

      /* update upper y limit (end) for the new 'column'.

         We need to interpolate the y value that corresponds to the
         upper side of the rectangle. The first thing is to decide if
         the corresponding side is

           vx[0],vy[0] to vx[1],vy[1] or
           vx[1],vy[1] to vx[2],vy[2]

         Then, the side is interpolated for the x value of the
         'column'. But, if the side is vertical (as it could happen if
         the rectangle is vertical and we are dealing with the first
         or last 'columns') then we pick the lower value of the side
         by using 'inter_low'.
       */
      if ((double)x < vx[1]) {
        /* interpolation */
        if (fabs(vx[0] - vx[1]) <= 0.01) {
          if (vy[0] < vy[1])
            ye = vy[1];
          else if (vy[0] > vy[1])
            ye = vy[0];
          else
            ye = vy[0] + (x - vx[0]) * (vy[1] - vy[0]) / (vx[1] - vx[0]);
        } else
          ye = vy[0] + (x - vx[0]) * (vy[1] - vy[0]) / (vx[1] - vx[0]);

      } else {
        /* interpolation */
        if (fabs(vx[1] - vx[2]) <= 0.01) {
          if (vy[1] < vy[2])
            ye = vy[2];
          else if (vy[1] > vy[2])
            ye = vy[1];
          else
            ye = vy[1] + (x - vx[1]) * (vy[2] - vy[1]) / (vx[2] - vx[1]);
        } else
          ye = vy[1] + (x - vx[1]) * (vy[2] - vy[1]) / (vx[2] - vx[1]);
      }  // end-else

      /* new y */
      y = (int)ceil(ys);
    }  // end-while

    // Are we done?
    if (x > vx[2]) break;

    ptsx[noPoints] = x;
    ptsy[noPoints] = y;
    noPoints++;
  }  // end-while

  *pNoPoints = noPoints;
}  // end-EnumerateRectPoints

///----------------------------------------------------------------------
/// Using the rectangle code from LSD to validate a line segment
/// The idea is to form a rectangle of with 2 over the line segment,
/// and compute the angles of all pixels within the rectangle to validate
/// a line segment. Long line segments can easily be validated by just going
/// over a single chain of pixels, but short line segments must be validates
/// using a rectangle of width 2 because some short valid lines are rejected
/// otherwise The above pixel enumeration code is directly taken from LSD code.
/// Hope they are OK with it :-)
///
bool ValidateLineSegmentRect(unsigned char *srcImg, int width, int height,
                             int *x, int *y, LineSegment *ls, NFALUT *LUT) {
  void EnumerateRectPoints(double sx, double sy, double ex, double ey, int x[],
                           int y[], int *pNoPoints);

#define PI 3.14159265358979323846
#define PRECISON_ANGLE 22.5
  static double prec = (PRECISON_ANGLE / 180) * PI;
  static double prob = 0.125;
#undef PRECISON_ANGLE

  // Compute Line's angle
  double lineAngle;

  if (ls->invert == 0) {
    // y = a + bx
    lineAngle = atan(ls->b);

  } else {
    // x = a + by
    lineAngle = atan(1.0 / ls->b);
  }  // end-else

  if (lineAngle < 0) lineAngle += PI;

  int noPoints = 0;

  // Enumerate all pixels that fall within the bounding rectangle
  EnumerateRectPoints(ls->sx, ls->sy, ls->ex, ls->ey, x, y, &noPoints);

  int count = 0;
  int aligned = 0;

  for (int i = 0; i < noPoints; i++) {
    int r = y[i];
    int c = x[i];

    if (r <= 0 || r >= height - 1 || c <= 0 || c >= width - 1) continue;

    count++;

    // compute gx & gy using the simple [-1 -1 -1]
    //                                  [ 1  1  1]  filter in both directions
    // Faster method below
    // A B C
    // D x E
    // F G H
    // gx = (C-A) + (E-D) + (H-F)
    // gy = (F-A) + (G-B) + (H-C)
    //
    // To make this faster:
    // com1 = (H-A)
    // com2 = (C-F)
    // Then: gx = com1 + com2 + (E-D) = (H-A) + (C-F) + (E-D) = (C-A) + (E-D) +
    // (H-F)
    //       gy = com2 - com1 + (G-B) = (H-A) - (C-F) + (G-B) = (F-A) + (G-B) +
    //       (H-C)
    //
    int com1 =
        srcImg[(r + 1) * width + c + 1] - srcImg[(r - 1) * width + c - 1];
    int com2 =
        srcImg[(r - 1) * width + c + 1] - srcImg[(r + 1) * width + c - 1];

    int gx =
        com1 + com2 + srcImg[r * width + c + 1] - srcImg[r * width + c - 1];
    int gy =
        com1 - com2 + srcImg[(r + 1) * width + c] - srcImg[(r - 1) * width + c];

    double pixelAngle = myAtan2((double)gx, (double)-gy);
    double diff = fabs(lineAngle - pixelAngle);

    if (diff <= prec || diff >= PI - prec) aligned++;
  }  // end-for

  return checkValidationByNFA(count, aligned, LUT);
#undef PI
}  // end-ValidateLineSegmentRect

///-----------------------------------------------------------------------------------------
/// Computes the minimum line length using the NFA formula given width & height
/// values
///
int ComputeMinLineLength(int width, int height) {
  // The reason we are dividing the theoretical minimum line length by 2 is
  // because we now test short line segments by a line support region rectangle
  // having width=2. This means that within a line support region rectangle for
  // a line segment of length "l" there are "2*l" many pixels. Thus, a line
  // segment of length "l" has a chance of getting validated by NFA.

  double logNT = 2.0 * (log10((double)width) + log10((double)height));
  return Round((-logNT / log10(0.125)) * 0.5);
}  // end-ComputeMinLineLength

///----------------------------------------------------------------------------------------------------------------
/// Extract all lines using edge drawing.
/// Valid lines would be those lines that are validated using the Helmholz
/// principle Invalid lines would be those lines that was detected, but erased
/// due to being invalidated by the Helmholtz principle
///
// Burak - original function decleration: EDLines *DetectLinesByED(unsigned char
// *srcImg, int width, int height, double smoothingSigma, EDLines *invalidLines)
// EDLines *DetectLinesByED(EdgeMap *&map, unsigned char *srcImg, int width,
//                          int height, double smoothingSigma,
//                          EDLines *invalidLines) {
//   // Make sure, smoothingSigma <= 2.0. Higher values do not produce good results
//   if (smoothingSigma > 2.0) smoothingSigma = 2.0;

//   EDLines *lines = new EDLines(width, height);

// #define USE_GX_GY_PLUS
// #ifdef USE_GX_GY_PLUS
//   /* used with g = gx+gy */
//   int GRADIENT_THRESH =
//       11;  // 9 seems to give the best results // (1-2), (3-4), (5-6), ..,
//            // (11-12) pairs produce the same gradient map! ANCHOR_THRESH pairs
//            // (1,2), (3,4), (5,6) etc. generate the same number of anchors!

//   if (smoothingSigma >= 2.0)
//     GRADIENT_THRESH += 2;  // higher GRADIENT_THRESH for higher smoothing so
//                            // that we do not get any false detections

// #elif USE_GX_GY_SQRT
//   /* used with g = sqrt(gx*gx+gy*gy) */
//   // This threshold should in fact be 10.45 (4/sin(22.5)=10.45). But, since
//   // we are computing the gradient over the smoothed image, a smaller value must
//   // be used. 8 seems to give results closest to the theoretical threshold
//   // value, and thus results closest to LSD.
//   int GRADIENT_THRESH = 8;  // with g = sqrt(gx*gx+gy*gy)
// #endif

//   int ANCHOR_THRESH = 3;  // smaller values produce noise!

//   /*----------- DETECT EDGES ----------------*/
//   // Timer timer;
//   // timer.Start();

//   // Detect edges by edge drawing
//   unsigned char *smoothImg = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);
//   ComputeGradientMapByLSD(smoothImg, gradImg, dirImg, width, height,
//                           GRADIENT_THRESH);
//   map = DoDetectEdgesByED(gradImg, dirImg, width, height, GRADIENT_THRESH,
//                           ANCHOR_THRESH);

// #if 0
//   void SaveImage(char *, char *, int, int, int);
//   for (int i=0; i<width*height; i++) if (gradImg[i] >= GRADIENT_THRESH) dirImg[i]=255; else dirImg[i]=0;
//   SaveImage("OutputImages/gradImgBW.pgm", (char *)dirImg, width, height, 8);
// #endif

//   delete gradImg;
//   delete dirImg;

//   // timer.Stop();
//   lines->edgeDetectionTime = 1;  // timer.ElapsedTime();

//   /*----------- FIT LINES ----------------*/
//   // timer.Start();

//   // Now, go over the edge segments & fit lines
//   lines->clear();

//   lines->MIN_LINE_LEN = ComputeMinLineLength(width, height);

//   // Too small?
//   //  if (lines->MIN_LINE_LEN < 8) lines->MIN_LINE_LEN = 8;

//   // Min line length 9 seems to give the closest results to LSD. Keep this?
//   if (lines->MIN_LINE_LEN < 9) lines->MIN_LINE_LEN = 9;

//   // A min line length of 10 gives less lines than LSD, but the lines are much
//   // cleaner
//   //  if (lines->MIN_LINE_LEN < 10) lines->MIN_LINE_LEN = 10;

//   double *x = lines->x;
//   double *y = lines->y;

//   // Use the whole segment
//   for (int segmentNo = 0; segmentNo < map->noSegments; segmentNo++) {
//     EdgeSegment *segment = &map->segments[segmentNo];

//     for (int k = 0; k < segment->noPixels; k++) {
//       x[k] = segment->pixels[k].c;
//       y[k] = segment->pixels[k].r;
//     }  // end-for

//     SplitSegment2Lines(x, y, segment->noPixels, segmentNo, lines);
//   }  // end-for

//   // timer.Stop();
//   lines->lineFitTime = 1;  // timer.ElapsedTime();

//   /*----------- JOIN COLLINEAR LINES ----------------*/
//   // timer.Start();

//   JoinCollinearLines(lines, 6.0, 1.30);

//   // timer.Stop();
//   lines->joinLineSegmentsTime = 1;  // timer.ElapsedTime();

//   /*----------- VALIDATE LINES ----------------*/
//   // timer.Start();

// #if 0
//   // Validate with unsmoothed image
//   ValidateLineSegments(map, srcImg, lines, invalidLines);
// #else
//   // Validate with smoothed image -- good especially for noisy images
//   double validationSigma =
//       smoothingSigma / 2.5;  // Gives one false detection in noisy images, but I
//                              // think this is better in general
//                              //  double validationSigma = smoothingSigma/3;
//   SmoothImage(srcImg, smoothImg, width, height, validationSigma);
//   ValidateLineSegments(map, smoothImg, lines, invalidLines);
// #endif

//   // timer.Stop();
//   lines->lineValidationTime = 1;  // timer.ElapsedTime();

//   delete smoothImg;
//   // Burak - need to return this, don't delete
//   // delete map;

//   return lines;
// }  // end-DetectLinesByED

///----------------------------------------------------------------------------------------------------------------
/// Extract all lines using edge drawing parameter free (EDPF)
/// Valid lines would be those lines that are validated using the Helmholz
/// principle Invalid lines would be those lines that was detected, but erased
/// due to being invalidated by the Helmholtz principle
///
// Burak - original function decleration: EDLines *DetectLinesByEDPF(unsigned
// char *srcImg, int width, int height, EDLines *invalidLines){
EDLines *DetectLinesByEDPF(EdgeMap *&map, unsigned char *srcImg, int width,
                           int height, bool flagOnlyUseEdgeSegmentLoops,
                           int thresManhDist) {
  // Burak - added the next line
  EDLines *invalidLines = NULL;
  EDLines *lines = new EDLines(width, height);

  /*----------- DETECT EDGES ----------------*/
  // Timer timer;
  // timer.Start();

  // Detect edges by edge drawing parameter free (EDPF)
  map = DetectEdgesByEDPF(srcImg, width, height);  // Detect edges by EDPF

  // Burak - find edge loops
  bool *validSegments = new bool[map->noSegments];
  for (int i = 0; i < map->noSegments; i++) {
    if (abs(map->segments[i].pixels[0].c -
            map->segments[i].pixels[map->segments[i].noPixels - 1].c) +
            abs(map->segments[i].pixels[0].r -
                map->segments[i].pixels[map->segments[i].noPixels - 1].r) <
        thresManhDist)
      validSegments[i] = true;
    else
      validSegments[i] = false;
  }
  // Burak - find edge loops

  // timer.Stop();
  lines->edgeDetectionTime = 1;  // timer.ElapsedTime();

  /*----------- FIT LINES ----------------*/
  // timer.Start();

  // Now, go over the edge segments & fit lines
  // lines->clear();

  lines->MIN_LINE_LEN = ComputeMinLineLength(width, height);

  // Too small?
  //  if (lines->MIN_LINE_LEN < 8) lines->MIN_LINE_LEN = 8;

  // Min line length 9 seems to give the closest results to LSD. Keep this?
  if (lines->MIN_LINE_LEN < 9) lines->MIN_LINE_LEN = 9;

  // A min line length of 10 gives less lines than LSD, but the lines are much
  // cleaner
  //  if (lines->MIN_LINE_LEN < 10) lines->MIN_LINE_LEN = 10;

  double *x = lines->x;
  double *y = lines->y;

  // Use the whole segment
  for (int segmentNo = 0; segmentNo < map->noSegments; segmentNo++) {
    // Burak - skip the segment if it's not a loop
    if (flagOnlyUseEdgeSegmentLoops && !validSegments[segmentNo]) continue;

    EdgeSegment *segment = &map->segments[segmentNo];

    for (int k = 0; k < segment->noPixels; k++) {
      x[k] = segment->pixels[k].c;
      y[k] = segment->pixels[k].r;
    }  // end-for

    SplitSegment2Lines(x, y, segment->noPixels, segmentNo, lines);
  }  // end-for

  // Burak - added next line
  delete[] validSegments;

  // timer.Stop();
  lines->lineFitTime = 1;  // timer.ElapsedTime();

  /*----------- JOIN COLLINEAR LINES ----------------*/
  // timer.Start();

  JoinCollinearLines(lines, 6.0, 1.50);

  // timer.Stop();
  lines->joinLineSegmentsTime = 1;  // timer.ElapsedTime();

  /*----------- VALIDATE LINES ----------------*/
  // timer.Start();

  ValidateLineSegments(map, srcImg, lines, invalidLines);

  // timer.Stop();
  lines->lineValidationTime = 1;  // timer.ElapsedTime();

  // Burak - need to return this
  // delete map;
  return lines;
}  // end-DetectLinesByEDPF
