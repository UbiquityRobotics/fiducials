#ifndef EDLINES_H
#define EDLINES_H

#include <memory.h>
// Burak - added EdgeMap.h because of the change in function arguments
#include "stag/ED/EdgeMap.h"
#include "stag/ED/LineSegment.h"

///----------------------------------------------
/// Simple class to manipulate line segments
///
struct EDLines {
 public:
  LineSegment *lines;
  int noLines;
  int capacity;

  // Thresholds used during line segment computation
  double LINE_ERROR;  // Mean square error during line fitting
  int MIN_LINE_LEN;   // Minimum length of the line segments

  // Temporary buffers used during line fitting
  double *x;
  double *y;

  // Used for timing :-)
  double edgeDetectionTime;
  double lineFitTime;
  double joinLineSegmentsTime;
  double lineValidationTime;
  double LUTComputationTime;  // Look Up Table (LUT) computation for line
                              // validation

 public:
  /// Constructor:
  EDLines(int width, int height) {
    int imageSize = width + height;
    capacity = imageSize * 8;
    lines = new LineSegment[capacity];
    noLines = 0;

    LINE_ERROR = 1.0;   // in pixels
    MIN_LINE_LEN = 11;  // in pixels

    edgeDetectionTime = lineFitTime = joinLineSegmentsTime = 0;
    lineValidationTime = LUTComputationTime = 0;

    x = new double[imageSize * 8];
    y = new double[imageSize * 8];
  }  // end-EDLines

  /// Destructor
  ~EDLines() {
    delete lines;
    delete x;
    delete y;
  }  // end-EDLines

  /// clear
  void clear() { noLines = 0; }

  void expandCapacity() {
    capacity *= 2;
    LineSegment *newArr = new LineSegment[capacity];
    memcpy(newArr, lines, sizeof(LineSegment) * noLines);
    delete lines;
    lines = newArr;
  }  // end-expandCapacity

  /// Append a new line to the end of the lines array
  void add(double a, double b, int invert, double sx, double sy, double ex,
           double ey, int segmentNo = 0, int firstPixelIndex = 0, int len = 0) {
    // Full array?
    if (noLines == capacity) expandCapacity();

    lines[noLines].a = a;
    lines[noLines].b = b;
    lines[noLines].invert = invert;
    lines[noLines].sx = sx;
    lines[noLines].sy = sy;
    lines[noLines].ex = ex;
    lines[noLines].ey = ey;

    lines[noLines].segmentNo = segmentNo;
    lines[noLines].firstPixelIndex = firstPixelIndex;
    lines[noLines].len = len;

    noLines++;
  }  // end-add

  void add(LineSegment *ls) {
    // Full array?
    if (noLines == capacity) expandCapacity();

    // Copy
    lines[noLines] = *ls;

    noLines++;
  }  // end-add
};

/// Extract all lines using edge drawing. Invalid lines are filled into
/// invalidLines (if requested)
// Burak - I need both the edge map and the line segments, so I changed the
// following two functions Burak - original function decleration: EDLines
// *DetectLinesByED(unsigned char *srcImg, int width, int height, double
// smoothingSigma=1.0, EDLines *invalidLines=NULL);
// EDLines *DetectLinesByED(EdgeMap *&map, unsigned char *srcImg, int width,
//                          int height, double smoothingSigma = 1.0,
//                          EDLines *invalidLines = NULL);
// Burak - original function decleration: EDLines *DetectLinesByEDPF(unsigned
// char *srcImg, int width, int height, EDLines *invalidLines=NULL);
EDLines *DetectLinesByEDPF(EdgeMap *&map, unsigned char *srcImg, int width,
                           int height, bool flagOnlyUseEdgeSegmentLoops,
                           int thresManhDist);

/// Dump lines to file
// void DumpLines2File(EDLines *lines, char *fname);

#endif