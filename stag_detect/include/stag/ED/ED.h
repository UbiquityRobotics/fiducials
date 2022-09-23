#ifndef ED_H
#define ED_H

#include "stag/ED/EdgeMap.h"

// /// Detect Edges by Edge Drawing (ED). smoothingSigma must be >= 1.0
// EdgeMap *DetectEdgesByED(unsigned char *srcImg, int width, int height,
//                          GradientOperator op = PREWITT_OPERATOR,
//                          int GRADIENT_THRESH = 20, int ANCHOR_THRESH = 0,
//                          double smoothingSigma = 1.0);

// /// Detect Edges by Edge Drawing (ED) and validate the resulting edge segments.
// /// smoothingSigma must be >= 1.0
// EdgeMap *DetectEdgesByEDV(unsigned char *srcImg, int width, int height,
//                           GradientOperator op = PREWITT_OPERATOR,
//                           int GRADIENT_THRESH = 20, int ANCHOR_THRESH = 0,
//                           double smoothingSigma = 1.0);

/// Detect Edges by Edge Drawing Parameter Free (EDPF). smoothingSigma must be
/// >= 1.0
EdgeMap *DetectEdgesByEDPF(unsigned char *srcImg, int width, int height,
                           double smoothingSigma = 1.0);

/// Use cvCanny and return the result as an EdgeMap. smoothingSigma must be
/// >= 1.0
// EdgeMap *DetectEdgesByCannySR(unsigned char *srcImg, int width, int height,
//                               int cannyLowThresh, int cannyHighThresh,
//                               int sobelKernelApertureSize = 3,
//                               double smoothingSigma = 1.0);

// /// Use cvCanny and return the result as an EdgeMap. smoothingSigma must be
// /// >= 1.0. Use ED2 for linking
// EdgeMap *CannyEDetectEdgesByCannyS2(unsigned char *srcImg, int width,
//                                     int height, int cannyLowThresh,
//                                     int cannyHighThresh,
//                                     int sobelKernelApertureSize = 3,
//                                     double smoothingSigma = 1.0);

// /// Use cvCanny and return the result as an EdgeMap. smoothingSigma must be
// /// >= 1.0. Validate the edge segments after detection
// EdgeMap *DetectEdgesByCannySRPF(unsigned char *srcImg, int width, int height,
//                                 int sobelKernelApertureSize = 3,
//                                 double smoothingSigma = 1.0);

// ///-------------------------- Color images below
// ///------------------------------------------------
// /// Detect Edges by Edge Drawing (ED) for color images represented by ch1Img,
// /// ch2Img and ch3Img. Uses the multi-image gradient method by DiZenzo for
// /// gradient computation smoothingSigma must be >= 1.0
// EdgeMap *DetectEdgesByED(unsigned char *ch1Img, unsigned char *ch2Img,
//                          unsigned char *ch3Img, int width, int height,
//                          int GRADIENT_THRESH = 20, int ANCHOR_THRESH = 0,
//                          double smoothingSigma = 1.0);

// /// Detect Edges by Edge Drawing Parameter Free (EDPF) for color images
// /// represented by ch1Img, ch2Img and ch3Img. smoothingSigma must be >= 1.0
// EdgeMap *DetectEdgesByEDPF(unsigned char *ch1Img, unsigned char *ch2Img,
//                            unsigned char *ch3Img, int width, int height,
//                            double smoothingSigma = 1.0);

// ///--------------------------- Contour Detection specific functions below
// ///-------------------------
// /// Run ED over srcImg, but use contourImg to determine the boundaries of the
// /// edge areas: Used for grayscale images
// EdgeMap *DetectContourEdgeMapByED1(unsigned char *srcImg,
//                                    unsigned char *contourImg, int width,
//                                    int height, int CONTOUR_THRESH,
//                                    int GRADIENT_THRESH, int ANCHOR_THRESH);

// /// Run ED over "ch1Img+ch2Img+ch3Img", but use contourImg to determine the
// /// boundaries of the edge areas: Used for color images
// EdgeMap *DetectContourEdgeMapByED1(unsigned char *ch1Img, unsigned char *ch2Img,
//                                    unsigned char *ch3Img,
//                                    unsigned char *contourImg, int width,
//                                    int height, int CONTOUR_THRESH,
//                                    int GRADIENT_THRESH, int ANCHOR_THRESH);

// /// Run ED over the contour image using contour image as the edge areas: Uses
// /// EDWalk2 for edge linking (no edge directions)
// EdgeMap *DetectContourEdgeMapByED2(unsigned char *contourImg, int width,
//                                    int height, double smoothingSigma = 0.675);

// /// Deneme
// EdgeMap *DetectContourEdgeMapByED3(unsigned char *srcImg,
//                                    unsigned char *contourImg, int width,
//                                    int height);

// /// Multi-scale gradient computation & edge drawing
// EdgeMap *DetectContourEdgeMapByED3(unsigned char *ch1Img, unsigned char *ch2Img,
//                                    unsigned char *ch3Img,
//                                    unsigned char *contourImg, int width,
//                                    int height, int GRADIENT_THRESH,
//                                    int ANCHOR_THRESH, double maxSigma = 2.75);

// /// Detects edges by changing the size of the gradient operator depending on
// /// "smoothingSigma" parameter and sets GRADIENT_THRESH accordingly
// EdgeMap *DetectEdgesByED10(unsigned char *srcImg, int width, int height,
//                            double smoothingSigma);

// ///------------------------------ Deneme stuff
// ///---------------------------------------------------------------------------------------------
// /// Detects edges by changing the size of the gradient operator depending on
// /// "smoothingSigma" parameter and sets GRADIENT_THRESH accordingly
// EdgeMap *DetectEdgesByED10(unsigned char *srcImg, int width, int height,
//                            int GRADIENT_THRESH, int ANCHOR_THRESH,
//                            double smoothingSigma);
// EdgeMap *DetectEdgesByED11(unsigned char *srcImg, int width, int height,
//                            double smoothingSigma);

// EdgeMap *DetectEdgesByED10(unsigned char *ch1Img, unsigned char *ch2Img,
//                            unsigned char *ch3Img, int width, int height,
//                            int GRADIENT_THRESH, int ANCHOR_THRESH,
//                            double smoothingSigma);
// EdgeMap *DetectEdgesByED10V(unsigned char *ch1Img, unsigned char *ch2Img,
//                             unsigned char *ch3Img, int width, int height,
//                             int GRADIENT_THRESH, int ANCHOR_THRESH,
//                             double smoothingSigma);

// EdgeMap *DetectEdgesByED11(unsigned char *ch1Img, unsigned char *ch2Img,
//                            unsigned char *ch3Img, int width, int height,
//                            double smoothingSigma);

#endif