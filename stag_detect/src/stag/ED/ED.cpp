#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

// #include <opencv/cv.h>
// #include <opencv/cxcore.h>
// #include <opencv/highgui.h>

//#include "EDDLL.h"

#include "stag/ED/EDInternals.h"
#include "stag/ED/ED.h"
#include "stag/ED/ImageSmooth.h"
#include "stag/ED/GradientOperators.h"
#include "stag/ED/ValidateEdgeSegments.h"

// #include "stag/ED/Utilities.h"
// #include "Timer.h"

// Burak - won't be needing this
//#include "ImageVideoLib.h"

///----------------------------------------------------------------------------------------------
/// Detect edges by the Edge Drawing method (ED)
///
// EdgeMap *DetectEdgesByED(unsigned char *srcImg, int width, int height,
//                          GradientOperator op, int GRADIENT_THRESH,
//                          int ANCHOR_THRESH, double smoothingSigma) {
//   // Check parameters for sanity
//   if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
//   if (ANCHOR_THRESH < 0) ANCHOR_THRESH = 0;
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

//   // Allocate space for temporary storage
//   unsigned char *smoothImg = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   switch (op) {
//     case PREWITT_OPERATOR:
//       ComputeGradientMapByPrewitt(smoothImg, gradImg, dirImg, width, height,
//                                   GRADIENT_THRESH);
//       break;
//     case SOBEL_OPERATOR:
//       ComputeGradientMapBySobel(smoothImg, gradImg, dirImg, width, height,
//                                 GRADIENT_THRESH);
//       break;
//     case SCHARR_OPERATOR:
//       ComputeGradientMapByScharr(smoothImg, gradImg, dirImg, width, height,
//                                  GRADIENT_THRESH);
//       break;
//   }  // end-switch

// #if 0
//   memset(smoothImg, 0, width*height);
//   for (int i=0; i<width*height; i++){if (gradImg[i]>=GRADIENT_THRESH) smoothImg[i] = 255;}
//   SaveImage("OutputImages/gradImg.pgm", (char *)smoothImg, width, height, 8);
// #endif

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothImg;

//   return map;
// }  // DetectEdgesByED

// ///----------------------------------------------------------------------------------------------
// /// Detect edges by the Edge Drawing method (ED), and validate the resulting
// /// edge segments
// ///
// EdgeMap *DetectEdgesByEDV(unsigned char *srcImg, int width, int height,
//                           GradientOperator op, int GRADIENT_THRESH,
//                           int ANCHOR_THRESH, double smoothingSigma) {
//   // Check parameters for sanity
//   if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
//   if (ANCHOR_THRESH < 0) ANCHOR_THRESH = 0;
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

//   // Allocate space for temporary storage
//   unsigned char *smoothImg = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   switch (op) {
//     case PREWITT_OPERATOR:
//       ComputeGradientMapByPrewitt(smoothImg, gradImg, dirImg, width, height,
//                                   GRADIENT_THRESH);
//       break;
//     case SOBEL_OPERATOR:
//       ComputeGradientMapBySobel(smoothImg, gradImg, dirImg, width, height,
//                                 GRADIENT_THRESH);
//       break;
//     case SCHARR_OPERATOR:
//       ComputeGradientMapByScharr(smoothImg, gradImg, dirImg, width, height,
//                                  GRADIENT_THRESH);
//       break;
//   }  // end-switch

// #if 0
//   memset(smoothImg, 0, width*height);
//   for (int i=0; i<width*height; i++){if (gradImg[i]>=GRADIENT_THRESH) smoothImg[i] = 255;}
//   SaveImage("OutputImages/gradImg.pgm", (char *)smoothImg, width, height, 8);
// #endif

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

//   /*----------- Validate Edge Segments ---------------------*/
// #if 0
//   ValidateEdgeSegments(map, srcImg);
// #elif 1
//   smoothingSigma /= 2.5;
//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);
//   ValidateEdgeSegments(map, smoothImg, 2.25);  // with Prewitt
// //  ValidateEdgeSegments2(map, smoothImg, 2);   // with LSD
// #endif

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothImg;

//   return map;
// }  // DetectEdgesByEDV

///----------------------------------------------------------------------------------------------
/// Detect edges by the Edge Drawing Parameter Free method (EDPF)
///
EdgeMap *DetectEdgesByEDPF(unsigned char *srcImg, int width, int height,
                           double smoothingSigma) {
  // Check parameters for sanity
  if (smoothingSigma < 1.0) smoothingSigma = 1.0;

  // Allocate space for temporary storage
  unsigned char *smoothImg = new unsigned char[width * height];
  unsigned char *dirImg = new unsigned char[width * height];
  short *gradImg = new short[width * height];

  /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
  SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);

  /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
  int GRADIENT_THRESH = 16;
  ComputeGradientMapByPrewitt(smoothImg, gradImg, dirImg, width, height,
                              GRADIENT_THRESH);

#if 0
  DumpGradImage("OutputImages/gradImgBW.pgm", gradImg, width, height, GRADIENT_THRESH);
#endif

  /*-------- Detect the edges by ED --------------------------*/
  int ANCHOR_THRESH = 0;
  EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
                                   GRADIENT_THRESH, ANCHOR_THRESH);

  /*----------- Validate Edge Segments ---------------------*/
#if 0
  ValidateEdgeSegments(map, srcImg);
#elif 1
  smoothingSigma /= 2.5;
  SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);
  ValidateEdgeSegments(map, smoothImg, 2.25);  // with Prewitt
//  ValidateEdgeSegments2(map, smoothImg, 2);   // with LSD
#endif

  // Clean up
  delete gradImg;
  delete dirImg;
  delete smoothImg;

  return map;
}  // DetectEdgesByEDPF

///-------------------------------------------------------------------------------
/// Use cvCanny and return the result as an EdgeMap. smoothingSigma must be
/// >= 1.0
///
// EdgeMap *DetectEdgesByCannySR(unsigned char *srcImg, int width, int height,
//                               int cannyLowThresh, int cannyHighThresh,
//                               int sobelKernelApertureSize,
//                               double smoothingSigma) {
//   IplImage *iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
//   IplImage *edgeImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

//   // Copy the source image to iplImg buffer
//   for (int i = 0; i < height; i++) {
//     unsigned char *p =
//         (unsigned char *)(iplImg->imageData + iplImg->widthStep * i);
//     for (int j = 0; j < width; j++) p[j] = srcImg[i * width + j];
//   }  // end-for

//   if (smoothingSigma <= 1.0)
//     cvSmooth(iplImg, iplImg, CV_GAUSSIAN, 5, 5);
//   else
//     cvSmooth(iplImg, iplImg, CV_GAUSSIAN, 0, 0, smoothingSigma);

//   // Copy the smooth image
//   unsigned char *smoothImg = new unsigned char[width * height];
//   for (int i = 0; i < height; i++) {
//     unsigned char *p =
//         (unsigned char *)(iplImg->imageData + iplImg->widthStep * i);
//     for (int j = 0; j < width; j++) smoothImg[i * width + j] = p[j];
//   }  // end-for

//   // Detect edges by cvCanny
//   if (sobelKernelApertureSize != 3 && sobelKernelApertureSize != 5 &&
//       sobelKernelApertureSize != 7)
//     sobelKernelApertureSize = 3;
//   cvCanny(iplImg, edgeImg, cannyLowThresh, cannyHighThresh,
//           sobelKernelApertureSize);

//   // Create the edge map
//   EdgeMap *map = new EdgeMap(width, height);
//   memset(map->edgeImg, 0, width * height);

//   for (int i = 1; i < height - 1; i++) {
//     unsigned char *p =
//         (unsigned char *)(edgeImg->imageData + edgeImg->widthStep * i);
//     for (int j = 1; j < width - 1; j++)
//       if (p[j]) map->edgeImg[i * width + j] = 254;  // Anchor
//   }                                                 // end-for

//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   cvSmooth(edgeImg, edgeImg, CV_GAUSSIAN, 5, 5);
//   //  cvSmooth(edgeImg, edgeImg, CV_GAUSSIAN, 0, 0, 0.5);

//   // Compute the gradient and edge directions for pixels in the edge areas
//   memset(gradImg, 0, sizeof(short) * width * height);
//   for (int i = 1; i < height - 1; i++) {
//     unsigned char *p =
//         (unsigned char *)(edgeImg->imageData + edgeImg->widthStep * i);
//     for (int j = 1; j < width - 1; j++) {
//       if (p[j] < 32) continue;

//       // Compute the gradient value & edge direction
//       int com1 = smoothImg[(i + 1) * width + j + 1] -
//                  smoothImg[(i - 1) * width + j - 1];
//       int com2 = smoothImg[(i - 1) * width + j + 1] -
//                  smoothImg[(i + 1) * width + j - 1];

//       int gx =
//           abs(com1 + com2 +
//               (smoothImg[i * width + j + 1] - smoothImg[i * width + j - 1]));
//       int gy = abs(
//           com1 - com2 +
//           (smoothImg[(i + 1) * width + j] - smoothImg[(i - 1) * width + j]));
//       int sum = gx + gy;
//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (gx >= gy)
//         dirImg[index] = 1;  // vertical edge
//       else
//         dirImg[index] = 2;  // horizontal edge
//     }                       // end-for
//   }                         // end-for

//   void JoinAnchorPointsUsingSortedAnchors(short *gradImg, unsigned char *dirImg,
//                                           EdgeMap *map, int GRADIENT_THRESH,
//                                           int MIN_PATH_LEN);
//   JoinAnchorPointsUsingSortedAnchors(gradImg, dirImg, map, 1, 10);

//   delete smoothImg;
//   delete gradImg;
//   delete dirImg;

//   cvReleaseImage(&iplImg);
//   cvReleaseImage(&edgeImg);

//   return map;
// }  // end-DetectEdgesByCannySR

// ///-------------------------------------------------------------------------------
// /// Use cvCanny and return the result as an EdgeMap. smoothingSigma must be
// /// >= 1.0. Use ED2 for directionless linking
// ///
// EdgeMap *DetectEdgesByCannySR2(unsigned char *srcImg, int width, int height,
//                                int cannyLowThresh, int cannyHighThresh,
//                                int sobelKernelApertureSize,
//                                double smoothingSigma) {
//   IplImage *iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
//   IplImage *edgeImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

//   // Copy the source image to iplImg buffer
//   for (int i = 0; i < height; i++) {
//     unsigned char *p =
//         (unsigned char *)(iplImg->imageData + iplImg->widthStep * i);
//     for (int j = 0; j < width; j++) p[j] = srcImg[i * width + j];
//   }  // end-for

//   if (smoothingSigma <= 1.0)
//     cvSmooth(iplImg, iplImg, CV_GAUSSIAN, 5, 5);
//   else
//     cvSmooth(iplImg, iplImg, CV_GAUSSIAN, 0, 0, smoothingSigma);

//   if (sobelKernelApertureSize != 3 && sobelKernelApertureSize != 5 &&
//       sobelKernelApertureSize != 7)
//     sobelKernelApertureSize = 3;
//   cvCanny(iplImg, edgeImg, cannyLowThresh, cannyHighThresh,
//           sobelKernelApertureSize);
//   //  cvSaveImage("OutputImages/canny1.pgm", edgeImg);

//   short *gradImg = new short[width * height];
//   for (int i = 0; i < height; i++) {
//     unsigned char *p =
//         (unsigned char *)(edgeImg->imageData + edgeImg->widthStep * i);
//     for (int j = 0; j < width; j++)
//       if (p[j])
//         gradImg[i * width + j] = 255;
//       else
//         gradImg[i * width + j] = 0;
//   }  // end-for

//   //  cvSmooth(edgeImg, edgeImg, CV_GAUSSIAN, 5, 5);
//   cvSmooth(edgeImg, edgeImg, CV_GAUSSIAN, 0, 0, 0.5);
//   //  cvSaveImage("OutputImages/canny2.pgm", edgeImg);

//   for (int i = 0; i < height; i++) {
//     unsigned char *p =
//         (unsigned char *)(edgeImg->imageData + edgeImg->widthStep * i);
//     for (int j = 0; j < width; j++) {
//       if (gradImg[i * width + j]) continue;
//       if (p[j]) gradImg[i * width + j] = p[j];
//     }  // end-for
//   }    // end-for

//   //  DumpGradImage("OutputImages/gradImg.pgm", gradImg, width, height);
//   EdgeMap *map = DoDetectEdgesByED(gradImg, width, height, 4);

//   cvReleaseImage(&iplImg);
//   cvReleaseImage(&edgeImg);

//   delete gradImg;

//   return map;
// }  // end-DetectEdgesByCannySR2

// ///----------------------------------------------------------------------------------------------------------------------------------------
// /// Use DetectEdgesByCannySR and then validate the edge segments afterwards
// ///
// EdgeMap *DetectEdgesByCannySRPF(unsigned char *srcImg, int width, int height,
//                                 int sobelKernelApertureSize,
//                                 double smoothingSigma) {
//   EdgeMap *map = DetectEdgesByCannySR(srcImg, width, height, 20, 20,
//                                       sobelKernelApertureSize, smoothingSigma);

//   unsigned char *smoothImg = new unsigned char[width * height];

//   smoothingSigma /= 2.5;
//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);
//   ValidateEdgeSegments(map, smoothImg, 2.25);  // With Prewitt
//   //  ValidateEdgeSegments2(map, smoothImg, 2);  // With LSD

//   delete smoothImg;

//   return map;
// }  // end-DetectEdgesByCannySRPF

/****************************************** ED FOR COLOR IMAGES
 * **********************************************/
///--------------------------------------------------------------------------------------------------------
/// Given an image, scales its pixels to [0-255]
static void ScaleImage(unsigned char *srcImg, int width, int height) {
  unsigned char min = 255;
  unsigned char max = 0;

  for (int i = 0; i < width * height; i++) {
    if (srcImg[i] < min)
      min = srcImg[i];
    else if (srcImg[i] > max)
      max = srcImg[i];
  }  // end-for

  double scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    srcImg[i] = (unsigned char)((srcImg[i] - min) * scale);
  }
}  // end-ScaleImage

///--------------------------------------------------------------------------------------------------------
/// Detect Edges by Edge Drawing (ED) for color images represented by ch1Img,
/// ch2Img and ch3Img. Uses the multi-image gradient method by DiZenzo for
/// gradient computation smoothingSigma must be >= 1.0
///
// EdgeMap *DetectEdgesByED(unsigned char *ch1Img, unsigned char *ch2Img,
//                          unsigned char *ch3Img, int width, int height,
//                          int GRADIENT_THRESH, int ANCHOR_THRESH,
//                          double smoothingSigma) {
//   // Check parameters for sanity
//   if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
//   if (ANCHOR_THRESH < 0) ANCHOR_THRESH = 0;
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

// #if 1
//   // Scale the images to [0-255]
//   ScaleImage(ch1Img, width, height);
//   ScaleImage(ch2Img, width, height);
//   ScaleImage(ch3Img, width, height);
// #endif

//   // Allocate space for temporary storage
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   // Timer timer;
//   // timer.Start();

//   ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//                               dirImg, width, height, GRADIENT_THRESH);
//   //  ComputeGradientMapByPrewitt5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height, GRADIENT_THRESH);
//   //  ComputeGradientMapByDiZenzo(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height, GRADIENT_THRESH);

//   // timer.Stop();
//   //  printf("\nComputeGradientMapByPrewitt takes %4.2lf ms\n\n",
//   //  timer.ElapsedTime());

// #if 0
//   DumpGradImage("OutputImages/gradBW.pgm", gradImg, width, height, GRADIENT_THRESH);
// #endif

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);
//   //  EdgeMap *map = DoDetectEdgesByED(gradImg, width, height, GRADIENT_THRESH);
//   //  // Fails especially around the corners, where the gradient operator does
//   //  not give good response.

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothCh3Img;
//   delete smoothCh2Img;
//   delete smoothCh1Img;

//   return map;
// }  // end-DetectEdgesByED

// ///--------------------------------------------------------------------------------------------------------------------------------------------
// /// Detect Edges by Edge Drawing Parameter Free (EDPF) for color images
// /// represented by ch1Img, ch2Img and ch3Img. smoothingSigma must be >= 1.0
// ///
// EdgeMap *DetectEdgesByEDPF(unsigned char *ch1Img, unsigned char *ch2Img,
//                            unsigned char *ch3Img, int width, int height,
//                            double smoothingSigma) {
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

// #if 0
//   // Scale the images to [0-255]
//   ScaleImage(ch1Img, width, height);
//   ScaleImage(ch2Img, width, height);
//   ScaleImage(ch3Img, width, height);
// #endif

//   // Allocate space for temporary storage
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   //  int GRADIENT_THRESH = 16;
//   //  GRADIENT_THRESH += (smoothingSigma-1.0)*4;
//   int GRADIENT_THRESH = 16;

//   //  ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height, GRADIENT_THRESH);
//   ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//                               dirImg, width, height);  // 3x3 scaled prewitt
//   //  ComputeGradientMapByPrewitt5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 5x5 scaled prewitt
//   //  ComputeGradientMapByPrewitt7x7(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 7x7 scaled prewitt

//   //  ComputeGradientMapBySobel(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled sobel
//   //  ComputeGradientMapByDiZenzo(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height); // 3x3 scaled prewitt with Di Zenzo
//   //  ComputeGradientMapByEMD(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//   //  dirImg, width, height); ComputeGradientMapByEMD5x5(smoothCh1Img,
//   //  smoothCh2Img, smoothCh3Img, gradImg, dirImg, width, height);

//   //  ComputeGradientMapByCIE94(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);
//   //  ComputeGradientMapByCIEDE2000(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height); //

//   // Now, detect the edges by ED
//   int ANCHOR_THRESH = 0;
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

//   // Validate edge segments
// #if 0
//   ValidateEdgeSegments(map, ch1Img, ch2Img, ch3Img);
// #elif 1
//   // Prewitt operator
//   smoothingSigma /= 2.5;
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   ValidateEdgeSegments(map, smoothCh1Img, smoothCh2Img, smoothCh3Img, 2.25);
// #else
//   // LSD operator
//   smoothingSigma /= 1.667;
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   ValidateEdgeSegments2(map, smoothCh1Img, smoothCh2Img, smoothCh3Img, 2);
// #endif

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothCh3Img;
//   delete smoothCh2Img;
//   delete smoothCh1Img;

//   return map;
// }  // end-DetectEdgesByEDPF

// ///****************************************** CONTOUR DETECTION RELATED
// ///FUNCTIONS *************************************
// ///----------------------------------------------------------------------------------------------
// /// Computation of the edge image over the contour image: Use the contour image
// /// to thin down the edge-ares of the gradient map, and then run ED
// ///
// EdgeMap *DetectContourEdgeMapByED1(unsigned char *srcImg,
//                                    unsigned char *contourImg, int width,
//                                    int height, int CONTOUR_THRESH,
//                                    int GRADIENT_THRESH, int ANCHOR_THRESH) {
//   // Allocate space for temporary storage
//   unsigned char *smoothImg = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   //  SmoothImage(srcImg, smoothImg, width, height, 0.675);
//   SmoothImage(srcImg, smoothImg, width, height, 1.0);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   ComputeGradientMapByPrewitt(smoothImg, gradImg, dirImg, width, height,
//                               GRADIENT_THRESH);

//   // Now, use the contour image, and erase edge areas where thre is no contour
//   unsigned char *smoothContourImg = new unsigned char[width * height];
//   SmoothImage(contourImg, smoothContourImg, width, height, 1.0);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       if (smoothContourImg[i * width + j] >= CONTOUR_THRESH) continue;

//       gradImg[i * width + j] = 0;
//     }  // end-for
//   }    // end-for

//   delete smoothContourImg;

// #if 0
//   memset(smoothImg, 0, width*height);
//   for (int i=0; i<width*height; i++){if (gradImg[i]>=GRADIENT_THRESH) smoothImg[i] = 255;}
//   SaveImage("OutputImages/gradImg.pgm", (char *)smoothImg, width, height, 8);
// #endif

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothImg;

//   return map;
// }  // end-DetectContourEdgeMapByED1

// ///--------------------------------------------------------------------------------------------------------
// /// Computation of the edge image over the contour image: Use the contour image
// /// to thin down the edge-ares of the gradient map, and then run ED
// ///
// EdgeMap *DetectContourEdgeMapByED1(unsigned char *ch1Img, unsigned char *ch2Img,
//                                    unsigned char *ch3Img,
//                                    unsigned char *contourImg, int width,
//                                    int height, int CONTOUR_THRESH,
//                                    int GRADIENT_THRESH, int ANCHOR_THRESH) {
// #if 0
//   // Scale the images to [0-255]
//   ScaleImage(ch1Img, width, height);
//   ScaleImage(ch2Img, width, height);
//   ScaleImage(ch3Img, width, height);
// #endif

//   // Allocate space for temporary storage
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   double smoothingSigma = 1.0;

//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//                               dirImg, width, height, GRADIENT_THRESH);  // orig
//   //  ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled prewitt

//   // Now, use the contour image, and erase edge areas where there is no contour
//   unsigned char *smoothContourImg = new unsigned char[width * height];
//   SmoothImage(contourImg, smoothContourImg, width, height, 1.0);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       if (smoothContourImg[i * width + j] >= CONTOUR_THRESH) continue;

//       gradImg[i * width + j] = 0;
//     }  // end-for
//   }    // end-for

//   delete smoothContourImg;

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothCh3Img;
//   delete smoothCh2Img;
//   delete smoothCh1Img;

//   return map;
// }  // end-DetectContourEdgeMapByED1

// ///----------------------------------------------------------------------------------------------
// /// Run ED over the contour image using contour image as the edge areas: Uses
// /// EDWalk2 for edge linking (no edge directions)
// ///
// EdgeMap *DetectContourEdgeMapByED2(unsigned char *contourImg, int width,
//                                    int height, double smoothingSigma) {
//   short *gradImg = new short[width * height];

//   // Allocate space for temporary storage
//   unsigned char *smoothImg = new unsigned char[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(contourImg, smoothImg, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT -------------------*/
//   memset(gradImg, 0, sizeof(short) * width * height);
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       gradImg[i * width + j] = smoothImg[i * width + j];
//     }  // end-for
//   }    // end-for

//   delete smoothImg;

// #if 0
//   DumpGradImage("OutputImages/gradImg.pgm", gradImg, width, height);
// #endif

//   // Run ED without directions. Makes use of EDWalk2
//   EdgeMap *map = DoDetectEdgesByED(gradImg, width, height, 4);

//   // Clean up
//   delete gradImg;

//   return map;
// }  // end-DetectContourEdgeMapByED2

// ///----------------------------------------------------------------------------------------------
// /// Similar to CannySR with contourImg as the edgemap
// ///
// EdgeMap *DetectContourEdgeMapByED3(unsigned char *srcImg,
//                                    unsigned char *contourImg, int width,
//                                    int height) {
//   EdgeMap *map = new EdgeMap(width, height);

//   memset(map->edgeImg, 0, width * height);
//   for (int i = 0; i < width * height; i++) {
//     if (contourImg[i]) map->edgeImg[i] = 254;
//   }  // set anchors

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   unsigned char *smoothContourImg = new unsigned char[width * height];
//   SmoothImage(contourImg, smoothContourImg, width, height, 1.0);

//   // Compute the gradient and edge directions for pixels in the edge areas
//   unsigned char *smoothImg = new unsigned char[width * height];
//   SmoothImage(srcImg, smoothImg, width, height);

//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   unsigned char *dirImg = new unsigned char[width * height];

//   //  SaveImage("OutputImages/smoothContourImg.pgm", (char *)smoothContourImg,
//   //  width, height, 8); SaveImage("OutputImages/smoothImg.pgm", (char
//   //  *)smoothImg, width, height, 8);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       if (smoothContourImg[i * width + j] < 32) continue;

//       // Compute the gradient value & edge direction
//       int com1 = smoothImg[(i + 1) * width + j + 1] -
//                  smoothImg[(i - 1) * width + j - 1];
//       int com2 = smoothImg[(i - 1) * width + j + 1] -
//                  smoothImg[(i + 1) * width + j - 1];

//       int gx =
//           abs(com1 + com2 +
//               (smoothImg[i * width + j + 1] - smoothImg[i * width + j - 1]));
//       int gy = abs(
//           com1 - com2 +
//           (smoothImg[(i + 1) * width + j] - smoothImg[(i - 1) * width + j]));
//       int sum = gx + gy;
//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (gx >= gy)
//         dirImg[index] = 1;  // vertical edge
//       else
//         dirImg[index] = 2;  // horizontal edge
//     }                       // end-for
//   }                         // end-for

//   //  DumpGradImage("OutputImages/gradImgBW.pgm", gradImg, width, height, 1);

//   void JoinAnchorPointsUsingSortedAnchors(short *gradImg, unsigned char *dirImg,
//                                           EdgeMap *map, int GRADIENT_THRESH,
//                                           int MIN_PATH_LEN);
//   JoinAnchorPointsUsingSortedAnchors(gradImg, dirImg, map, 1, 10);

//   // Clean up
//   delete smoothImg;
//   delete smoothContourImg;
//   delete gradImg;
//   delete dirImg;

//   return map;
// }  // end-DetectContourEdgeMapByED3

// ///----------------------------------------------------------------------------------------------
// /// Run ED over the contour image using contour image as the edge areas: Uses
// /// original ED for edge linking (with edge directions)
// ///
// EdgeMap *DetectContourEdgeMapByED3(unsigned char *ch1Img, unsigned char *ch2Img,
//                                    unsigned char *ch3Img,
//                                    unsigned char *contourImg, int width,
//                                    int height, int GRADIENT_THRESH,
//                                    int ANCHOR_THRESH, double maxSigma) {
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];

//   short *gradImg = new short[width * height];
//   short *gxImg = new short[width * height];
//   short *gyImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   memset(gradImg, 0, sizeof(short) * width * height);
//   memset(gxImg, 0, sizeof(short) * width * height);
//   memset(gyImg, 0, sizeof(short) * width * height);

//   for (double smoothingSigma = 1.0; smoothingSigma <= maxSigma;
//        smoothingSigma += 0.25) {
//     SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//     SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//     SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//     for (int i = 1; i < height - 1; i++) {
//       for (int j = 1; j < width - 1; j++) {
//         // Prewitt for channel1
//         int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                    smoothCh1Img[(i - 1) * width + j - 1];
//         int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                    smoothCh1Img[(i + 1) * width + j - 1];

//         int gxCh1 = abs(com1 + com2 +
//                         (smoothCh1Img[i * width + j + 1] -
//                          smoothCh1Img[i * width + j - 1]));
//         int gyCh1 = abs(com1 - com2 +
//                         (smoothCh1Img[(i + 1) * width + j] -
//                          smoothCh1Img[(i - 1) * width + j]));
//         int gradCh1 = gxCh1 + gyCh1;

//         // Prewitt for channel2
//         com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//                smoothCh2Img[(i - 1) * width + j - 1];
//         com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//                smoothCh2Img[(i + 1) * width + j - 1];

//         int gxCh2 = abs(com1 + com2 +
//                         (smoothCh2Img[i * width + j + 1] -
//                          smoothCh2Img[i * width + j - 1]));
//         int gyCh2 = abs(com1 - com2 +
//                         (smoothCh2Img[(i + 1) * width + j] -
//                          smoothCh2Img[(i - 1) * width + j]));
//         int gradCh2 = gxCh2 + gyCh2;

//         // Prewitt for channel3
//         com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//                smoothCh3Img[(i - 1) * width + j - 1];
//         com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//                smoothCh3Img[(i + 1) * width + j - 1];

//         int gxCh3 = abs(com1 + com2 +
//                         (smoothCh3Img[i * width + j + 1] -
//                          smoothCh3Img[i * width + j - 1]));
//         int gyCh3 = abs(com1 - com2 +
//                         (smoothCh3Img[(i + 1) * width + j] -
//                          smoothCh3Img[(i - 1) * width + j]));
//         int gradCh3 = gxCh3 + gyCh3;

//         // Combine the gradients
//         int gx, gy, grad;

//         // Sum
//         gx = gxCh1 + gxCh2 + gxCh3;
//         gy = gyCh1 + gyCh2 + gyCh3;
//         grad = gradCh1 + gradCh2 + gradCh3;

//         int index = i * width + j;

//         gxImg[index] += gx;
//         gyImg[index] += gy;
//         gradImg[index] += grad;
//       }  // end-for
//     }    // end-for
//   }      // end-for

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   unsigned char *smoothImg = smoothCh1Img;
//   SmoothImage(contourImg, smoothImg, width, height, 1.0);

//   int max = 0;
//   for (int i = 0; i < width * height; i++) {
//     if (gradImg[i] > max) max = gradImg[i];
//   }
//   double scale = 255.0 / max;

// #if 0
//   for (int i=0; i<width*height; i++){
//     if (smoothImg[i] <= 1) gradImg[i] = 0;
//     else gradImg[i] = (short)(scale*gradImg[i]);
//   } //end-for
//   GRADIENT_THRESH = 1;
// #else
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(scale * gradImg[i]);
// #endif

// #if 0
//   DumpGradImage("OutputImages/gradImg.pgm", gradImg, width, height);
//   DumpGradImage("OutputImages/gradImgBW.pgm", gradImg, width, height, GRADIENT_THRESH);
// #endif

//   // Compute the edge directions
//   unsigned char *dirImg = new unsigned char[width * height];

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       if (gxImg[i * width + j] > gyImg[i * width + j])
//         dirImg[i * width + j] = 1;  // vertical
//       else
//         dirImg[i * width + j] = 2;  // horizontal
//     }                               // end-for
//   }                                 // end-for

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

// #if 0
//   // Validate
//   memset(gradImg, 0, sizeof(short)*width*height);      

//   for (double smoothingSigma = 1.0; smoothingSigma <= maxSigma; smoothingSigma += 0.25){  
//     SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma/2.5);
//     SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma/2.5);
//     SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma/2.5);

//     for (int i=1; i<height-1; i++){
//       for (int j=1; j<width-1; j++){
//         // Prewitt for channel1
//         int com1 = smoothCh1Img[(i+1)*width+j+1] - smoothCh1Img[(i-1)*width+j-1];
//         int com2 = smoothCh1Img[(i-1)*width+j+1] - smoothCh1Img[(i+1)*width+j-1];

//         int gxCh1 = abs(com1 + com2 + (smoothCh1Img[i*width+j+1] - smoothCh1Img[i*width+j-1]));
//         int gyCh1 = abs(com1 - com2 + (smoothCh1Img[(i+1)*width+j] - smoothCh1Img[(i-1)*width+j]));
//         int gradCh1 = gxCh1+gyCh1;

//         // Prewitt for channel2
//         com1 = smoothCh2Img[(i+1)*width+j+1] - smoothCh2Img[(i-1)*width+j-1];
//         com2 = smoothCh2Img[(i-1)*width+j+1] - smoothCh2Img[(i+1)*width+j-1];

//         int gxCh2 = abs(com1 + com2 + (smoothCh2Img[i*width+j+1] - smoothCh2Img[i*width+j-1]));
//         int gyCh2 = abs(com1 - com2 + (smoothCh2Img[(i+1)*width+j] - smoothCh2Img[(i-1)*width+j]));
//         int gradCh2 = gxCh2+gyCh2;

//         // Prewitt for channel3
//         com1 = smoothCh3Img[(i+1)*width+j+1] - smoothCh3Img[(i-1)*width+j-1];
//         com2 = smoothCh3Img[(i-1)*width+j+1] - smoothCh3Img[(i+1)*width+j-1];

//         int gxCh3 = abs(com1 + com2 + (smoothCh3Img[i*width+j+1] - smoothCh3Img[i*width+j-1]));
//         int gyCh3 = abs(com1 - com2 + (smoothCh3Img[(i+1)*width+j] - smoothCh3Img[(i-1)*width+j]));
//         int gradCh3 = gxCh3+gyCh3;

//         // Sum
//         int grad = gradCh1 + gradCh2 + gradCh3;

//         int index = i*width+j;
//         gradImg[index] += grad;
//       } //end-for
//     } // end-for
//   } //end-for

//   ValidateEdgeSegmentsRuzon(map, gradImg, 3);
// #endif

//   // Clean up
//   delete gradImg;
//   delete gxImg;
//   delete gyImg;
//   delete dirImg;
//   delete smoothCh1Img;
//   delete smoothCh2Img;
//   delete smoothCh3Img;

//   return map;
// }  // end-DetectContourEdgeMapByED3

// ///----------------------------------------------------------------------------------------------
// /// Detect edges by ED. Changes the size of the prewitt kernel depending on
// /// "smoothingSigma" parameter Also adjusts the value of GRADIENT_THRESH based
// /// on the kernel size Used for grayscale images
// ///
// EdgeMap *DetectEdgesByED10(unsigned char *srcImg, int width, int height,
//                            int GRADIENT_THRESH, int ANCHOR_THRESH,
//                            double smoothingSigma) {
//   // Check parameters for sanity
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

//   // Allocate space for temporary storage
//   unsigned char *smoothImg = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(srcImg, smoothImg, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   ComputeGradientMapByPrewitt(smoothImg, gradImg, dirImg, width, height,
//                               GRADIENT_THRESH);

// #if 0
//   char fname[100];
//   sprintf(fname, "OutputImages/gradImg%4.2lf.pgm", smoothingSigma);
//   memset(smoothImg, 0, width*height);
//   for (int i=0; i<width*height; i++){if (gradImg[i]>=GRADIENT_THRESH) smoothImg[i] = 255;}
//   SaveImage(fname, (char *)smoothImg, width, height, 8);

//   sprintf(fname, "OutputImages/gradImgGS%4.2lf.pgm", smoothingSigma);
//   DumpGradImage(fname, gradImg, width, height);
// #endif

//   // Now, detect the edges by ED
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

// #if 0
//   sprintf(fname, "OutputImages/ED-%4.2lf.pgm", smoothingSigma);
//   map->ConvertEdgeSegments2EdgeImg();
//   SaveImage(fname, (char *)map->edgeImg, width, height, 8);
// #endif

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothImg;

//   return map;
// }  // DetectEdgesByED10

// ///--------------------------------------------------------------------------------------------------------
// /// Detect edges by ED. Changes the size of the prewitt kernel depending on
// /// "smoothingSigma" parameter Also adjusts the value of GRADIENT_THRESH based
// /// on the kernel size Used for color images
// ///
// EdgeMap *DetectEdgesByED10(unsigned char *ch1Img, unsigned char *ch2Img,
//                            unsigned char *ch3Img, int width, int height,
//                            int GRADIENT_THRESH, int ANCHOR_THRESH,
//                            double smoothingSigma) {
//   // Check parameters for sanity
//   if (smoothingSigma < 1.0) smoothingSigma = 1.0;

// #if 0
//   // Scale the images to [0-255]
//   ScaleImage(ch1Img, width, height);
//   ScaleImage(ch2Img, width, height);
//   ScaleImage(ch3Img, width, height);
// #endif

//   // Allocate space for temporary storage
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   //  GRADIENT_THRESH += (smoothingSigma-1.0)*2;

// #if 1
//   // Compute the gradient map & edge directions
//   //  ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled Prewitt
//   //  ComputeGradientMapByPrewitt5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 5x5 scaled Prewitt
//   //  ComputeGradientMapByPrewitt7x7(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 7x7 scaled Prewitt
//   //  ComputeGradientMapBySobel(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled Sobel
//   ComputeGradientMapByDiZenzo(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//                               dirImg, width, height);
//   //  ComputeGradientMapByCIE94(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);
//   //  ComputeGradientMapByCIEDE2000(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height); ComputeGradientMapByEMD(smoothCh1Img,
//   //  smoothCh2Img, smoothCh3Img, gradImg, dirImg, width, height);
//   //  ComputeGradientMapByEMD5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);

//   // Detect edges by ED: 2 edge directions only
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

// #else
//   // Compute the gradient map & edge directions
//   ComputeGradientMapByDiZenzo4Dirs(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//                                    gradImg, dirImg, width, height);

//   // Detect edges by ED: 4 edge directions
//   EdgeMap *map = DoDetectEdgesByED4Dirs(gradImg, dirImg, width, height,
//                                         GRADIENT_THRESH, ANCHOR_THRESH);
// #endif

// #if 0
//   char fname[100];
//   sprintf(fname, "OutputImages/gradImgBW-%4.2lf.pgm", smoothingSigma);
//   DumpGradImage(fname, gradImg, width, height, GRADIENT_THRESH);

//   sprintf(fname, "OutputImages/gradImg-%4.2lf.pgm", smoothingSigma);
//   DumpGradImage(fname, gradImg, width, height);
// #endif

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothCh3Img;
//   delete smoothCh2Img;
//   delete smoothCh1Img;

//   return map;
// }  // end-DetectEdgesByED10

// ///--------------------------------------------------------------------------------------------------------
// /// Detect edges by ED. Changes the size of the prewitt kernel depending on
// /// "smoothingSigma" parameter Also adjusts the value of GRADIENT_THRESH based
// /// on the kernel size Used for color images
// ///
// EdgeMap *DetectEdgesByED10V(unsigned char *ch1Img, unsigned char *ch2Img,
//                             unsigned char *ch3Img, int width, int height,
//                             int GRADIENT_THRESH, int ANCHOR_THRESH,
//                             double smoothingSigma) {
//   // Check parameters for sanity
//   //  if (smoothingSigma < 1.0) smoothingSigma = 1.0;

// #if 0
//   // Scale the images to [0-255]
//   ScaleImage(ch1Img, width, height);
//   ScaleImage(ch2Img, width, height);
//   ScaleImage(ch3Img, width, height);
// #endif

//   // Allocate space for temporary storage
//   unsigned char *smoothCh1Img = new unsigned char[width * height];
//   unsigned char *smoothCh2Img = new unsigned char[width * height];
//   unsigned char *smoothCh3Img = new unsigned char[width * height];
//   unsigned char *dirImg = new unsigned char[width * height];
//   short *gradImg = new short[width * height];

//   /*------------ SMOOTH THE IMAGE BY A GAUSSIAN KERNEL -------------------*/
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   /*------------ COMPUTE GRADIENT & EDGE DIRECTION MAPS -------------------*/
//   //  GRADIENT_THRESH += (smoothingSigma-1.0)*2;

// #if 1
//   // Compute the gradient map & edge directions
//   //  ComputeGradientMapByPrewitt(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled Prewitt
//   //  ComputeGradientMapByPrewitt5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 5x5 scaled Prewitt
//   //  ComputeGradientMapByPrewitt7x7(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 7x7 scaled Prewitt
//   //  ComputeGradientMapBySobel(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);  // 3x3 scaled Sobel
//   ComputeGradientMapByDiZenzo(smoothCh1Img, smoothCh2Img, smoothCh3Img, gradImg,
//                               dirImg, width, height);
//   //  ComputeGradientMapByCIE94(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);
//   //  ComputeGradientMapByCIEDE2000(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height); ComputeGradientMapByEMD(smoothCh1Img,
//   //  smoothCh2Img, smoothCh3Img, gradImg, dirImg, width, height);
//   //  ComputeGradientMapByEMD5x5(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//   //  gradImg, dirImg, width, height);

//   // Detect edges by ED: 2 edge directions only
//   EdgeMap *map = DoDetectEdgesByED(gradImg, dirImg, width, height,
//                                    GRADIENT_THRESH, ANCHOR_THRESH);

// #else
//   // Compute the gradient map & edge directions
//   ComputeGradientMapByDiZenzo4Dirs(smoothCh1Img, smoothCh2Img, smoothCh3Img,
//                                    gradImg, dirImg, width, height);

//   // Detect edges by ED: 4 edge directions
//   EdgeMap *map = DoDetectEdgesByED4Dirs(gradImg, dirImg, width, height,
//                                         GRADIENT_THRESH, ANCHOR_THRESH);
// #endif

// #if 0
//   char fname[100];
//   sprintf(fname, "OutputImages/gradImgBW-%4.2lf.pgm", smoothingSigma);
//   DumpGradImage(fname, gradImg, width, height, GRADIENT_THRESH);

//   sprintf(fname, "OutputImages/gradImg-%4.2lf.pgm", smoothingSigma);
//   DumpGradImage(fname, gradImg, width, height);
// #endif

//   // Validate edge segments
// #if 0
//   ValidateEdgeSegments(map, ch1Img, ch2Img, ch3Img);
// #elif 1
//   // Prewitt operator
//   smoothingSigma /= 2.5;
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   ValidateEdgeSegments(map, smoothCh1Img, smoothCh2Img, smoothCh3Img, 2.25);
// #else
//   // LSD operator
//   smoothingSigma /= 1.667;
//   SmoothImage(ch1Img, smoothCh1Img, width, height, smoothingSigma);
//   SmoothImage(ch2Img, smoothCh2Img, width, height, smoothingSigma);
//   SmoothImage(ch3Img, smoothCh3Img, width, height, smoothingSigma);

//   ValidateEdgeSegments2(map, smoothCh1Img, smoothCh2Img, smoothCh3Img, 2);
// #endif

//   // Clean up
//   delete gradImg;
//   delete dirImg;
//   delete smoothCh3Img;
//   delete smoothCh2Img;
//   delete smoothCh1Img;

//   return map;
// }  // end-DetectEdgesByED10V
