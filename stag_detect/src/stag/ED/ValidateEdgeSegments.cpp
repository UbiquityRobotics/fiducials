#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>

#include "stag/ED/ValidateEdgeSegments.h"

#define MAX_GRAD_VALUE 128 * 256

///---------------------------------------------------------------------------
/// LSD gradient map computation during segment validation
///
// static short *ComputeLSD(unsigned char *srcImg, int width, int height,
//                          double *H) {
//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // LSD gradient computation
//       // A B
//       // C D
//       // gx = (B-A) + (D-C)
//       // gy = (C-A) + (D-B)
//       //
//       // To make this faster:
//       // com1 = (D-A)
//       // com2 = (B-C)
//       // Then: gx = com1 + com2 = (D-A) + (B-C) = (B-A) + (D-C)
//       //       gy = com1 - com2 = (D-A) - (B-C) = (C-A) + (D-B)
//       //
//       int com1 = srcImg[(i + 1) * width + j + 1] - srcImg[i * width + j];
//       int com2 = srcImg[i * width + j + 1] - srcImg[(i + 1) * width + j];

//       int gx = abs(com1 + com2);
//       int gy = abs(com1 - com2);

//       int g = gx + gy;

//       gradImg[i * width + j] = g;
//       grads[g]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2) * (height - 2);
//   //  size -= grads[0];
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;
//   return gradImg;
// }  // end-ComputeLSD

///---------------------------------------------------------------------------
/// Prewitt gradient map computation during segment validation
///
static short *ComputePrewitt3x3(unsigned char *srcImg, int width, int height,
                                double *H) {
  short *gradImg = new short[width * height];
  memset(gradImg, 0, sizeof(short) * width * height);

  int maxGradValue = MAX_GRAD_VALUE;
  int *grads = new int[maxGradValue];
  memset(grads, 0, sizeof(int) * maxGradValue);

  for (int i = 1; i < height - 1; i++) {
    for (int j = 1; j < width - 1; j++) {
      // Prewitt Operator in horizontal and vertical direction
      // A B C
      // D x E
      // F G H
      // gx = (C-A) + (E-D) + (H-F)
      // gy = (F-A) + (G-B) + (H-C)
      //
      // To make this faster:
      // com1 = (H-A)
      // com2 = (C-F)
      // Then: gx = com1 + com2 + (E-D) = (H-A) + (C-F) + (E-D) = (C-A) + (E-D)
      // + (H-F)
      //       gy = com1 - com2 + (G-B) = (H-A) - (C-F) + (G-B) = (F-A) + (G-B)
      //       + (H-C)
      //
      int com1 =
          srcImg[(i + 1) * width + j + 1] - srcImg[(i - 1) * width + j - 1];
      int com2 =
          srcImg[(i - 1) * width + j + 1] - srcImg[(i + 1) * width + j - 1];

      int gx = abs(com1 + com2 +
                   (srcImg[i * width + j + 1] - srcImg[i * width + j - 1]));
      int gy = abs(com1 - com2 +
                   (srcImg[(i + 1) * width + j] - srcImg[(i - 1) * width + j]));

      int g = gx + gy;

      gradImg[i * width + j] = g;
      grads[g]++;
    }  // end-for
  }    // end-for

  // Compute probability function H
  int size = (width - 2) * (height - 2);
  //  size -= grads[0];
  for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
  for (int i = 0; i < maxGradValue; i++)
    H[i] = (double)grads[i] / ((double)size);

  delete grads;
  return gradImg;
}  // end-ComputePrewitt3x3

///---------------------------------------------------------------------------
/// Number of false alarms code as suggested by Desolneux, Moisan and Morel
/// (DMM)
///
#define EPSILON 1.0
static double NFA(int np, double prob, int len) {
  double nfa = np;
  for (int i = 0; i < len && nfa > EPSILON; i++) nfa *= prob;
  //  for (int i=0; i<len; i++) nfa*=prob;

  return nfa;
}  // end-NFA

///----------------------------------------------------------------------------------
/// Resursive validation using half of the pixels as suggested by DMM algorithm
/// We take pixels at Nyquist distance, i.e., 2 (as suggested by DMM)
///
static void TestSegment(EdgeMap *map, short *gradImg, int i, int index1,
                        int index2, int np, double *H, double div) {
  unsigned char *edgeImg = map->edgeImg;

#define MIN_PATH_LEN 10
  int width = map->width;

  int chainLen = index2 - index1 + 1;
  if (chainLen < MIN_PATH_LEN) return;

  /// Test from index1 to index2. If OK, then we are done. Otherwise, split into
  /// two and recursively test the left & right halves

  // First find the min. gradient along the segment
  int minGrad = 1 << 30;
  int minGradIndex;
  for (int k = index1; k <= index2; k++) {
    int r = map->segments[i].pixels[k].r;
    int c = map->segments[i].pixels[k].c;
    if (gradImg[r * width + c] < minGrad) {
      minGrad = gradImg[r * width + c];
      minGradIndex = k;
    }
  }  // end-for

  // Compute nfa
  double nfa = NFA(np, H[minGrad], (int)(chainLen / div));

  if (nfa <= EPSILON) {
    for (int k = index1; k <= index2; k++) {
      int r = map->segments[i].pixels[k].r;
      int c = map->segments[i].pixels[k].c;

      edgeImg[r * width + c] = 255;
    }  // end-for

    return;
  }  // end-if

  // Split into two halves. We divide at the point where the gradient is the
  // minimum
  int end = minGradIndex - 1;
  while (end > index1) {
    int r = map->segments[i].pixels[end].r;
    int c = map->segments[i].pixels[end].c;

    if (gradImg[r * width + c] <= minGrad)
      end--;
    else
      break;
  }  // end-while

  int start = minGradIndex + 1;
  while (start < index2) {
    int r = map->segments[i].pixels[start].r;
    int c = map->segments[i].pixels[start].c;

    if (gradImg[r * width + c] <= minGrad)
      start++;
    else
      break;
  }  // end-while

  TestSegment(map, gradImg, i, index1, end, np, H, div);
  TestSegment(map, gradImg, i, start, index2, np, H, div);
}  // end-TestSegment

///----------------------------------------------------------------------------------
/// Resursive validation using half of the pixels as suggested by DMM algorithm
/// We take pixels at Nyquist distance, i.e., 2 (as suggested by DMM)
///
// static void TestSegment2(EdgeMap *map, short *gradImg, int i, int index1,
//                          int index2, int np, double *H, double div) {
//   unsigned char *edgeImg = map->edgeImg;

// #define MIN_PATH_LEN 10
//   int width = map->width;

//   int chainLen = index2 - index1 + 1;
//   if (chainLen < MIN_PATH_LEN) return;

//     /// Test from index1 to index2. If OK, then we are done. Otherwise, split
//     /// into two and recursively test the left & right halves

// #if 0
//   // Find the min. gradient along the segment & also compute the number of pixels at Nyquist distance from each other
//   chainLen = 1;
//   int index = index1;
//   int minGrad = 1<<30;
//   int minGradIndex;
//   for (int k=index1; k<=index2; k++){
//     int r = map->segments[i].pixels[k].r;
//     int c = map->segments[i].pixels[k].c;
//     if (gradImg[r*width+c] < minGrad){minGrad = gradImg[r*width+c]; minGradIndex = k;}

//     // Count the number of pixels at Nyquist distance from each other
//     int dx = abs(map->segments[i].pixels[index].c - map->segments[i].pixels[k].c);
//     int dy = abs(map->segments[i].pixels[index].r - map->segments[i].pixels[k].r);    
//     if (dx >= 2 || dy >= 2){
//       index = k;
//       chainLen++;
//     } // end-if
//   } //end-for
// #else
//   // Find the min. gradient along the segment & also compute the number of
//   // pixels at Nyquist distance from each other
//   int minGrad = 1 << 30;
//   int minGradIndex;

//   int count1 = 0;
//   int count2 = 0;

//   for (int k = index1; k <= index2; k++) {
//     int r = map->segments[i].pixels[k].r;
//     int c = map->segments[i].pixels[k].c;
//     if (gradImg[r * width + c] < minGrad) {
//       minGrad = gradImg[r * width + c];
//       minGradIndex = k;
//     }

//     // Count the number of pixels at Nyquist distance from each other
//     if (k == index1) {
//       count1++;
//       continue;
//     }
//     int dx =
//         abs(map->segments[i].pixels[k].c - map->segments[i].pixels[k - 1].c);
//     int dy =
//         abs(map->segments[i].pixels[k].r - map->segments[i].pixels[k - 1].r);
//     if (dx == 1 && dy == 1)
//       count2++;  // diagonal
//     else
//       count1++;  // one pixel left/right/up/down
//   }              // end-for

//   chainLen = (int)(count1 + 1.414 * count2);
// //  chainLen = (int)(count1 + 2*count2);
// #endif

//   // Compute nfa
//   double nfa = NFA(np, H[minGrad], (int)(chainLen / div));

//   if (nfa <= EPSILON) {
//     for (int k = index1; k <= index2; k++) {
//       int r = map->segments[i].pixels[k].r;
//       int c = map->segments[i].pixels[k].c;

//       edgeImg[r * width + c] = 255;
//     }  // end-for

//     return;
//   }  // end-if

//   // Split into two halves. We divide at the point where the gradient is the
//   // minimum
//   int end = minGradIndex - 1;
//   while (end > index1) {
//     int r = map->segments[i].pixels[end].r;
//     int c = map->segments[i].pixels[end].c;

//     if (gradImg[r * width + c] <= minGrad)
//       end--;
//     else
//       break;
//   }  // end-while

//   int start = minGradIndex + 1;
//   while (start < index2) {
//     int r = map->segments[i].pixels[start].r;
//     int c = map->segments[i].pixels[start].c;

//     if (gradImg[r * width + c] <= minGrad)
//       start++;
//     else
//       break;
//   }  // end-while

//   TestSegment2(map, gradImg, i, index1, end, np, H, div);
//   TestSegment2(map, gradImg, i, start, index2, np, H, div);
// }  // end-TestSegment2

///----------------------------------------------------------------------------------------------
/// After the validation of the edge segments, extracts the valid ones
/// In other words, updates the valid segments' pixel arrays and their lengths
///
void ExtractNewSegments(EdgeMap *map) {
  unsigned char *edgeImg = map->edgeImg;
  int width = map->width;
  EdgeSegment *segments = &map->segments[map->noSegments];
  int noSegments = 0;

  for (int i = 0; i < map->noSegments; i++) {
    int start = 0;
    while (start < map->segments[i].noPixels) {
      while (start < map->segments[i].noPixels) {
        int r = map->segments[i].pixels[start].r;
        int c = map->segments[i].pixels[start].c;

        if (edgeImg[r * width + c]) break;
        start++;
      }  // end-while

      int end = start + 1;
      while (end < map->segments[i].noPixels) {
        int r = map->segments[i].pixels[end].r;
        int c = map->segments[i].pixels[end].c;

        if (edgeImg[r * width + c] == 0) break;
        end++;
      }  // end-while

      int len = end - start;
      if (len >= 10) {
        // A new segment. Accepted only only long enough (whatever that means)
        segments[noSegments].pixels = &map->segments[i].pixels[start];
        segments[noSegments].noPixels = len;
        noSegments++;
      }  // end-else

      start = end + 1;
    }  // end-while
  }    // end-for

  // Copy to ed
  for (int i = 0; i < noSegments; i++) map->segments[i] = segments[i];
  map->noSegments = noSegments;
}  // end-ExtractNewSegments

///----------------------------------------------------------------------------------
/// Validate the edge segments using the Helmholtz principle
///
void ValidateEdgeSegments(EdgeMap *map, unsigned char *srcImg,
                          double divForTestSegment) {
  int width = map->width;
  int height = map->height;

  memset(map->edgeImg, 0, width * height);

  int maxGradValue = MAX_GRAD_VALUE;
  double *H = new double[maxGradValue];
  memset(H, 0, sizeof(double) * maxGradValue);

  //  short *gradImg = ComputeLSD(srcImg, width, height, H);
  short *gradImg = ComputePrewitt3x3(srcImg, width, height, H);

  // Compute np: # of segment pieces
#if 1
  // Does this underestimate the number of pieces of edge segments?
  // What's the correct value?
  int np = 0;
  for (int i = 0; i < map->noSegments; i++) {
    int len = map->segments[i].noPixels;
    np += (len * (len - 1)) / 2;
  }  // end-for

//  np *= 32;
#elif 0
  // This definitely overestimates the number of pieces of edge segments
  int np = 0;
  for (int i = 0; i < map->noSegments; i++) {
    np += map->segments[i].noPixels;
  }  // end-for

  np = (np * (np - 1)) / 2;
#endif

  // Validate segments
  for (int i = 0; i < map->noSegments; i++) {
    TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
                divForTestSegment);
    //    TestSegment2(map, gradImg, i, 0, map->segments[i].noPixels-1, np, H,
    //    divForTestSegment);
  }  // end-for

  /// Extract the new edge segments after validation
  ExtractNewSegments(map);

  delete H;
  delete gradImg;
}  // end-ValidateEdgeSegments

///----------------------------------------------------------------------------------
/// ValidateEdgeSegments2: Uses LSD operator for gradient computation
///
// void ValidateEdgeSegments2(EdgeMap *map, unsigned char *srcImg,
//                            double divForTestSegment) {
//   int width = map->width;
//   int height = map->height;

//   memset(map->edgeImg, 0, width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
// #if 0
//       int gx = abs((srcImg[i*width+j+1] - srcImg[i*width+j]) + (srcImg[(i+1)*width+j+1] - srcImg[(i+1)*width+j]));
//       int gy = abs((srcImg[(i+1)*width+j] - srcImg[i*width+j]) + (srcImg[(i+1)*width+j+1] - srcImg[i*width+j+1]));
//       int g = gx + gy;
// #elif 1
//       // Prewitt
//       // 0 degrees
//       int g = 0, G;

//       G = abs(srcImg[(i - 1) * width + j - 1] + srcImg[(i - 1) * width + j] +
//               srcImg[(i - 1) * width + j + 1] -
//               srcImg[(i + 1) * width + j - 1] - srcImg[(i + 1) * width + j] -
//               srcImg[(i + 1) * width + j + 1]);
//       if (G > g) g = G;

//       // 45 degrees
//       G = abs(srcImg[i * width + j + 1] + srcImg[(i + 1) * width + j + 1] +
//               srcImg[(i + 1) * width + j] - srcImg[i * width + j - 1] -
//               srcImg[(i - 1) * width + j - 1] - srcImg[(i - 1) * width + j]);
//       if (G > g) g = G;

//       // 90 degrees
//       G = abs(srcImg[(i - 1) * width + j + 1] + srcImg[i * width + j + 1] +
//               srcImg[(i + 1) * width + j + 1] -
//               srcImg[(i - 1) * width + j - 1] - srcImg[i * width + j - 1] -
//               srcImg[(i + 1) * width + j - 1]);
//       if (G > g) g = G;

//       // 135 degrees
//       G = abs(srcImg[(i - 1) * width + j] + srcImg[(i - 1) * width + j + 1] +
//               srcImg[i * width + j + 1] - srcImg[(i + 1) * width + j] -
//               srcImg[(i + 1) * width + j - 1] - srcImg[i * width + j - 1]);
//       if (G > g) g = G;
// #else
//       // Sobel
//       int g = 0, G;

//       // 0 degrees
//       G = abs(
//           srcImg[(i - 1) * width + j - 1] + 2 * srcImg[(i - 1) * width + j] +
//           srcImg[(i - 1) * width + j + 1] - srcImg[(i + 1) * width + j - 1] -
//           2 * srcImg[(i + 1) * width + j] - srcImg[(i + 1) * width + j + 1]);
//       if (G > g) g = G;

//       // 45 degrees
//       G = abs(srcImg[(i - 1) * width + j] +
//               2 * srcImg[(i - 1) * width + j - 1] + srcImg[i * width + j - 1] -
//               srcImg[(i + 1) * width + j] -
//               2 * srcImg[(i + 1) * width + j + 1] - srcImg[i * width + j + 1]);
//       if (G > g) g = G;

//       // 90 degrees
//       G = abs(srcImg[(i - 1) * width + j + 1] + 2 * srcImg[i * width + j + 1] +
//               srcImg[(i + 1) * width + j + 1] -
//               srcImg[(i - 1) * width + j - 1] - 2 * srcImg[i * width + j - 1] -
//               srcImg[(i + 1) * width + j - 1]);
//       if (G > g) g = G;

//       // 135 degrees
//       G = abs(srcImg[(i - 1) * width + j] +
//               2 * srcImg[(i - 1) * width + j + 1] + srcImg[i * width + j + 1] -
//               srcImg[(i + 1) * width + j] -
//               2 * srcImg[(i + 1) * width + j - 1] - srcImg[i * width + j - 1]);
//       if (G > g) g = G;
// #endif

//       gradImg[i * width + j] = g;
//       grads[g]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2) * (height - 2);
//   //  size -= grads[0];
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Compute np: # of segment pieces
// #if 1
//   // Does this underestimate the number of pieces of edge segments?
//   // What's the correct value?
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     int len = map->segments[i].noPixels;
//     np += (len * (len - 1)) / 2;
//   }  // end-for

// //  np *= 32;
// #elif 0
//   // This definitely overestimates the number of pieces of edge segments
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     np += map->segments[i].noPixels;
//   }  // end-for

//   np = (np * (np - 1)) / 2;
// #endif

//   // Validate segments
//   for (int i = 0; i < map->noSegments; i++) {
//     TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                 divForTestSegment);
//     //    TestSegment2(map, gradImg, i, 0, map->segments[i].noPixels-1, np, H,
//     //    divForTestSegment);
//   }  // end-for

//   /// Extract the new edge segments after validation
//   ExtractNewSegments(map);

//   delete H;
//   delete gradImg;
// }  // end-ValidateEdgeSegments2

///--------------------------------------------------------------------------------------------------------------------
/// Using multiple div values. Returns the number of maps
///
// int ValidateEdgeSegmentsMultipleDiv(EdgeMap *map, unsigned char *srcImg,
//                                     unsigned char *maps[], int noMaps) {
//   int width = map->width;
//   int height = map->height;

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   // Compute the gradient
//   short *gradImg = ComputePrewitt3x3(srcImg, width, height, H);

//   int index = 0;
//   for (double validationDiv = 1; validationDiv <= 8.5;
//        validationDiv += 0.5, index++) {
//     // Clear up the edge map
//     memset(map->edgeImg, 0, width * height);

//     // Compute np: # of segment pieces
//     int np = 0;
//     for (int i = 0; i < map->noSegments; i++) {
//       int len = map->segments[i].noPixels;
//       np += (len * (len - 1)) / 2;
//     }  // end-for

//     unsigned char *contourImg = maps[index];

//     if (contourImg == NULL) {
//       maps[index] = contourImg = new unsigned char[width * height];
//       memset(contourImg, 0, width * height);
//       noMaps++;
//     }  // end-if

//     // Validate segments
//     for (int i = 0; i < map->noSegments; i++) {
//       TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                   validationDiv);
//     }  // end-for

//     /// Extract the new edge segments after validation
//     ExtractNewSegments(map);

//     // Add the resulting edge map to contourImg
//     for (int i = 0; i < map->noSegments; i++) {
//       for (int j = 0; j < map->segments[i].noPixels; j++) {
//         int r = map->segments[i].pixels[j].r;
//         int c = map->segments[i].pixels[j].c;

//         contourImg[r * width + c]++;
//       }  // end-for
//     }    // end-for
//   }      // end-for

//   delete H;
//   delete gradImg;

//   return noMaps;
// }  // end-for

///--------------------------------------------------------------------------------------------------------------------
/// Validate the edge segments using the Helmholtz principle (for color images)
/// channel1, channel2 and channel3 images
///
// void ValidateEdgeSegments(EdgeMap *map, unsigned char *ch1Img,
//                           unsigned char *ch2Img, unsigned char *ch3Img,
//                           double divForTestSegment) {
//   int width = map->width;
//   int height = map->height;

//   memset(map->edgeImg, 0, width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   // Compute the gradient
//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Gradient for channel1
//       int com1 =
//           ch1Img[(i + 1) * width + j + 1] - ch1Img[(i - 1) * width + j - 1];
//       int com2 =
//           ch1Img[(i - 1) * width + j + 1] - ch1Img[(i + 1) * width + j - 1];

//       int gxCh1 = abs(com1 + com2 +
//                       (ch1Img[i * width + j + 1] - ch1Img[i * width + j - 1]));
//       int gyCh1 =
//           abs(com1 - com2 +
//               (ch1Img[(i + 1) * width + j] - ch1Img[(i - 1) * width + j]));
//       int ch1Grad = gxCh1 + gyCh1;

//       // Gradient for channel2
//       com1 = ch2Img[(i + 1) * width + j + 1] - ch2Img[(i - 1) * width + j - 1];
//       com2 = ch2Img[(i - 1) * width + j + 1] - ch2Img[(i + 1) * width + j - 1];

//       int gxCh2 = abs(com1 + com2 +
//                       (ch2Img[i * width + j + 1] - ch2Img[i * width + j - 1]));
//       int gyCh2 =
//           abs(com1 - com2 +
//               (ch2Img[(i + 1) * width + j] - ch2Img[(i - 1) * width + j]));
//       int ch2Grad = gxCh2 + gyCh2;

//       // Gradient for channel3
//       com1 = ch3Img[(i + 1) * width + j + 1] - ch3Img[(i - 1) * width + j - 1];
//       com2 = ch3Img[(i - 1) * width + j + 1] - ch3Img[(i + 1) * width + j - 1];

//       int gxCh3 = abs(com1 + com2 +
//                       (ch3Img[i * width + j + 1] - ch3Img[i * width + j - 1]));
//       int gyCh3 =
//           abs(com1 - com2 +
//               (ch3Img[(i + 1) * width + j] - ch3Img[(i - 1) * width + j]));
//       int ch3Grad = gxCh3 + gyCh3;

//       // Take average
// #if 1
//       int grad = (ch1Grad + ch2Grad + ch3Grad + 2) / 3;
// //      int grad = ch1Grad + ch2Grad + ch3Grad;
// #else
//       // Take the max
//       int grad = ch1Grad;
//       if (ch2Grad > grad) grad = ch2Grad;
//       if (ch3Grad > grad) grad = ch3Grad;
// #endif

//       gradImg[i * width + j] = grad;
//       grads[grad]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2) * (height - 2);
//   //  size -= grads[0];
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Compute np: # of segment pieces
// #if 1
//   // Does this underestimate the number of pieces of edge segments?
//   // What's the correct value?
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     int len = map->segments[i].noPixels;
//     np += (len * (len - 1)) / 2;
//   }  // end-for

// //  np *= 32;
// #elif 0
//   // This definitely overestimates the number of pieces of edge segments
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     np += map->segments[i].noPixels;
//   }  // end-for

//   np = (np * (np - 1)) / 2;
// #endif

//   // Validate segments
//   for (int i = 0; i < map->noSegments; i++) {
//     TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                 divForTestSegment);
//     //    TestSegment2(map, gradImg, i, 0, map->segments[i].noPixels-1, np, H,
//     //    divForTestSegment);
//   }  // end-for

//   /// Extract the new edge segments after validation
//   ExtractNewSegments(map);

//   delete H;
//   delete gradImg;
// }  // end-ValidateEdgeSegments

// ///--------------------------------------------------------------------------------------------------------------------
// /// Validate the edge segments using the Helmholtz principle (for color images)
// /// channel1, channel2 and channel3 images with LSD
// ///
// void ValidateEdgeSegments2(EdgeMap *map, unsigned char *ch1Img,
//                            unsigned char *ch2Img, unsigned char *ch3Img,
//                            double divForTestSegment) {
//   int width = map->width;
//   int height = map->height;

//   memset(map->edgeImg, 0, width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   // Compute the gradient
//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
// #if 0
//       // Gradient for channel1 -- LSD
//       int com1 = ch1Img[(i+1)*width+j+1] - ch1Img[i*width+j];
//       int com2 = ch1Img[i*width+j+1] - ch1Img[(i+1)*width+j];

//       int gx = abs(com1 + com2);
//       int gy = abs(com1 - com2);
//       int ch1Grad = gx+gy;

//       // Gradient for channel2
//       com1 = ch2Img[(i+1)*width+j+1] - ch2Img[i*width+j];
//       com2 = ch2Img[i*width+j+1] - ch2Img[(i+1)*width+j];

//       gx = abs(com1 + com2);
//       gy = abs(com1 - com2);
//       int ch2Grad = gx+gy;

//       // Gradient for channel3
//       com1 = ch3Img[(i+1)*width+j+1] - ch3Img[i*width+j];
//       com2 = ch3Img[i*width+j+1] - ch3Img[(i+1)*width+j];

//       gx = abs(com1 + com2);
//       gy = abs(com1 - com2);
//       int ch3Grad = gx+gy;

//       // Take average
//       int grad = (ch1Grad + ch2Grad + ch3Grad + 2)/3;
// #elif 1
//       // Use DiZenZo's gradient vector gradient computation
//       // Prewitt for channel1
//       int gxCh1 = ch1Img[(i - 1) * width + j + 1] -
//                   ch1Img[(i - 1) * width + j - 1] + ch1Img[i * width + j + 1] -
//                   ch1Img[i * width + j - 1] + ch1Img[(i + 1) * width + j + 1] -
//                   ch1Img[(i + 1) * width + j - 1];

//       int gyCh1 =
//           ch1Img[(i - 1) * width + j - 1] - ch1Img[(i + 1) * width + j - 1] +
//           ch1Img[(i - 1) * width + j] - ch1Img[(i + 1) * width + j] +
//           ch1Img[(i - 1) * width + j + 1] - ch1Img[(i + 1) * width + j + 1];

//       // Prewitt for channel2
//       int gxCh2 = ch2Img[(i - 1) * width + j + 1] -
//                   ch2Img[(i - 1) * width + j - 1] + ch2Img[i * width + j + 1] -
//                   ch2Img[i * width + j - 1] + ch2Img[(i + 1) * width + j + 1] -
//                   ch2Img[(i + 1) * width + j - 1];

//       int gyCh2 =
//           ch2Img[(i - 1) * width + j - 1] - ch2Img[(i + 1) * width + j - 1] +
//           ch2Img[(i - 1) * width + j] - ch2Img[(i + 1) * width + j] +
//           ch2Img[(i - 1) * width + j + 1] - ch2Img[(i + 1) * width + j + 1];

//       // Prewitt for channel3
//       int gxCh3 = ch3Img[(i - 1) * width + j + 1] -
//                   ch3Img[(i - 1) * width + j - 1] + ch3Img[i * width + j + 1] -
//                   ch3Img[i * width + j - 1] + ch3Img[(i + 1) * width + j + 1] -
//                   ch3Img[(i + 1) * width + j - 1];

//       int gyCh3 =
//           ch3Img[(i - 1) * width + j - 1] - ch3Img[(i + 1) * width + j - 1] +
//           ch3Img[(i - 1) * width + j] - ch3Img[(i + 1) * width + j] +
//           ch3Img[(i - 1) * width + j + 1] - ch3Img[(i + 1) * width + j + 1];

//       // Koschan & Abidi - 2005 - Signal Processing Magazine
//       int gxx = gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3;
//       int gyy = gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3;
//       int gxy = gxCh1 * gyCh1 + gxCh2 * gyCh2 + gxCh3 * gyCh3;

//       double theta = atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;

//       // Magnitude
//       //      int grad = (int)(sqrt(gxx*cos(theta)*cos(theta) +
//       //      2*gxy*sin(theta)*cos(theta) + gyy*sin(theta)*sin(theta)) + 0.5);
//       //      // Where did I get this from?
//       int grad = (int)(sqrt(((gxx + gyy) + (gxx - gyy) * cos(2 * theta) +
//                              2 * gxy * sin(2.0 * theta)) /
//                             2.0) +
//                        0.5);  // Image Processing Book??
// #endif

//       gradImg[i * width + j] = grad;
//       grads[grad]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2) * (height - 2);
//   //  size -= grads[0];
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Compute np: # of segment pieces
// #if 1
//   // Does this underestimate the number of pieces of edge segments?
//   // What's the correct value?
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     int len = map->segments[i].noPixels;
//     np += (len * (len - 1)) / 2;
//   }  // end-for

// //  np *= 32;
// #elif 0
//   // This definitely overestimates the number of pieces of edge segments
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     np += map->segments[i].noPixels;
//   }  // end-for

//   np = (np * (np - 1)) / 2;
// #endif

//   // Validate segments
//   for (int i = 0; i < map->noSegments; i++) {
//     TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                 divForTestSegment);
//     //    TestSegment2(map, gradImg, i, 0, map->segments[i].noPixels-1, np, H,
//     //    divForTestSegment);
//   }  // end-for

//   /// Extract the new edge segments after validation
//   ExtractNewSegments(map);

//   delete H;
//   delete gradImg;
// }  // end-ValidateEdgeSegments2

// ///--------------------------------------------------------------------------------------------------------------------
// /// Validate the edge segments using the Helmholtz principle (for color images)
// /// channel1, channel2 and channel3 images
// ///
// int ValidateEdgeSegmentsMultipleDiv(EdgeMap *map, unsigned char *ch1Img,
//                                     unsigned char *ch2Img,
//                                     unsigned char *ch3Img,
//                                     unsigned char *maps[], int noMaps, int op) {
//   int width = map->width;
//   int height = map->height;

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   // Compute the gradient
//   short *gradImg = new short[width * height];
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   if (op == 1) {
//     for (int i = 1; i < height - 1; i++) {
//       for (int j = 1; j < width - 1; j++) {
//         // Gradient for channel1
//         int com1 =
//             ch1Img[(i + 1) * width + j + 1] - ch1Img[(i - 1) * width + j - 1];
//         int com2 =
//             ch1Img[(i - 1) * width + j + 1] - ch1Img[(i + 1) * width + j - 1];

//         int gxCh1 =
//             abs(com1 + com2 +
//                 (ch1Img[i * width + j + 1] - ch1Img[i * width + j - 1]));
//         int gyCh1 =
//             abs(com1 - com2 +
//                 (ch1Img[(i + 1) * width + j] - ch1Img[(i - 1) * width + j]));
//         int ch1Grad = gxCh1 + gyCh1;

//         // Gradient for channel2
//         com1 =
//             ch2Img[(i + 1) * width + j + 1] - ch2Img[(i - 1) * width + j - 1];
//         com2 =
//             ch2Img[(i - 1) * width + j + 1] - ch2Img[(i + 1) * width + j - 1];

//         int gxCh2 =
//             abs(com1 + com2 +
//                 (ch2Img[i * width + j + 1] - ch2Img[i * width + j - 1]));
//         int gyCh2 =
//             abs(com1 - com2 +
//                 (ch2Img[(i + 1) * width + j] - ch2Img[(i - 1) * width + j]));
//         int ch2Grad = gxCh2 + gyCh2;

//         // Gradient for channel3
//         com1 =
//             ch3Img[(i + 1) * width + j + 1] - ch3Img[(i - 1) * width + j - 1];
//         com2 =
//             ch3Img[(i - 1) * width + j + 1] - ch3Img[(i + 1) * width + j - 1];

//         int gxCh3 =
//             abs(com1 + com2 +
//                 (ch3Img[i * width + j + 1] - ch3Img[i * width + j - 1]));
//         int gyCh3 =
//             abs(com1 - com2 +
//                 (ch3Img[(i + 1) * width + j] - ch3Img[(i - 1) * width + j]));
//         int ch3Grad = gxCh3 + gyCh3;

//         // Take average
//         //        int grad = (ch1Grad + ch2Grad + ch3Grad + 2)/3;  // works
//         //        better
//         int grad = (ch1Grad + ch2Grad + ch3Grad + 1) / 3;
//         //        int grad = ch1Grad + ch2Grad + ch3Grad;

//         gradImg[i * width + j] = grad;
//         grads[grad]++;
//       }  // end-for
//     }    // end-for

//   } else {
//     // 2x2 finite differences (LSD)
//     for (int i = 1; i < height - 1; i++) {
//       for (int j = 1; j < width - 1; j++) {
//         // 2x2 finite difference (LSD) gradient
//         int com1 = ch1Img[(i + 1) * width + j + 1] - ch1Img[i * width + j];
//         int com2 = ch1Img[i * width + j + 1] - ch1Img[(i + 1) * width + j];

//         int gxCh1 = abs(com1 + com2);
//         int gyCh1 = abs(com1 - com2);
//         int ch1Grad = gxCh1 + gyCh1;

//         // Gradient for channel2
//         com1 = ch2Img[(i + 1) * width + j + 1] - ch2Img[i * width + j];
//         com2 = ch2Img[i * width + j + 1] - ch2Img[(i + 1) * width + j];

//         int gxCh2 = abs(com1 + com2);
//         int gyCh2 = abs(com1 - com2);
//         int ch2Grad = gxCh2 + gyCh2;

//         // Gradient for channel3
//         com1 = ch3Img[(i + 1) * width + j + 1] - ch3Img[i * width + j];
//         com2 = ch3Img[i * width + j + 1] - ch3Img[(i + 1) * width + j];

//         int gxCh3 = abs(com1 + com2);
//         int gyCh3 = abs(com1 - com2);
//         int ch3Grad = gxCh3 + gyCh3;

//         int grad = (ch1Grad + ch2Grad + ch3Grad + 2) / 3;  // original

//         gradImg[i * width + j] = grad;
//         grads[grad]++;
//       }  // end-for
//     }    // end-for
//   }      // end-else

//   // Compute probability function H
//   int size = (width - 2) * (height - 2);
//   //  size -= grads[0];
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Validate for different div values
//   int index = 0;
//   for (double validationDiv = 1; validationDiv <= 8.5;
//        validationDiv += 0.5, index++) {  // Orig loop -- good results
//     unsigned char *contourImg = maps[index];

//     if (contourImg == NULL) {
//       maps[index] = contourImg = new unsigned char[width * height];
//       memset(contourImg, 0, width * height);
//       noMaps++;
//     }  // end-if

//     memset(map->edgeImg, 0, width * height);

//     // Compute np: # of segment pieces
//     int np = 0;
//     for (int i = 0; i < map->noSegments; i++) {
//       int len = map->segments[i].noPixels;
//       np += (len * (len - 1)) / 2;
//     }  // end-for

//     // Validate segments
//     for (int i = 0; i < map->noSegments; i++) {
//       TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                   validationDiv);
//     }  // end-for

//     // Extract the new edge segments after validation
//     ExtractNewSegments(map);

//     // Add the resulting edge map to contourImg
//     for (int i = 0; i < map->noSegments; i++) {
//       for (int j = 0; j < map->segments[i].noPixels; j++) {
//         int r = map->segments[i].pixels[j].r;
//         int c = map->segments[i].pixels[j].c;

//         contourImg[r * width + c]++;
//       }  // end-for
//     }    // end-for
//   }      // end-for

//   delete H;
//   delete gradImg;

//   return noMaps;
// }  // end-ValidateEdgeSegmentsMultipleDiv

// ///----------------------------------------------------------------------------------
// /// Validation given an already computed gradient image
// ///
// void ValidateEdgeSegmentsWithGradientMap(EdgeMap *map, short *gradImg,
//                                          double divForTestSegment) {
//   int width = map->width;
//   int height = map->height;

//   memset(map->edgeImg, 0, width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   int offset = 1;
//   for (int i = offset; i < height - offset; i++) {
//     for (int j = offset; j < width - offset; j++) {
//       int g = gradImg[i * width + j];

//       grads[g]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2 * offset) * (height - 2 * offset);
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Compute np: # of segment pieces
//   int np = 0;
//   for (int i = 0; i < map->noSegments; i++) {
//     int len = map->segments[i].noPixels;
//     np += (len * (len - 1)) / 2;
//   }  // end-for

//   // Validate segments
//   for (int i = 0; i < map->noSegments; i++) {
//     TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                 divForTestSegment);
//   }  // end-for

//   /// Extract the new edge segments after validation
//   ExtractNewSegments(map);

//   delete H;
// }  // end-ValidateEdgeSegmentsWithGradientMap

// ///----------------------------------------------------------------------------------
// /// Validation given an already computed gradient image
// ///
// int ValidateEdgeSegmentsWithGradientMapMultipleDiv(EdgeMap *map, short *gradImg,
//                                                    unsigned char *maps[],
//                                                    int noMaps) {
//   int width = map->width;
//   int height = map->height;

//   memset(map->edgeImg, 0, width * height);

//   int maxGradValue = MAX_GRAD_VALUE;
//   double *H = new double[maxGradValue];
//   memset(H, 0, sizeof(double) * maxGradValue);

//   int *grads = new int[maxGradValue];
//   memset(grads, 0, sizeof(int) * maxGradValue);

//   int offset = 1;
//   for (int i = offset; i < height - offset; i++) {
//     for (int j = offset; j < width - offset; j++) {
//       int g = gradImg[i * width + j];

//       grads[g]++;
//     }  // end-for
//   }    // end-for

//   // Compute probability function H
//   int size = (width - 2 * offset) * (height - 2 * offset);
//   for (int i = maxGradValue - 1; i > 0; i--) grads[i - 1] += grads[i];
//   for (int i = 0; i < maxGradValue; i++)
//     H[i] = (double)grads[i] / ((double)size);

//   delete grads;

//   // Validate for different div values
//   int index = 0;
//   for (double validationDiv = 1; validationDiv <= 8.5;
//        validationDiv += 0.5, index++) {
//     unsigned char *contourImg = maps[index];

//     if (contourImg == NULL) {
//       maps[index] = contourImg = new unsigned char[width * height];
//       memset(contourImg, 0, width * height);
//       noMaps++;
//     }  // end-if

//     memset(map->edgeImg, 0, width * height);

//     // Compute np: # of segment pieces
//     int np = 0;
//     for (int i = 0; i < map->noSegments; i++) {
//       int len = map->segments[i].noPixels;
//       np += (len * (len - 1)) / 2;
//     }  // end-for

//     // Validate segments
//     for (int i = 0; i < map->noSegments; i++) {
//       TestSegment(map, gradImg, i, 0, map->segments[i].noPixels - 1, np, H,
//                   validationDiv);
//     }  // end-for

//     // Extract the new edge segments after validation
//     ExtractNewSegments(map);

//     // Add the resulting edge map to contourImg
//     for (int i = 0; i < map->noSegments; i++) {
//       for (int j = 0; j < map->segments[i].noPixels; j++) {
//         int r = map->segments[i].pixels[j].r;
//         int c = map->segments[i].pixels[j].c;

//         contourImg[r * width + c]++;
//       }  // end-for
//     }    // end-for
//   }      // end-for

//   delete H;

//   return noMaps;
// }  // end-ValidateEdgeSegmentsWithGradientMapMultipleDiv
