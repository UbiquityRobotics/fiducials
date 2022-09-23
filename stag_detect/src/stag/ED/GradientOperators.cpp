#include <stdlib.h>
#include <math.h>

#include "stag/ED/GradientOperators.h"
// #include "stag/ED/GradientOperatorsCV.h"
// #include "stag/ED/Utilities.h"

/// Special defines
#define EDGE_VERTICAL 1
#define EDGE_HORIZONTAL 2
#define EDGE_45 3
#define EDGE_135 4

/// How to compute the gradient values
#define USE_GX_GY_PLUS 1  // compute the gradient by G = Gx+Gy
//#define USE_GX_GY_SQRT 1  // compute the gradient by G = sqrt(Gx*Gx+Gy*Gy)

///---------------------------------------------------------
/// LSD Operator
///
// void ComputeGradientMapByLSD(unsigned char *smoothImg, short *gradImg,
//                              unsigned char *dirImg, int width, int height,
//                              int GRADIENT_THRESH) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   // Compute the gradient image and edge directions for the rest of the pixels
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
//       int com1 = smoothImg[(i + 1) * width + j + 1] - smoothImg[i * width + j];
//       int com2 = smoothImg[i * width + j + 1] - smoothImg[(i + 1) * width + j];

//       int gx = abs(com1 + com2);
//       int gy = abs(com1 - com2);

// #if USE_GX_GY_PLUS
//       int sum = gx + gy;
// #elif USE_GX_GY_SQRT
//       int sum = sqrtLUT[gx][gy];
// #endif

//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (sum >= GRADIENT_THRESH) {
//         if (gx >= gy)
//           dirImg[index] = EDGE_VERTICAL;
//         else
//           dirImg[index] = EDGE_HORIZONTAL;
//       }  // end-if

//     }  // end-for
//   }    // end-for
// }  // end-ComputeGradientMapByLSD

///---------------------------------------------------------
/// Prewitt Operator
///
void ComputeGradientMapByPrewitt(unsigned char *smoothImg, short *gradImg,
                                 unsigned char *dirImg, int width, int height,
                                 int GRADIENT_THRESH) {
  // Initialize gradient image for row = 0, row = height-1, column=0,
  // column=width-1
  for (int j = 0; j < width; j++) {
    gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
  }
  for (int i = 1; i < height - 1; i++) {
    gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
  }

  // Compute the gradient image and edge directions for the rest of the pixels
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
      int com1 = smoothImg[(i + 1) * width + j + 1] -
                 smoothImg[(i - 1) * width + j - 1];
      int com2 = smoothImg[(i - 1) * width + j + 1] -
                 smoothImg[(i + 1) * width + j - 1];

      int gx =
          abs(com1 + com2 +
              (smoothImg[i * width + j + 1] - smoothImg[i * width + j - 1]));
      int gy = abs(
          com1 - com2 +
          (smoothImg[(i + 1) * width + j] - smoothImg[(i - 1) * width + j]));

#if USE_GX_GY_PLUS
      int sum = gx + gy;
#elif USE_GX_GY_SQRT
      int sum = sqrtLUT[gx][gy];
#endif

      int index = i * width + j;
      gradImg[index] = sum;

      if (sum >= GRADIENT_THRESH) {
        if (gx >= gy)
          dirImg[index] = EDGE_VERTICAL;
        else
          dirImg[index] = EDGE_HORIZONTAL;
      }  // end-if
    }    // end-for
  }      // end-for
}  // end-ComputeGradientMapByPrewitt

///-----------------------------------------------------------------------
/// Scaled Prewitt Operator: Scale the gradient values to [0-255]
///
// void ComputeGradientMapByPrewitt(unsigned char *smoothImg, short *gradImg,
//                                  unsigned char *dirImg, int width, int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   // Compute the gradient image and edge directions for the rest of the pixels
//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt Operator in horizontal and vertical direction
//       // A B C
//       // D x E
//       // F G H
//       // gx = (C-A) + (E-D) + (H-F)
//       // gy = (F-A) + (G-B) + (H-C)
//       //
//       // To make this faster:
//       // com1 = (H-A)
//       // com2 = (C-F)
//       // Then: gx = com1 + com2 + (E-D) = (H-A) + (C-F) + (E-D) = (C-A) + (E-D)
//       // + (H-F)
//       //       gy = com1 - com2 + (G-B) = (H-A) - (C-F) + (G-B) = (F-A) + (G-B)
//       //       + (H-C)
//       //
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
//         dirImg[index] = EDGE_VERTICAL;
//       else
//         dirImg[index] = EDGE_HORIZONTAL;

//       if (sum > max) max = sum;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient
//   double scale = 255.0 / max;
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(scale * gradImg[i]);
// }  // end-ComputeGradientMapByPrewitt

// ///---------------------------------------------------------
// /// Prewitt Operator
// ///
// void ComputeGradientMapByPrewitt4Dirs(unsigned char *smoothImg, short *gradImg,
//                                       unsigned char *dirImg, int width,
//                                       int height, int GRADIENT_THRESH) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   // Compute the gradient image and edge directions for the rest of the pixels
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Use 4 directions
//       int gx = smoothImg[(i - 1) * width + j + 1] -
//                smoothImg[(i - 1) * width + j - 1] +
//                smoothImg[i * width + j + 1] - smoothImg[i * width + j - 1] +
//                smoothImg[(i + 1) * width + j + 1] -
//                smoothImg[(i + 1) * width + j - 1];

//       int gy = smoothImg[(i - 1) * width + j - 1] -
//                smoothImg[(i + 1) * width + j - 1] +
//                smoothImg[(i - 1) * width + j] - smoothImg[(i + 1) * width + j] +
//                smoothImg[(i - 1) * width + j + 1] -
//                smoothImg[(i + 1) * width + j + 1];

//       int sum = abs(gx) + abs(gy);
//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (sum >= GRADIENT_THRESH) {
//         // Compute the gradient direction
//         double angle = atan2((double)gy, (double)gx);

// #define MYPI 3.14159
// #define MYPIDIV8 MYPI / 8
//         if (angle > MYPI / 2)
//           angle -= MYPI;
//         else if (angle < -MYPI / 2)
//           angle += MYPI;
//         angle += MYPI / 2;

//         if (angle <= MYPIDIV8)
//           dirImg[index] = EDGE_HORIZONTAL;
//         else if (angle <= 3 * MYPIDIV8)
//           dirImg[index] = EDGE_45;
//         else if (angle <= 5 * MYPIDIV8)
//           dirImg[index] = EDGE_VERTICAL;
//         else if (angle <= 7 * MYPIDIV8)
//           dirImg[index] = EDGE_135;
//         else
//           dirImg[index] = EDGE_HORIZONTAL;
//       }  // end-if
//     }    // end-for
//   }      // end-for
// }  // end-ComputeGradientMapByPrewitt4Dirs

// ///------------------------------------------------------------------------------------
// /// Performs Sobel Operator on the image
// ///
// void ComputeGradientMapBySobel(unsigned char *smoothImg, short *gradImg,
//                                unsigned char *dirImg, int width, int height,
//                                int GRADIENT_THRESH) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   // Compute the gradient image and edge directions for the rest of the pixels
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Sobel Operator in horizontal and vertical direction
//       // Faster method below
//       // A B C
//       // D x E
//       // F G H
//       // gx = (C-A) + 2*(E-D) + (H-F)
//       // gy = (F-A) + 2*(G-B) + (H-C)
//       //
//       // To make this faster:
//       // com1 = (H-A)
//       // com2 = (C-F)
//       // Then: gx = com1 + com2 + 2*(E-D) = (H-A) + (C-F) + 2*(E-D) = (C-A) +
//       // 2*(E-D) + (H-F)
//       //       gy = com1 - com2 + 2*(G-B) = (H-A) - (C-F) + 2*(G-B) = (F-A) +
//       //       2*(G-B) + (H-C)
//       //
//       int com1 = smoothImg[(i + 1) * width + j + 1] -
//                  smoothImg[(i - 1) * width + j - 1];
//       int com2 = smoothImg[(i - 1) * width + j + 1] -
//                  smoothImg[(i + 1) * width + j - 1];

//       int gx = abs(
//           com1 + com2 +
//           2 * (smoothImg[i * width + j + 1] - smoothImg[i * width + j - 1]));
//       int gy = abs(com1 - com2 +
//                    2 * (smoothImg[(i + 1) * width + j] -
//                         smoothImg[(i - 1) * width + j]));

// #if USE_GX_GY_PLUS
//       int sum = gx + gy;
// #elif USE_GX_GY_SQRT
//       int sum = sqrtLUT[gx][gy];
// #endif

//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (sum >= GRADIENT_THRESH) {
//         if (gx >= gy)
//           dirImg[index] = EDGE_VERTICAL;
//         else
//           dirImg[index] = EDGE_HORIZONTAL;
//       }  // end-if

//     }  // end-for
//   }    // end-for
// }  // end-ComputeGradientMapBySobel

// ///------------------------------------------------------------------------------------
// /// Performs Scharr Operator on the image
// ///
// void ComputeGradientMapByScharr(unsigned char *smoothImg, short *gradImg,
//                                 unsigned char *dirImg, int width, int height,
//                                 int GRADIENT_THRESH) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   // Compute the gradient image and edge directions for the rest of the pixels
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Scharr Operator in horizontal and vertical direction
//       // A B C
//       // D x E
//       // F G H
//       // gx = 3*(C-A) + 10*(E-D) + 3*(H-F)
//       // gy = 3*(F-A) + 10*(G-B) + 3*(H-C)
//       //
//       // To make this faster:
//       // com1 = (H-A)
//       // com2 = (C-F)
//       // Then: gx = 3*(com1 + com2) + 10*(E-D) = 3*(H-A) + 3*(C-F) + 10*(E-D) =
//       // 3*(C-A) + 10*(E-D) + 3*(H-F)
//       //       gy = 3*(com1 - com2) + 10*(G-B) = 3*(H-A) - 3*(C-F) + 10*(G-B) =
//       //       3*(F-A) + 10*(G-B) + 3*(H-C)
//       //
//       int com1 = smoothImg[(i + 1) * width + j + 1] -
//                  smoothImg[(i - 1) * width + j - 1];
//       int com2 = smoothImg[(i - 1) * width + j + 1] -
//                  smoothImg[(i + 1) * width + j - 1];

//       int gx = abs(3 * (com1 + com2) + 10 * (smoothImg[i * width + j + 1] -
//                                              smoothImg[i * width + j - 1]));
//       int gy = abs(3 * (com1 - com2) + 10 * (smoothImg[(i + 1) * width + j] -
//                                              smoothImg[(i - 1) * width + j]));

// #if USE_GX_GY_PLUS
//       int sum = gx + gy;
// #elif USE_GX_GY_SQRT
//       //      int sum = (int)(sqrt((double)gx*gx + gy*gy)+0.5);
//       int sum = (int)(fastsqrt2((float)(gx * gx + gy * gy)) + 0.5f);
// #endif

//       int index = i * width + j;
//       gradImg[index] = sum;

//       if (sum >= GRADIENT_THRESH) {
//         if (gx >= gy)
//           dirImg[index] = EDGE_VERTICAL;
//         else
//           dirImg[index] = EDGE_HORIZONTAL;
//       }  // end-if

//     }  // end-for
//   }    // end-for
// }  // end-ComputeGradientMapByScharr

// /****************************** GRADIENT OPERATORS FOR COLOR IMAGES
//  * ***********************************/
// /// 3x3 Prewitt for color images
// ///
// void ComputeGradientMapByPrewitt(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height,
//                                  int GRADIENT_THRESH) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 = abs(
//           com1 + com2 +
//           (smoothCh1Img[i * width + j + 1] - smoothCh1Img[i * width + j - 1]));
//       int gyCh1 = abs(com1 - com2 +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]));
//       int gradCh1 = gxCh1 + gyCh1;

//       // Prewitt for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 = abs(
//           com1 + com2 +
//           (smoothCh2Img[i * width + j + 1] - smoothCh2Img[i * width + j - 1]));
//       int gyCh2 = abs(com1 - com2 +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]));
//       int gradCh2 = gxCh2 + gyCh2;

//       // Prewitt for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 = abs(
//           com1 + com2 +
//           (smoothCh3Img[i * width + j + 1] - smoothCh3Img[i * width + j - 1]));
//       int gyCh3 = abs(com1 - com2 +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]));
//       int gradCh3 = gxCh3 + gyCh3;

//       // Combine the gradients
//       int gx, gy, grad;
// #if 1
//       // Avg
//       gx = (gxCh1 + gxCh2 + gxCh3 + 2) / 3;
//       gy = (gyCh1 + gyCh2 + gyCh3 + 2) / 3;
//       grad = (gradCh1 + gradCh2 + gradCh3 + 2) / 3;

// #elif 0
//       // Sum
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       grad = gradCh1 + gradCh2 + gradCh3;

// #elif 1
//       // Max
//       grad = gradCh1;
//       gx = gxCh1;
//       gy = gyCh1;
//       if (gradCh2 > grad) {
//         grad = gradCh2;
//         gx = gxCh2;
//         gy = gyCh2;
//       }
//       if (gradCh3 > grad) {
//         grad = gradCh3;
//         gx = gxCh3;
//         gy = gyCh3;
//       }
// #endif

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;

//       gradImg[i * width + j] = grad;
//     }  // end-for
//   }    // end-for
// }  // end-ComputeGradientMapByPrewitt

// ///--------------------------------------------------------------------------------------------------------------------
// /// 3x3 Prewitt using L1 norm -- scaled to [0-255]
// ///
// void ComputeGradientMapByPrewitt(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = 0;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = 0;
//   }

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 = abs(
//           com1 + com2 +
//           (smoothCh1Img[i * width + j + 1] - smoothCh1Img[i * width + j - 1]));
//       int gyCh1 = abs(com1 - com2 +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]));

//       // Prewitt for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 = abs(
//           com1 + com2 +
//           (smoothCh2Img[i * width + j + 1] - smoothCh2Img[i * width + j - 1]));
//       int gyCh2 = abs(com1 - com2 +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]));

//       // Prewitt for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 = abs(
//           com1 + com2 +
//           (smoothCh3Img[i * width + j + 1] - smoothCh3Img[i * width + j - 1]));
//       int gyCh3 = abs(com1 - com2 +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]));

//       // Combine the gradients
//       int gx, gy;

// #if 1
//       // L1-Norm: |G1| + |G2| + |G3|
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       int grad = gx + gy;
// #else
//       // L-Infinity-Norm: Max{|G1|, |G2|, |G3|}
//       gx = gxCh1;
//       if (gxCh2 > gx) gx = gxCh2;
//       if (gxCh3 > gx) gx = gxCh3;
//       gy = gyCh1;
//       if (gyCh2 > gy) gy = gyCh2;
//       if (gyCh3 > gy) gy = gyCh3;
//       int grad = gx + gy;
// #endif

//       int index = i * width + j;
//       if (gx > gy)
//         dirImg[index] = EDGE_VERTICAL;
//       else
//         dirImg[index] = EDGE_HORIZONTAL;

//       gradImg[index] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapByPrewitt

// ///--------------------------------------------------------------------------------------------------------------------
// /// 3x3 Prewitt using L2 norm -- scaled to [0-255]
// ///
// void ComputeGradientMapByPrewittL2(unsigned char *smoothCh1Img,
//                                    unsigned char *smoothCh2Img,
//                                    unsigned char *smoothCh3Img, short *gradImg,
//                                    unsigned char *dirImg, int width,
//                                    int height) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = 0;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = 0;
//   }

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 = abs(
//           com1 + com2 +
//           (smoothCh1Img[i * width + j + 1] - smoothCh1Img[i * width + j - 1]));
//       int gyCh1 = abs(com1 - com2 +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]));

//       // Prewitt for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 = abs(
//           com1 + com2 +
//           (smoothCh2Img[i * width + j + 1] - smoothCh2Img[i * width + j - 1]));
//       int gyCh2 = abs(com1 - com2 +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]));

//       // Prewitt for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 = abs(
//           com1 + com2 +
//           (smoothCh3Img[i * width + j + 1] - smoothCh3Img[i * width + j - 1]));
//       int gyCh3 = abs(com1 - com2 +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]));

//       // Combine the gradients
//       int gx, gy;

//       // L2-Norm: {|G1|^2 + |G2|^2 + |G3|^2}^0.5
//       gx = (int)(sqrt((double)gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3) +
//                  0.5);
//       gy = (int)(sqrt((double)gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3) +
//                  0.5);
//       int grad = (int)(sqrt((double)gx * gx + gy * gy) + 0.5);

//       int index = i * width + j;
//       if (gx > gy)
//         dirImg[index] = EDGE_VERTICAL;
//       else
//         dirImg[index] = EDGE_HORIZONTAL;

//       gradImg[index] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapByPrewittL2

// ///---------------------------------------------------------------------------------------------------------------
// /// 5x5 Prewitt -- scaled to [0-255]
// ///
// void ComputeGradientMapByPrewitt5x5(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 2; i < height - 2; i++) {
//     for (int j = 2; j < width - 2; j++) {
//       // Prewitt for channel1
//       int gxCh1 = abs(
//           (smoothCh1Img[(i - 2) * width + j + 1] -
//            smoothCh1Img[(i - 2) * width + j - 1]) +
//           (smoothCh1Img[(i - 1) * width + j + 1] -
//            smoothCh1Img[(i - 1) * width + j - 1]) +
//           (smoothCh1Img[(i)*width + j + 1] - smoothCh1Img[(i)*width + j - 1]) +
//           (smoothCh1Img[(i + 1) * width + j + 1] -
//            smoothCh1Img[(i + 1) * width + j - 1]) +
//           (smoothCh1Img[(i + 2) * width + j + 1] -
//            smoothCh1Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh1Img[(i - 2) * width + j + 2] -
//                smoothCh1Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh1Img[(i - 1) * width + j + 2] -
//                smoothCh1Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i)*width + j + 2] -
//                smoothCh1Img[(i)*width + j - 2]) +
//           2 * (smoothCh1Img[(i + 1) * width + j + 2] -
//                smoothCh1Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                smoothCh1Img[(i + 2) * width + j - 2]));

//       int gyCh1 = abs((smoothCh1Img[(i + 1) * width + j - 2] -
//                        smoothCh1Img[(i - 1) * width + j - 2]) +
//                       (smoothCh1Img[(i + 1) * width + j - 1] -
//                        smoothCh1Img[(i - 1) * width + j - 1]) +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]) +
//                       (smoothCh1Img[(i + 1) * width + j + 1] -
//                        smoothCh1Img[(i - 1) * width + j + 1]) +
//                       (smoothCh1Img[(i + 1) * width + j + 2] -
//                        smoothCh1Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh1Img[(i + 2) * width + j - 2] -
//                            smoothCh1Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j - 1] -
//                            smoothCh1Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j] -
//                            smoothCh1Img[(i - 2) * width + j]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 1] -
//                            smoothCh1Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                            smoothCh1Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel2
//       int gxCh2 = abs(
//           (smoothCh2Img[(i - 2) * width + j + 1] -
//            smoothCh2Img[(i - 2) * width + j - 1]) +
//           (smoothCh2Img[(i - 1) * width + j + 1] -
//            smoothCh2Img[(i - 1) * width + j - 1]) +
//           (smoothCh2Img[(i)*width + j + 1] - smoothCh2Img[(i)*width + j - 1]) +
//           (smoothCh2Img[(i + 1) * width + j + 1] -
//            smoothCh2Img[(i + 1) * width + j - 1]) +
//           (smoothCh2Img[(i + 2) * width + j + 1] -
//            smoothCh2Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh2Img[(i - 2) * width + j + 2] -
//                smoothCh2Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh2Img[(i - 1) * width + j + 2] -
//                smoothCh2Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i)*width + j + 2] -
//                smoothCh2Img[(i)*width + j - 2]) +
//           2 * (smoothCh2Img[(i + 1) * width + j + 2] -
//                smoothCh2Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                smoothCh2Img[(i + 2) * width + j - 2]));

//       int gyCh2 = abs((smoothCh2Img[(i + 1) * width + j - 2] -
//                        smoothCh2Img[(i - 1) * width + j - 2]) +
//                       (smoothCh2Img[(i + 1) * width + j - 1] -
//                        smoothCh2Img[(i - 1) * width + j - 1]) +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]) +
//                       (smoothCh2Img[(i + 1) * width + j + 1] -
//                        smoothCh2Img[(i - 1) * width + j + 1]) +
//                       (smoothCh2Img[(i + 1) * width + j + 2] -
//                        smoothCh2Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh2Img[(i + 2) * width + j - 2] -
//                            smoothCh2Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j - 1] -
//                            smoothCh2Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j] -
//                            smoothCh2Img[(i - 2) * width + j]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 1] -
//                            smoothCh2Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                            smoothCh2Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel3
//       int gxCh3 = abs(
//           (smoothCh3Img[(i - 2) * width + j + 1] -
//            smoothCh3Img[(i - 2) * width + j - 1]) +
//           (smoothCh3Img[(i - 1) * width + j + 1] -
//            smoothCh3Img[(i - 1) * width + j - 1]) +
//           (smoothCh3Img[(i)*width + j + 1] - smoothCh3Img[(i)*width + j - 1]) +
//           (smoothCh3Img[(i + 1) * width + j + 1] -
//            smoothCh3Img[(i + 1) * width + j - 1]) +
//           (smoothCh3Img[(i + 2) * width + j + 1] -
//            smoothCh3Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh3Img[(i - 2) * width + j + 2] -
//                smoothCh3Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh3Img[(i - 1) * width + j + 2] -
//                smoothCh3Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i)*width + j + 2] -
//                smoothCh3Img[(i)*width + j - 2]) +
//           2 * (smoothCh3Img[(i + 1) * width + j + 2] -
//                smoothCh3Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                smoothCh3Img[(i + 2) * width + j - 2]));

//       int gyCh3 = abs((smoothCh3Img[(i + 1) * width + j - 2] -
//                        smoothCh3Img[(i - 1) * width + j - 2]) +
//                       (smoothCh3Img[(i + 1) * width + j - 1] -
//                        smoothCh3Img[(i - 1) * width + j - 1]) +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]) +
//                       (smoothCh3Img[(i + 1) * width + j + 1] -
//                        smoothCh3Img[(i - 1) * width + j + 1]) +
//                       (smoothCh3Img[(i + 1) * width + j + 2] -
//                        smoothCh3Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh3Img[(i + 2) * width + j - 2] -
//                            smoothCh3Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j - 1] -
//                            smoothCh3Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j] -
//                            smoothCh3Img[(i - 2) * width + j]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 1] -
//                            smoothCh3Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                            smoothCh3Img[(i - 2) * width + j + 2]));

//       // Combine the gradients
//       int gx, gy, grad;

//       // L1-norm
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       grad = gx + gy;

//       // L2-norm
//       //      gx = (int)(sqrt((double)gxCh1*gxCh1 + gxCh2*gxCh2 +
//       //      gxCh3*gxCh3)+0.5); gy = (int)(sqrt((double)gyCh1*gyCh1 +
//       //      gyCh2*gyCh2 + gyCh3*gyCh3)+0.5); grad =
//       //      (int)(sqrt((double)gx*gx+gy*gy)+0.5);

//       if (grad > max) max = grad;
//       gradImg[i * width + j] = grad;

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapByPrewitt5x5

// ///---------------------------------------------------------------------------------------------------------------
// /// 7x7 Prewitt -- scaled to [0-255]
// ///
// void ComputeGradientMapByPrewitt7x7(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 3; i < height - 3; i++) {
//     for (int j = 3; j < width - 3; j++) {
//       // Prewitt for channel1
//       int gxCh1 = abs(
//           (smoothCh1Img[(i - 3) * width + j + 1] -
//            smoothCh1Img[(i - 3) * width + j - 1]) +
//           (smoothCh1Img[(i - 2) * width + j + 1] -
//            smoothCh1Img[(i - 2) * width + j - 1]) +
//           (smoothCh1Img[(i - 1) * width + j + 1] -
//            smoothCh1Img[(i - 1) * width + j - 1]) +
//           (smoothCh1Img[(i)*width + j + 1] - smoothCh1Img[(i)*width + j - 1]) +
//           (smoothCh1Img[(i + 1) * width + j + 1] -
//            smoothCh1Img[(i + 1) * width + j - 1]) +
//           (smoothCh1Img[(i + 2) * width + j + 1] -
//            smoothCh1Img[(i + 2) * width + j - 1]) +
//           (smoothCh1Img[(i + 3) * width + j + 1] -
//            smoothCh1Img[(i + 3) * width + j - 1]) +

//           2 * (smoothCh1Img[(i - 3) * width + j + 2] -
//                smoothCh1Img[(i - 3) * width + j - 2]) +
//           2 * (smoothCh1Img[(i - 2) * width + j + 2] -
//                smoothCh1Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh1Img[(i - 1) * width + j + 2] -
//                smoothCh1Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i)*width + j + 2] -
//                smoothCh1Img[(i)*width + j - 2]) +
//           2 * (smoothCh1Img[(i + 1) * width + j + 2] -
//                smoothCh1Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                smoothCh1Img[(i + 2) * width + j - 2]) +
//           2 * (smoothCh1Img[(i + 3) * width + j + 2] -
//                smoothCh1Img[(i + 3) * width + j - 2]) +

//           4 * (smoothCh1Img[(i - 3) * width + j + 3] -
//                smoothCh1Img[(i - 3) * width + j - 3]) +
//           4 * (smoothCh1Img[(i - 2) * width + j + 3] -
//                smoothCh1Img[(i - 2) * width + j - 3]) +
//           4 * (smoothCh1Img[(i - 1) * width + j + 3] -
//                smoothCh1Img[(i - 1) * width + j - 3]) +
//           4 * (smoothCh1Img[(i)*width + j + 3] -
//                smoothCh1Img[(i)*width + j - 3]) +
//           4 * (smoothCh1Img[(i + 1) * width + j + 3] -
//                smoothCh1Img[(i + 1) * width + j - 3]) +
//           4 * (smoothCh1Img[(i + 2) * width + j + 3] -
//                smoothCh1Img[(i + 2) * width + j - 3]) +
//           4 * (smoothCh1Img[(i + 3) * width + j + 3] -
//                smoothCh1Img[(i + 3) * width + j - 3]));

//       int gyCh1 = abs((smoothCh1Img[(i + 1) * width + j - 3] -
//                        smoothCh1Img[(i - 1) * width + j - 3]) +
//                       (smoothCh1Img[(i + 1) * width + j - 2] -
//                        smoothCh1Img[(i - 1) * width + j - 2]) +
//                       (smoothCh1Img[(i + 1) * width + j - 1] -
//                        smoothCh1Img[(i - 1) * width + j - 1]) +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]) +
//                       (smoothCh1Img[(i + 1) * width + j + 1] -
//                        smoothCh1Img[(i - 1) * width + j + 1]) +
//                       (smoothCh1Img[(i + 1) * width + j + 2] -
//                        smoothCh1Img[(i - 1) * width + j + 2]) +
//                       (smoothCh1Img[(i + 1) * width + j + 3] -
//                        smoothCh1Img[(i - 1) * width + j + 3]) +

//                       2 * (smoothCh1Img[(i + 2) * width + j - 3] -
//                            smoothCh1Img[(i - 2) * width + j - 3]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j - 2] -
//                            smoothCh1Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j - 1] -
//                            smoothCh1Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j] -
//                            smoothCh1Img[(i - 2) * width + j]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 1] -
//                            smoothCh1Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                            smoothCh1Img[(i - 2) * width + j + 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 3] -
//                            smoothCh1Img[(i - 2) * width + j + 3]) +

//                       4 * (smoothCh1Img[(i + 3) * width + j - 3] -
//                            smoothCh1Img[(i - 3) * width + j - 3]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j - 2] -
//                            smoothCh1Img[(i - 3) * width + j - 2]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j - 1] -
//                            smoothCh1Img[(i - 3) * width + j - 1]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j] -
//                            smoothCh1Img[(i - 3) * width + j]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j + 1] -
//                            smoothCh1Img[(i - 3) * width + j + 1]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j + 2] -
//                            smoothCh1Img[(i - 3) * width + j + 2]) +
//                       4 * (smoothCh1Img[(i + 3) * width + j + 3] -
//                            smoothCh1Img[(i - 3) * width + j + 3]));

//       // Prewitt for channel2
//       int gxCh2 = abs(
//           (smoothCh2Img[(i - 3) * width + j + 1] -
//            smoothCh2Img[(i - 3) * width + j - 1]) +
//           (smoothCh2Img[(i - 2) * width + j + 1] -
//            smoothCh2Img[(i - 2) * width + j - 1]) +
//           (smoothCh2Img[(i - 1) * width + j + 1] -
//            smoothCh2Img[(i - 1) * width + j - 1]) +
//           (smoothCh2Img[(i)*width + j + 1] - smoothCh2Img[(i)*width + j - 1]) +
//           (smoothCh2Img[(i + 1) * width + j + 1] -
//            smoothCh2Img[(i + 1) * width + j - 1]) +
//           (smoothCh2Img[(i + 2) * width + j + 1] -
//            smoothCh2Img[(i + 2) * width + j - 1]) +
//           (smoothCh2Img[(i + 3) * width + j + 1] -
//            smoothCh2Img[(i + 3) * width + j - 1]) +

//           2 * (smoothCh2Img[(i - 3) * width + j + 2] -
//                smoothCh2Img[(i - 3) * width + j - 2]) +
//           2 * (smoothCh2Img[(i - 2) * width + j + 2] -
//                smoothCh2Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh2Img[(i - 1) * width + j + 2] -
//                smoothCh2Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i)*width + j + 2] -
//                smoothCh2Img[(i)*width + j - 2]) +
//           2 * (smoothCh2Img[(i + 1) * width + j + 2] -
//                smoothCh2Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                smoothCh2Img[(i + 2) * width + j - 2]) +
//           2 * (smoothCh2Img[(i + 3) * width + j + 2] -
//                smoothCh2Img[(i + 3) * width + j - 2]) +

//           4 * (smoothCh2Img[(i - 3) * width + j + 3] -
//                smoothCh2Img[(i - 3) * width + j - 3]) +
//           4 * (smoothCh2Img[(i - 2) * width + j + 3] -
//                smoothCh2Img[(i - 2) * width + j - 3]) +
//           4 * (smoothCh2Img[(i - 1) * width + j + 3] -
//                smoothCh2Img[(i - 1) * width + j - 3]) +
//           4 * (smoothCh2Img[(i)*width + j + 3] -
//                smoothCh2Img[(i)*width + j - 3]) +
//           4 * (smoothCh2Img[(i + 1) * width + j + 3] -
//                smoothCh2Img[(i + 1) * width + j - 3]) +
//           4 * (smoothCh2Img[(i + 2) * width + j + 3] -
//                smoothCh2Img[(i + 2) * width + j - 3]) +
//           4 * (smoothCh2Img[(i + 3) * width + j + 3] -
//                smoothCh2Img[(i + 3) * width + j - 3]));

//       int gyCh2 = abs((smoothCh2Img[(i + 1) * width + j - 3] -
//                        smoothCh2Img[(i - 1) * width + j - 3]) +
//                       (smoothCh2Img[(i + 1) * width + j - 2] -
//                        smoothCh2Img[(i - 1) * width + j - 2]) +
//                       (smoothCh2Img[(i + 1) * width + j - 1] -
//                        smoothCh2Img[(i - 1) * width + j - 1]) +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]) +
//                       (smoothCh2Img[(i + 1) * width + j + 1] -
//                        smoothCh2Img[(i - 1) * width + j + 1]) +
//                       (smoothCh2Img[(i + 1) * width + j + 2] -
//                        smoothCh2Img[(i - 1) * width + j + 2]) +
//                       (smoothCh2Img[(i + 1) * width + j + 3] -
//                        smoothCh2Img[(i - 1) * width + j + 3]) +

//                       2 * (smoothCh2Img[(i + 2) * width + j - 3] -
//                            smoothCh2Img[(i - 2) * width + j - 3]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j - 2] -
//                            smoothCh2Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j - 1] -
//                            smoothCh2Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j] -
//                            smoothCh2Img[(i - 2) * width + j]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 1] -
//                            smoothCh2Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                            smoothCh2Img[(i - 2) * width + j + 2]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 3] -
//                            smoothCh2Img[(i - 2) * width + j + 3]) +

//                       4 * (smoothCh2Img[(i + 3) * width + j - 3] -
//                            smoothCh2Img[(i - 3) * width + j - 3]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j - 2] -
//                            smoothCh2Img[(i - 3) * width + j - 2]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j - 1] -
//                            smoothCh2Img[(i - 3) * width + j - 1]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j] -
//                            smoothCh2Img[(i - 3) * width + j]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j + 1] -
//                            smoothCh2Img[(i - 3) * width + j + 1]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j + 2] -
//                            smoothCh2Img[(i - 3) * width + j + 2]) +
//                       4 * (smoothCh2Img[(i + 3) * width + j + 3] -
//                            smoothCh2Img[(i - 3) * width + j + 3]));

//       // Prewitt for channel3
//       int gxCh3 = abs(
//           (smoothCh3Img[(i - 3) * width + j + 1] -
//            smoothCh3Img[(i - 3) * width + j - 1]) +
//           (smoothCh3Img[(i - 2) * width + j + 1] -
//            smoothCh3Img[(i - 2) * width + j - 1]) +
//           (smoothCh3Img[(i - 1) * width + j + 1] -
//            smoothCh3Img[(i - 1) * width + j - 1]) +
//           (smoothCh3Img[(i)*width + j + 1] - smoothCh3Img[(i)*width + j - 1]) +
//           (smoothCh3Img[(i + 1) * width + j + 1] -
//            smoothCh3Img[(i + 1) * width + j - 1]) +
//           (smoothCh3Img[(i + 2) * width + j + 1] -
//            smoothCh3Img[(i + 2) * width + j - 1]) +
//           (smoothCh3Img[(i + 3) * width + j + 1] -
//            smoothCh3Img[(i + 3) * width + j - 1]) +

//           2 * (smoothCh3Img[(i - 3) * width + j + 2] -
//                smoothCh3Img[(i - 3) * width + j - 2]) +
//           2 * (smoothCh3Img[(i - 2) * width + j + 2] -
//                smoothCh3Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh3Img[(i - 1) * width + j + 2] -
//                smoothCh3Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i)*width + j + 2] -
//                smoothCh3Img[(i)*width + j - 2]) +
//           2 * (smoothCh3Img[(i + 1) * width + j + 2] -
//                smoothCh3Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                smoothCh3Img[(i + 2) * width + j - 2]) +
//           2 * (smoothCh3Img[(i + 3) * width + j + 2] -
//                smoothCh3Img[(i + 3) * width + j - 2]) +

//           4 * (smoothCh3Img[(i - 3) * width + j + 3] -
//                smoothCh3Img[(i - 3) * width + j - 3]) +
//           4 * (smoothCh3Img[(i - 2) * width + j + 3] -
//                smoothCh3Img[(i - 2) * width + j - 3]) +
//           4 * (smoothCh3Img[(i - 1) * width + j + 3] -
//                smoothCh3Img[(i - 1) * width + j - 3]) +
//           4 * (smoothCh3Img[(i)*width + j + 3] -
//                smoothCh3Img[(i)*width + j - 3]) +
//           4 * (smoothCh3Img[(i + 1) * width + j + 3] -
//                smoothCh3Img[(i + 1) * width + j - 3]) +
//           4 * (smoothCh3Img[(i + 2) * width + j + 3] -
//                smoothCh3Img[(i + 2) * width + j - 3]) +
//           4 * (smoothCh3Img[(i + 3) * width + j + 3] -
//                smoothCh3Img[(i + 3) * width + j - 3]));

//       int gyCh3 = abs((smoothCh3Img[(i + 1) * width + j - 3] -
//                        smoothCh3Img[(i - 1) * width + j - 3]) +
//                       (smoothCh3Img[(i + 1) * width + j - 2] -
//                        smoothCh3Img[(i - 1) * width + j - 2]) +
//                       (smoothCh3Img[(i + 1) * width + j - 1] -
//                        smoothCh3Img[(i - 1) * width + j - 1]) +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]) +
//                       (smoothCh3Img[(i + 1) * width + j + 1] -
//                        smoothCh3Img[(i - 1) * width + j + 1]) +
//                       (smoothCh3Img[(i + 1) * width + j + 2] -
//                        smoothCh3Img[(i - 1) * width + j + 2]) +
//                       (smoothCh3Img[(i + 1) * width + j + 3] -
//                        smoothCh3Img[(i - 1) * width + j + 3]) +

//                       2 * (smoothCh3Img[(i + 2) * width + j - 3] -
//                            smoothCh3Img[(i - 2) * width + j - 3]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j - 2] -
//                            smoothCh3Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j - 1] -
//                            smoothCh3Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j] -
//                            smoothCh3Img[(i - 2) * width + j]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 1] -
//                            smoothCh3Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                            smoothCh3Img[(i - 2) * width + j + 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 3] -
//                            smoothCh3Img[(i - 2) * width + j + 3]) +

//                       4 * (smoothCh3Img[(i + 3) * width + j - 3] -
//                            smoothCh3Img[(i - 3) * width + j - 3]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j - 2] -
//                            smoothCh3Img[(i - 3) * width + j - 2]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j - 1] -
//                            smoothCh3Img[(i - 3) * width + j - 1]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j] -
//                            smoothCh3Img[(i - 3) * width + j]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j + 1] -
//                            smoothCh3Img[(i - 3) * width + j + 1]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j + 2] -
//                            smoothCh3Img[(i - 3) * width + j + 2]) +
//                       4 * (smoothCh3Img[(i + 3) * width + j + 3] -
//                            smoothCh3Img[(i - 3) * width + j + 3]));

//       // Combine the gradients
//       int gx, gy, grad;

//       // L1-norm
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       grad = gx + gy;

//       //      gx = (int)(sqrt((double)gxCh1*gxCh1 + gxCh2*gxCh2 +
//       //      gxCh3*gxCh3)+0.5); gy = (int)(sqrt((double)gyCh1*gyCh1 +
//       //      gyCh2*gyCh2 + gyCh3*gyCh3)+0.5);
//       //     grad = (int)(sqrt((double)gx*gx+gy*gy)+0.5);

//       if (grad > max) max = grad;
//       gradImg[i * width + j] = grad;

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapByPrewitt7x7

// ///--------------------------------------------------------------------------------------------------------------------
// /// 3x3 Sobel -- scaled to [0-255]
// ///
// void ComputeGradientMapBySobel(unsigned char *smoothCh1Img,
//                                unsigned char *smoothCh2Img,
//                                unsigned char *smoothCh3Img, short *gradImg,
//                                unsigned char *dirImg, int width, int height) {
//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = 0;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = 0;
//   }

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Sobel for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 = abs(com1 + com2 +
//                       2 * (smoothCh1Img[i * width + j + 1] -
//                            smoothCh1Img[i * width + j - 1]));
//       int gyCh1 = abs(com1 - com2 +
//                       2 * (smoothCh1Img[(i + 1) * width + j] -
//                            smoothCh1Img[(i - 1) * width + j]));

//       // Sobel for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 = abs(com1 + com2 +
//                       2 * (smoothCh2Img[i * width + j + 1] -
//                            smoothCh2Img[i * width + j - 1]));
//       int gyCh2 = abs(com1 - com2 +
//                       2 * (smoothCh2Img[(i + 1) * width + j] -
//                            smoothCh2Img[(i - 1) * width + j]));

//       // Sobel for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 = abs(com1 + com2 +
//                       2 * (smoothCh3Img[i * width + j + 1] -
//                            smoothCh3Img[i * width + j - 1]));
//       int gyCh3 = abs(com1 - com2 +
//                       2 * (smoothCh3Img[(i + 1) * width + j] -
//                            smoothCh3Img[(i - 1) * width + j]));

//       // Combine the gradients
//       int gx, gy, grad;

// #if 0
//       // L1-Norm: |G1| + |G2| + |G3|
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       grad = gx+gy;

//       if (gx > gy) dirImg[i*width+j] = EDGE_VERTICAL;
//       else         dirImg[i*width+j] = EDGE_HORIZONTAL;
// #elif 1
//       // L2-Norm: {|G1|^2 + |G2|^2 + |G3|^2}^0.5
//       gx = (int)(sqrt((double)gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3) +
//                  0.5);
//       gy = (int)(sqrt((double)gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3) +
//                  0.5);
//       //      grad = gx+gy;
//       grad = (int)(sqrt((double)gx * gx + gy * gy) + 0.5);

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
// #else
//       // L-Infinity-Norm: Max{|G1|, |G2|, |G3|}
//       gx = gxCh1;
//       if (gxCh2 > gx) gx = gxCh2;
//       if (gxCh3 > gx) gx = gxCh3;
//       gy = gyCh1;
//       if (gyCh2 > gy) gy = gyCh2;
//       if (gyCh3 > gy) gy = gyCh3;
//       grad = gx + gy;

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
// #endif

//       gradImg[i * width + j] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapBySobel

// ///---------------------------------------------------------------------------------------------------------------
// /// 5x5 Sobel -- scaled to [0-255]
// ///
// void ComputeGradientMapBySobel5x5(unsigned char *smoothCh1Img,
//                                   unsigned char *smoothCh2Img,
//                                   unsigned char *smoothCh3Img, short *gradImg,
//                                   unsigned char *dirImg, int width,
//                                   int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 2; i < height - 2; i++) {
//     for (int j = 2; j < width - 2; j++) {
//       // Prewitt for channel1
//       int gxCh1 = abs((smoothCh1Img[(i - 2) * width + j + 1] -
//                        smoothCh1Img[(i - 2) * width + j - 1]) +
//                       (smoothCh1Img[(i - 1) * width + j + 1] -
//                        smoothCh1Img[(i - 1) * width + j - 1]) +
//                       2 * (smoothCh1Img[(i)*width + j + 1] -
//                            smoothCh1Img[(i)*width + j - 1]) +
//                       (smoothCh1Img[(i + 1) * width + j + 1] -
//                        smoothCh1Img[(i + 1) * width + j - 1]) +
//                       (smoothCh1Img[(i + 2) * width + j + 1] -
//                        smoothCh1Img[(i + 2) * width + j - 1]) +

//                       2 * (smoothCh1Img[(i - 2) * width + j + 2] -
//                            smoothCh1Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i - 1) * width + j + 2] -
//                            smoothCh1Img[(i - 1) * width + j - 2]) +
//                       4 * (smoothCh1Img[(i)*width + j + 2] -
//                            smoothCh1Img[(i)*width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 1) * width + j + 2] -
//                            smoothCh1Img[(i + 1) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                            smoothCh1Img[(i + 2) * width + j - 2]));

//       int gyCh1 = abs((smoothCh1Img[(i + 1) * width + j - 2] -
//                        smoothCh1Img[(i - 1) * width + j - 2]) +
//                       (smoothCh1Img[(i + 1) * width + j - 1] -
//                        smoothCh1Img[(i - 1) * width + j - 1]) +
//                       2 * (smoothCh1Img[(i + 1) * width + j] -
//                            smoothCh1Img[(i - 1) * width + j]) +
//                       (smoothCh1Img[(i + 1) * width + j + 1] -
//                        smoothCh1Img[(i - 1) * width + j + 1]) +
//                       (smoothCh1Img[(i + 1) * width + j + 2] -
//                        smoothCh1Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh1Img[(i + 2) * width + j - 2] -
//                            smoothCh1Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j - 1] -
//                            smoothCh1Img[(i - 2) * width + j - 1]) +
//                       4 * (smoothCh1Img[(i + 2) * width + j] -
//                            smoothCh1Img[(i - 2) * width + j]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 1] -
//                            smoothCh1Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                            smoothCh1Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel2
//       int gxCh2 = abs(
//           (smoothCh2Img[(i - 2) * width + j + 1] -
//            smoothCh2Img[(i - 2) * width + j - 1]) +
//           (smoothCh2Img[(i - 1) * width + j + 1] -
//            smoothCh2Img[(i - 1) * width + j - 1]) +
//           (smoothCh2Img[(i)*width + j + 1] - smoothCh2Img[(i)*width + j - 1]) +
//           2 * (smoothCh2Img[(i + 1) * width + j + 1] -
//                smoothCh2Img[(i + 1) * width + j - 1]) +
//           (smoothCh2Img[(i + 2) * width + j + 1] -
//            smoothCh2Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh2Img[(i - 2) * width + j + 2] -
//                smoothCh2Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh2Img[(i - 1) * width + j + 2] -
//                smoothCh2Img[(i - 1) * width + j - 2]) +
//           4 * (smoothCh2Img[(i)*width + j + 2] -
//                smoothCh2Img[(i)*width + j - 2]) +
//           2 * (smoothCh2Img[(i + 1) * width + j + 2] -
//                smoothCh2Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                smoothCh2Img[(i + 2) * width + j - 2]));

//       int gyCh2 = abs((smoothCh2Img[(i + 1) * width + j - 2] -
//                        smoothCh2Img[(i - 1) * width + j - 2]) +
//                       (smoothCh2Img[(i + 1) * width + j - 1] -
//                        smoothCh2Img[(i - 1) * width + j - 1]) +
//                       2 * (smoothCh2Img[(i + 1) * width + j] -
//                            smoothCh2Img[(i - 1) * width + j]) +
//                       (smoothCh2Img[(i + 1) * width + j + 1] -
//                        smoothCh2Img[(i - 1) * width + j + 1]) +
//                       (smoothCh2Img[(i + 1) * width + j + 2] -
//                        smoothCh2Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh2Img[(i + 2) * width + j - 2] -
//                            smoothCh2Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j - 1] -
//                            smoothCh2Img[(i - 2) * width + j - 1]) +
//                       4 * (smoothCh2Img[(i + 2) * width + j] -
//                            smoothCh2Img[(i - 2) * width + j]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 1] -
//                            smoothCh2Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                            smoothCh2Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel3
//       int gxCh3 = abs((smoothCh3Img[(i - 2) * width + j + 1] -
//                        smoothCh3Img[(i - 2) * width + j - 1]) +
//                       (smoothCh3Img[(i - 1) * width + j + 1] -
//                        smoothCh3Img[(i - 1) * width + j - 1]) +
//                       2 * (smoothCh3Img[(i)*width + j + 1] -
//                            smoothCh3Img[(i)*width + j - 1]) +
//                       (smoothCh3Img[(i + 1) * width + j + 1] -
//                        smoothCh3Img[(i + 1) * width + j - 1]) +
//                       (smoothCh3Img[(i + 2) * width + j + 1] -
//                        smoothCh3Img[(i + 2) * width + j - 1]) +

//                       2 * (smoothCh3Img[(i - 2) * width + j + 2] -
//                            smoothCh3Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i - 1) * width + j + 2] -
//                            smoothCh3Img[(i - 1) * width + j - 2]) +
//                       4 * (smoothCh3Img[(i)*width + j + 2] -
//                            smoothCh3Img[(i)*width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 1) * width + j + 2] -
//                            smoothCh3Img[(i + 1) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                            smoothCh3Img[(i + 2) * width + j - 2]));

//       int gyCh3 = abs((smoothCh3Img[(i + 1) * width + j - 2] -
//                        smoothCh3Img[(i - 1) * width + j - 2]) +
//                       (smoothCh3Img[(i + 1) * width + j - 1] -
//                        smoothCh3Img[(i - 1) * width + j - 1]) +
//                       2 * (smoothCh3Img[(i + 1) * width + j] -
//                            smoothCh3Img[(i - 1) * width + j]) +
//                       (smoothCh3Img[(i + 1) * width + j + 1] -
//                        smoothCh3Img[(i - 1) * width + j + 1]) +
//                       (smoothCh3Img[(i + 1) * width + j + 2] -
//                        smoothCh3Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh3Img[(i + 2) * width + j - 2] -
//                            smoothCh3Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j - 1] -
//                            smoothCh3Img[(i - 2) * width + j - 1]) +
//                       4 * (smoothCh3Img[(i + 2) * width + j] -
//                            smoothCh3Img[(i - 2) * width + j]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 1] -
//                            smoothCh3Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                            smoothCh3Img[(i - 2) * width + j + 2]));

//       // Combine the gradients
//       int gx, gy, grad;

//       // L2-norm
//       gx = (int)(sqrt((double)gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3) +
//                  0.5);
//       gy = (int)(sqrt((double)gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3) +
//                  0.5);
//       grad = (int)(sqrt((double)gx * gx + gy * gy) + 0.5);

//       if (grad > max) max = grad;
//       gradImg[i * width + j] = grad;

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient value to [0-255]
//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++) {
//     gradImg[i] = (short)(gradImg[i] / scale);
//   }  // end-for
// }  // end-ComputeGradientMapBySobel5x5

// ///------------------------------------------------------------------------------------
// /// MyCompass: Grayscale image
// ///
// void ComputeGradientMapByMyCompass4Dirs(unsigned char *smoothCh1Img,
//                                         unsigned char *smoothCh2Img,
//                                         unsigned char *smoothCh3Img,
//                                         short *gradImg, unsigned char *dirImg,
//                                         int width, int height) {
//   int *tmpGradImg = new int[width * height];
//   memset(tmpGradImg, 0, sizeof(int) * width * height);

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       int maxG = 0;
//       int maxAngle = 0;
//       int G;

// #if 1
//       // Prewitt
//       // 0 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j - 1] +
//               smoothCh1Img[(i - 1) * width + j] +
//               smoothCh1Img[(i - 1) * width + j + 1] -
//               smoothCh1Img[(i + 1) * width + j - 1] -
//               smoothCh1Img[(i + 1) * width + j] -
//               smoothCh1Img[(i + 1) * width + j + 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j - 1] +
//               smoothCh2Img[(i - 1) * width + j] +
//               smoothCh2Img[(i - 1) * width + j + 1] -
//               smoothCh2Img[(i + 1) * width + j - 1] -
//               smoothCh2Img[(i + 1) * width + j] -
//               smoothCh2Img[(i + 1) * width + j + 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j - 1] +
//               smoothCh3Img[(i - 1) * width + j] +
//               smoothCh3Img[(i - 1) * width + j + 1] -
//               smoothCh3Img[(i + 1) * width + j - 1] -
//               smoothCh3Img[(i + 1) * width + j] -
//               smoothCh3Img[(i + 1) * width + j + 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 0;
//       }

//       // 45 degrees
//       G = abs(smoothCh1Img[i * width + j + 1] +
//               smoothCh1Img[(i + 1) * width + j + 1] +
//               smoothCh1Img[(i + 1) * width + j] -
//               smoothCh1Img[i * width + j - 1] -
//               smoothCh1Img[(i - 1) * width + j - 1] -
//               smoothCh1Img[(i - 1) * width + j]) +
//           abs(smoothCh2Img[i * width + j + 1] +
//               smoothCh2Img[(i + 1) * width + j + 1] +
//               smoothCh2Img[(i + 1) * width + j] -
//               smoothCh2Img[i * width + j - 1] -
//               smoothCh2Img[(i - 1) * width + j - 1] -
//               smoothCh2Img[(i - 1) * width + j]) +
//           abs(smoothCh3Img[i * width + j + 1] +
//               smoothCh3Img[(i + 1) * width + j + 1] +
//               smoothCh3Img[(i + 1) * width + j] -
//               smoothCh3Img[i * width + j - 1] -
//               smoothCh3Img[(i - 1) * width + j - 1] -
//               smoothCh3Img[(i - 1) * width + j]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 45;
//       }

//       // 90 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j + 1] +
//               smoothCh1Img[i * width + j + 1] +
//               smoothCh1Img[(i + 1) * width + j + 1] -
//               smoothCh1Img[(i - 1) * width + j - 1] -
//               smoothCh1Img[i * width + j - 1] -
//               smoothCh1Img[(i + 1) * width + j - 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j + 1] +
//               smoothCh2Img[i * width + j + 1] +
//               smoothCh2Img[(i + 1) * width + j + 1] -
//               smoothCh2Img[(i - 1) * width + j - 1] -
//               smoothCh2Img[i * width + j - 1] -
//               smoothCh2Img[(i + 1) * width + j - 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j + 1] +
//               smoothCh3Img[i * width + j + 1] +
//               smoothCh3Img[(i + 1) * width + j + 1] -
//               smoothCh3Img[(i - 1) * width + j - 1] -
//               smoothCh3Img[i * width + j - 1] -
//               smoothCh3Img[(i + 1) * width + j - 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 90;
//       }

//       // 135 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j] +
//               smoothCh1Img[(i - 1) * width + j + 1] +
//               smoothCh1Img[i * width + j + 1] -
//               smoothCh1Img[(i + 1) * width + j] -
//               smoothCh1Img[(i + 1) * width + j - 1] -
//               smoothCh1Img[i * width + j - 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j] +
//               smoothCh2Img[(i - 1) * width + j + 1] +
//               smoothCh2Img[i * width + j + 1] -
//               smoothCh2Img[(i + 1) * width + j] -
//               smoothCh2Img[(i + 1) * width + j - 1] -
//               smoothCh2Img[i * width + j - 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j] +
//               smoothCh3Img[(i - 1) * width + j + 1] +
//               smoothCh3Img[i * width + j + 1] -
//               smoothCh3Img[(i + 1) * width + j] -
//               smoothCh3Img[(i + 1) * width + j - 1] -
//               smoothCh3Img[i * width + j - 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 135;
//       }
// #else
//       // Sobel
//       // 0 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j - 1] +
//               2 * smoothCh1Img[(i - 1) * width + j] +
//               smoothCh1Img[(i - 1) * width + j + 1] -
//               smoothCh1Img[(i + 1) * width + j - 1] -
//               2 * smoothCh1Img[(i + 1) * width + j] -
//               smoothCh1Img[(i + 1) * width + j + 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j - 1] +
//               2 * smoothCh2Img[(i - 1) * width + j] +
//               smoothCh2Img[(i - 1) * width + j + 1] -
//               smoothCh2Img[(i + 1) * width + j - 1] -
//               2 * smoothCh2Img[(i + 1) * width + j] -
//               smoothCh2Img[(i + 1) * width + j + 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j - 1] +
//               2 * smoothCh3Img[(i - 1) * width + j] +
//               smoothCh3Img[(i - 1) * width + j + 1] -
//               smoothCh3Img[(i + 1) * width + j - 1] -
//               2 * smoothCh3Img[(i + 1) * width + j] -
//               smoothCh3Img[(i + 1) * width + j + 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 0;
//       }

//       // 45 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j] +
//               2 * smoothCh1Img[(i - 1) * width + j - 1] +
//               smoothCh1Img[i * width + j - 1] -
//               smoothCh1Img[(i + 1) * width + j] -
//               2 * smoothCh1Img[(i + 1) * width + j + 1] -
//               smoothCh1Img[i * width + j + 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j] +
//               2 * smoothCh2Img[(i - 1) * width + j - 1] +
//               smoothCh2Img[i * width + j - 1] -
//               smoothCh2Img[(i + 1) * width + j] -
//               2 * smoothCh2Img[(i + 1) * width + j + 1] -
//               smoothCh2Img[i * width + j + 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j] +
//               2 * smoothCh3Img[(i - 1) * width + j - 1] +
//               smoothCh3Img[i * width + j - 1] -
//               smoothCh3Img[(i + 1) * width + j] -
//               2 * smoothCh3Img[(i + 1) * width + j + 1] -
//               smoothCh3Img[i * width + j + 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 45;
//       }

//       // 90 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j + 1] +
//               2 * smoothCh1Img[i * width + j + 1] +
//               smoothCh1Img[(i + 1) * width + j + 1] -
//               smoothCh1Img[(i - 1) * width + j - 1] -
//               2 * smoothCh1Img[i * width + j - 1] -
//               smoothCh1Img[(i + 1) * width + j - 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j + 1] +
//               2 * smoothCh2Img[i * width + j + 1] +
//               smoothCh2Img[(i + 1) * width + j + 1] -
//               smoothCh2Img[(i - 1) * width + j - 1] -
//               2 * smoothCh2Img[i * width + j - 1] -
//               smoothCh2Img[(i + 1) * width + j - 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j + 1] +
//               2 * smoothCh3Img[i * width + j + 1] +
//               smoothCh3Img[(i + 1) * width + j + 1] -
//               smoothCh3Img[(i - 1) * width + j - 1] -
//               2 * smoothCh3Img[i * width + j - 1] -
//               smoothCh3Img[(i + 1) * width + j - 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 90;
//       }

//       // 135 degrees
//       G = abs(smoothCh1Img[(i - 1) * width + j] +
//               2 * smoothCh1Img[(i - 1) * width + j + 1] +
//               smoothCh1Img[i * width + j + 1] -
//               smoothCh1Img[(i + 1) * width + j] -
//               2 * smoothCh1Img[(i + 1) * width + j - 1] -
//               smoothCh1Img[i * width + j - 1]) +
//           abs(smoothCh2Img[(i - 1) * width + j] +
//               2 * smoothCh2Img[(i - 1) * width + j + 1] +
//               smoothCh2Img[i * width + j + 1] -
//               smoothCh2Img[(i + 1) * width + j] -
//               2 * smoothCh2Img[(i + 1) * width + j - 1] -
//               smoothCh2Img[i * width + j - 1]) +
//           abs(smoothCh3Img[(i - 1) * width + j] +
//               2 * smoothCh3Img[(i - 1) * width + j + 1] +
//               smoothCh3Img[i * width + j + 1] -
//               smoothCh3Img[(i + 1) * width + j] -
//               2 * smoothCh3Img[(i + 1) * width + j - 1] -
//               smoothCh3Img[i * width + j - 1]);
//       if (G > maxG) {
//         maxG = G;
//         maxAngle = 135;
//       }
// #endif

//       tmpGradImg[i * width + j] = maxG;
//       if (maxG > max) max = maxG;

//         // 4 directions
// #if 1
//       if (maxAngle == 0)
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
//       else if (maxAngle == 45)
//         dirImg[i * width + j] = EDGE_45;
//       else if (maxAngle == 90)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_135;
// #else
//       if (maxAngle == 0 || maxAngle == 45)
//         dirImg[i * width + j] = EDGE_HORIZONTAL;
//       else
//         dirImg[i * width + j] = EDGE_VERTICAL;
// #endif
//     }  // end-for
//   }    // end-for

//   double scale = max / 255.0;
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(tmpGradImg[i] / scale);

//   delete tmpGradImg;
// }  // end-ComputeGradientMapByMyCompass4Dirs

// ///-----------------------------------------------------------------------------------------------------------------------------------
// /// Compute color image gradient
// ///
// void ComputeGradientMapByPrewitt(IplImage *smoothImg, short *gradImg,
//                                  unsigned char *dirImg, int GRADIENT_THRESH) {
//   if (smoothImg->nChannels != 3) return;

//   int width = smoothImg->width;
//   int height = smoothImg->height;

//   // Initialize gradient image for row = 0, row = height-1, column=0,
//   // column=width-1
//   for (int j = 0; j < width; j++) {
//     gradImg[j] = gradImg[(height - 1) * width + j] = GRADIENT_THRESH - 1;
//   }
//   for (int i = 1; i < height - 1; i++) {
//     gradImg[i * width] = gradImg[(i + 1) * width - 1] = GRADIENT_THRESH - 1;
//   }

//   for (int i = 1; i < height - 1; i++) {
//     unsigned char *prevLine = (unsigned char *)(smoothImg->imageData +
//                                                 (i - 1) * smoothImg->widthStep);
//     unsigned char *curLine =
//         (unsigned char *)(smoothImg->imageData + i * smoothImg->widthStep);
//     unsigned char *nextLine = (unsigned char *)(smoothImg->imageData +
//                                                 (i + 1) * smoothImg->widthStep);

//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = nextLine[(j + 1) * 3] - prevLine[(j - 1) * 3];
//       int com2 = prevLine[(j + 1) * 3] - nextLine[(j - 1) * 3];

//       int gxCh1 =
//           abs(com1 + com2 + (curLine[(j + 1) * 3] - curLine[(j - 1) * 3]));
//       int gyCh1 = abs(com1 - com2 + (nextLine[j * 3] - prevLine[j * 3]));
//       int gradCh1 = gxCh1 + gyCh1;

//       /// Channel 2
//       com1 = nextLine[(j + 1) * 3 + 1] - prevLine[(j - 1) * 3 + 1];
//       com2 = prevLine[(j + 1) * 3 + 1] - nextLine[(j - 1) * 3 + 1];

//       int gxCh2 = abs(com1 + com2 +
//                       (curLine[(j + 1) * 3 + 1] - curLine[(j - 1) * 3 + 1]));
//       int gyCh2 =
//           abs(com1 - com2 + (nextLine[j * 3 + 1] - prevLine[j * 3 + 1]));
//       int gradCh2 = gxCh2 + gyCh2;

//       /// Channel 3
//       com1 = nextLine[(j + 1) * 3 + 2] - prevLine[(j - 1) * 3 + 2];
//       com2 = prevLine[(j + 1) * 3 + 2] - nextLine[(j - 1) * 3 + 2];

//       int gxCh3 = abs(com1 + com2 +
//                       (curLine[(j + 1) * 3 + 2] - curLine[(j - 1) * 3 + 2]));
//       int gyCh3 =
//           abs(com1 - com2 + (nextLine[j * 3 + 2] - prevLine[j * 3 + 2]));
//       int gradCh3 = gxCh3 + gyCh3;

//       // Combine the gradients
//       int gx, gy, grad;

// #if 1
//       // Take the average (Avg)
//       gx = (gxCh1 + gxCh2 + gxCh3 + 2) / 3;
//       gy = (gyCh1 + gyCh2 + gyCh3 + 2) / 3;
//       grad = (gradCh1 + gradCh2 + gradCh3 + 2) / 3;

// #else
//       // Sum
//       gx = gxCh1 + gxCh2 + gxCh3;
//       gy = gyCh1 + gyCh2 + gyCh3;
//       grad = gradCh1 + gradCh2 + gradCh3;
// #endif

//       if (gx > gy)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;

//       gradImg[i * width + j] = grad;
//     }  // end-for
//   }    // end-for
// }  // end-ComputeGradientMapByPrewitt

// ///-----------------------------------------------------------------------------------------------------------------
// /// Color image gradientby DiZenzo's method
// ///
// void ComputeGradientMapByDiZenzo(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 =
//           com1 + com2 +
//           (smoothCh1Img[i * width + j + 1] - smoothCh1Img[i * width + j - 1]);
//       int gyCh1 = com1 - com2 +
//                   (smoothCh1Img[(i + 1) * width + j] -
//                    smoothCh1Img[(i - 1) * width + j]);

//       // Prewitt for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 =
//           com1 + com2 +
//           (smoothCh2Img[i * width + j + 1] - smoothCh2Img[i * width + j - 1]);
//       int gyCh2 = com1 - com2 +
//                   (smoothCh2Img[(i + 1) * width + j] -
//                    smoothCh2Img[(i - 1) * width + j]);

//       // Prewitt for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 =
//           com1 + com2 +
//           (smoothCh3Img[i * width + j + 1] - smoothCh3Img[i * width + j - 1]);
//       int gyCh3 = com1 - com2 +
//                   (smoothCh3Img[(i + 1) * width + j] -
//                    smoothCh3Img[(i - 1) * width + j]);

//       int gxx = gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3;
//       int gyy = gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3;
//       int gxy = gxCh1 * gyCh1 + gxCh2 * gyCh2 + gxCh3 * gyCh3;

// #if 1
//       // Di Zenzo's formulas from Gonzales & Woods - Page 337
//       double theta =
//           atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;  // Gradient Direction
//       int grad = (int)(sqrt(((gxx + gyy) + (gxx - gyy) * cos(2 * theta) +
//                              2 * gxy * sin(2 * theta)) /
//                             2.0) +
//                        0.5);  // Gradient Magnitude

// #else
//       // Koschan & Abidi - 2005 - Signal Processing Magazine
//       double theta =
//           atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;  // Gradient Direction

//       double cosTheta = cos(theta);
//       double sinTheta = sin(theta);
//       int grad =
//           (int)(sqrt(gxx * cosTheta * cosTheta + 2 * gxy * sinTheta * cosTheta +
//                      gyy * sinTheta * sinTheta) +
//                 0.5);  // Gradient Magnitude
// #endif

//       // Gradient is perpendicular to the edge passing through the pixel
//       if (theta >= -3.14159 / 4 && theta <= 3.14159 / 4)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;

//       gradImg[i * width + j] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient values to 0-255
//   double scale = 255.0 / max;
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(gradImg[i] * scale);
// }  // end-ComputeGradientMapByDiZenzo

// ///-----------------------------------------------------------------------------------------------------------------
// /// Color image gradientby DiZenzo's method (4 dirs)
// ///
// void ComputeGradientMapByDiZenzo4Dirs(unsigned char *smoothCh1Img,
//                                       unsigned char *smoothCh2Img,
//                                       unsigned char *smoothCh3Img,
//                                       short *gradImg, unsigned char *dirImg,
//                                       int width, int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       // Prewitt for channel1
//       int com1 = smoothCh1Img[(i + 1) * width + j + 1] -
//                  smoothCh1Img[(i - 1) * width + j - 1];
//       int com2 = smoothCh1Img[(i - 1) * width + j + 1] -
//                  smoothCh1Img[(i + 1) * width + j - 1];

//       int gxCh1 =
//           com1 + com2 +
//           (smoothCh1Img[i * width + j + 1] - smoothCh1Img[i * width + j - 1]);
//       int gyCh1 = com1 - com2 +
//                   (smoothCh1Img[(i + 1) * width + j] -
//                    smoothCh1Img[(i - 1) * width + j]);

//       // Prewitt for channel2
//       com1 = smoothCh2Img[(i + 1) * width + j + 1] -
//              smoothCh2Img[(i - 1) * width + j - 1];
//       com2 = smoothCh2Img[(i - 1) * width + j + 1] -
//              smoothCh2Img[(i + 1) * width + j - 1];

//       int gxCh2 =
//           com1 + com2 +
//           (smoothCh2Img[i * width + j + 1] - smoothCh2Img[i * width + j - 1]);
//       int gyCh2 = com1 - com2 +
//                   (smoothCh2Img[(i + 1) * width + j] -
//                    smoothCh2Img[(i - 1) * width + j]);

//       // Prewitt for channel3
//       com1 = smoothCh3Img[(i + 1) * width + j + 1] -
//              smoothCh3Img[(i - 1) * width + j - 1];
//       com2 = smoothCh3Img[(i - 1) * width + j + 1] -
//              smoothCh3Img[(i + 1) * width + j - 1];

//       int gxCh3 =
//           com1 + com2 +
//           (smoothCh3Img[i * width + j + 1] - smoothCh3Img[i * width + j - 1]);
//       int gyCh3 = com1 - com2 +
//                   (smoothCh3Img[(i + 1) * width + j] -
//                    smoothCh3Img[(i - 1) * width + j]);

//       int gxx = gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3;
//       int gyy = gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3;
//       int gxy = gxCh1 * gyCh1 + gxCh2 * gyCh2 + gxCh3 * gyCh3;

//       // Di Zenzo's formulas from Gonzales & Woods - Page 337
//       double theta =
//           atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;  // Gradient Direction
//       int grad = (int)(sqrt(((gxx + gyy) + (gxx - gyy) * cos(2 * theta) +
//                              2 * gxy * sin(2 * theta)) /
//                             2.0) +
//                        0.5);  // Gradient Magnitude

// #define MYPI 3.14159
//       if (theta > MYPI / 2 + 0.1)
//         printf("Bigger than PI/2. theta: %6.4lf\n", theta);
//       else if (theta < -MYPI / 2 - 0.1)
//         printf("Smaller than -PI/2\n");

//       // Gradient is perpendicular to the edge passing through the pixel
//       if (theta <= (-3 * MYPI) / 8 || theta >= (3 * MYPI) / 8)
//         dirImg[i * width + j] =
//             EDGE_HORIZONTAL;  // vertical gradient, horizontal edge
//       else if (theta <= -MYPI / 8)
//         dirImg[i * width + j] =
//             EDGE_135;  // 135 degree gradient, 45 degree edge  (due to y axis
//                        // being in the opposite direction)
//       else if (theta >= MYPI / 8)
//         dirImg[i * width + j] =
//             EDGE_45;  // 45 degree gradient, 135 degree edge (due to y axis
//                       // being in the opposite direction)
//       else
//         dirImg[i * width + j] =
//             EDGE_VERTICAL;  // horizontal gradient, vertical edge

//       gradImg[i * width + j] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   //  printf("max gradient: %4.2lf\n", max);

//   // Scale the gradient values to 0-255
//   double scale = 255.0 / max;
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(gradImg[i] * scale);
// }  // end-ComputeGradientMapByDiZenzo4Dirs

// ///---------------------------------------------------------------------
// /// Color image gradientby DiZenzo's method
// ///
// void ComputeGradientMapByDiZenzo5x5(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height) {
//   memset(gradImg, 0, sizeof(short) * width * height);

//   int max = 0;
//   for (int i = 2; i < height - 2; i++) {
//     for (int j = 2; j < width - 2; j++) {
//       int gxCh1 = abs(
//           (smoothCh1Img[(i - 2) * width + j + 1] -
//            smoothCh1Img[(i - 2) * width + j - 1]) +
//           (smoothCh1Img[(i - 1) * width + j + 1] -
//            smoothCh1Img[(i - 1) * width + j - 1]) +
//           (smoothCh1Img[(i)*width + j + 1] - smoothCh1Img[(i)*width + j - 1]) +
//           (smoothCh1Img[(i + 1) * width + j + 1] -
//            smoothCh1Img[(i + 1) * width + j - 1]) +
//           (smoothCh1Img[(i + 2) * width + j + 1] -
//            smoothCh1Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh1Img[(i - 2) * width + j + 2] -
//                smoothCh1Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh1Img[(i - 1) * width + j + 2] -
//                smoothCh1Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i)*width + j + 2] -
//                smoothCh1Img[(i)*width + j - 2]) +
//           2 * (smoothCh1Img[(i + 1) * width + j + 2] -
//                smoothCh1Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                smoothCh1Img[(i + 2) * width + j - 2]));

//       int gyCh1 = abs((smoothCh1Img[(i + 1) * width + j - 2] -
//                        smoothCh1Img[(i - 1) * width + j - 2]) +
//                       (smoothCh1Img[(i + 1) * width + j - 1] -
//                        smoothCh1Img[(i - 1) * width + j - 1]) +
//                       (smoothCh1Img[(i + 1) * width + j] -
//                        smoothCh1Img[(i - 1) * width + j]) +
//                       (smoothCh1Img[(i + 1) * width + j + 1] -
//                        smoothCh1Img[(i - 1) * width + j + 1]) +
//                       (smoothCh1Img[(i + 1) * width + j + 2] -
//                        smoothCh1Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh1Img[(i + 2) * width + j - 2] -
//                            smoothCh1Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j - 1] -
//                            smoothCh1Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j] -
//                            smoothCh1Img[(i - 2) * width + j]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 1] -
//                            smoothCh1Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh1Img[(i + 2) * width + j + 2] -
//                            smoothCh1Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel2
//       int gxCh2 = abs(
//           (smoothCh2Img[(i - 2) * width + j + 1] -
//            smoothCh2Img[(i - 2) * width + j - 1]) +
//           (smoothCh2Img[(i - 1) * width + j + 1] -
//            smoothCh2Img[(i - 1) * width + j - 1]) +
//           (smoothCh2Img[(i)*width + j + 1] - smoothCh2Img[(i)*width + j - 1]) +
//           (smoothCh2Img[(i + 1) * width + j + 1] -
//            smoothCh2Img[(i + 1) * width + j - 1]) +
//           (smoothCh2Img[(i + 2) * width + j + 1] -
//            smoothCh2Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh2Img[(i - 2) * width + j + 2] -
//                smoothCh2Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh2Img[(i - 1) * width + j + 2] -
//                smoothCh2Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i)*width + j + 2] -
//                smoothCh2Img[(i)*width + j - 2]) +
//           2 * (smoothCh2Img[(i + 1) * width + j + 2] -
//                smoothCh2Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                smoothCh2Img[(i + 2) * width + j - 2]));

//       int gyCh2 = abs((smoothCh2Img[(i + 1) * width + j - 2] -
//                        smoothCh2Img[(i - 1) * width + j - 2]) +
//                       (smoothCh2Img[(i + 1) * width + j - 1] -
//                        smoothCh2Img[(i - 1) * width + j - 1]) +
//                       (smoothCh2Img[(i + 1) * width + j] -
//                        smoothCh2Img[(i - 1) * width + j]) +
//                       (smoothCh2Img[(i + 1) * width + j + 1] -
//                        smoothCh2Img[(i - 1) * width + j + 1]) +
//                       (smoothCh2Img[(i + 1) * width + j + 2] -
//                        smoothCh2Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh2Img[(i + 2) * width + j - 2] -
//                            smoothCh2Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j - 1] -
//                            smoothCh2Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j] -
//                            smoothCh2Img[(i - 2) * width + j]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 1] -
//                            smoothCh2Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh2Img[(i + 2) * width + j + 2] -
//                            smoothCh2Img[(i - 2) * width + j + 2]));

//       // Prewitt for channel3
//       int gxCh3 = abs(
//           (smoothCh3Img[(i - 2) * width + j + 1] -
//            smoothCh3Img[(i - 2) * width + j - 1]) +
//           (smoothCh3Img[(i - 1) * width + j + 1] -
//            smoothCh3Img[(i - 1) * width + j - 1]) +
//           (smoothCh3Img[(i)*width + j + 1] - smoothCh3Img[(i)*width + j - 1]) +
//           (smoothCh3Img[(i + 1) * width + j + 1] -
//            smoothCh3Img[(i + 1) * width + j - 1]) +
//           (smoothCh3Img[(i + 2) * width + j + 1] -
//            smoothCh3Img[(i + 2) * width + j - 1]) +

//           2 * (smoothCh3Img[(i - 2) * width + j + 2] -
//                smoothCh3Img[(i - 2) * width + j - 2]) +
//           2 * (smoothCh3Img[(i - 1) * width + j + 2] -
//                smoothCh3Img[(i - 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i)*width + j + 2] -
//                smoothCh3Img[(i)*width + j - 2]) +
//           2 * (smoothCh3Img[(i + 1) * width + j + 2] -
//                smoothCh3Img[(i + 1) * width + j - 2]) +
//           2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                smoothCh3Img[(i + 2) * width + j - 2]));

//       int gyCh3 = abs((smoothCh3Img[(i + 1) * width + j - 2] -
//                        smoothCh3Img[(i - 1) * width + j - 2]) +
//                       (smoothCh3Img[(i + 1) * width + j - 1] -
//                        smoothCh3Img[(i - 1) * width + j - 1]) +
//                       (smoothCh3Img[(i + 1) * width + j] -
//                        smoothCh3Img[(i - 1) * width + j]) +
//                       (smoothCh3Img[(i + 1) * width + j + 1] -
//                        smoothCh3Img[(i - 1) * width + j + 1]) +
//                       (smoothCh3Img[(i + 1) * width + j + 2] -
//                        smoothCh3Img[(i - 1) * width + j + 2]) +

//                       2 * (smoothCh3Img[(i + 2) * width + j - 2] -
//                            smoothCh3Img[(i - 2) * width + j - 2]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j - 1] -
//                            smoothCh3Img[(i - 2) * width + j - 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j] -
//                            smoothCh3Img[(i - 2) * width + j]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 1] -
//                            smoothCh3Img[(i - 2) * width + j + 1]) +
//                       2 * (smoothCh3Img[(i + 2) * width + j + 2] -
//                            smoothCh3Img[(i - 2) * width + j + 2]));

//       int gxx = gxCh1 * gxCh1 + gxCh2 * gxCh2 + gxCh3 * gxCh3;
//       int gyy = gyCh1 * gyCh1 + gyCh2 * gyCh2 + gyCh3 * gyCh3;
//       int gxy = gxCh1 * gyCh1 + gxCh2 * gyCh2 + gxCh3 * gyCh3;

// #if 1
//       // Di Zenzo's formulas from Gonzales & Woods - Page 337
//       double theta =
//           atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;  // Gradient Direction
//       int grad = (int)(sqrt(((gxx + gyy) + (gxx - gyy) * cos(2 * theta) +
//                              2 * gxy * sin(2.0 * theta)) /
//                             2.0) +
//                        0.5);  // Gradient Magnitude

// #else
//       // Koschan & Abidi - 2005 - Signal Processing Magazine
//       double theta =
//           atan2(2.0 * gxy, (double)(gxx - gyy)) / 2;  // Gradient Direction

//       double cosTheta = cos(theta);
//       double sinTheta = sin(theta);
//       int grad =
//           (int)(sqrt(gxx * cosTheta * cosTheta + 2 * gxy * sinTheta * cosTheta +
//                      gyy * sinTheta * sinTheta) +
//                 0.5);  // Gradient Magnitude
// #endif

//       // Gradient is perpendicular to the edge passing through the pixel
//       if (theta >= -3.14159 / 4 && theta <= 3.14159 / 4)
//         dirImg[i * width + j] = EDGE_VERTICAL;
//       else
//         dirImg[i * width + j] = EDGE_HORIZONTAL;

//       gradImg[i * width + j] = grad;
//       if (grad > max) max = grad;
//     }  // end-for
//   }    // end-for

//   // Scale the gradient values to 0-255
//   double scale = 255.0 / max;
//   for (int i = 0; i < width * height; i++)
//     gradImg[i] = (short)(gradImg[i] * scale);
// }  // end-ComputeGradientMapByDiZenzo5x5
