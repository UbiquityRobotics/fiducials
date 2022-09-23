#ifndef GRADIENT_OPERATORS_H
#define GRADIENT_OPERATORS_H

// void ComputeGradientMapByLSD(unsigned char *smoothImg, short *gradImg,
//                              unsigned char *dirImg, int width, int height,
//                              int GRADIENT_THRESH);
void ComputeGradientMapByPrewitt(unsigned char *smoothImg, short *gradImg,
                                 unsigned char *dirImg, int width, int height,
                                 int GRADIENT_THRESH);

// void ComputeGradientMapByPrewitt(unsigned char *smoothImg, short *gradImg,
//                                  unsigned char *dirImg, int width,
//                                  int height);  // scaled 3x3 Prewitt
// void ComputeGradientMapByPrewitt4Dirs(unsigned char *smoothImg, short *gradImg,
//                                       unsigned char *dirImg, int width,
//                                       int height, int GRADIENT_THRESH);

// void ComputeGradientMapBySobel(unsigned char *smoothImg, short *gradImg,
//                                unsigned char *dirImg, int width, int height,
//                                int GRADIENT_THRESH);
// void ComputeGradientMapByScharr(unsigned char *smoothImg, short *gradImg,
//                                 unsigned char *dirImg, int width, int height,
//                                 int GRADIENT_THRESH);

// /// Color image gradient
// void ComputeGradientMapByPrewitt(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height,
//                                  int GRADIENT_THRESH);

// /// Gradient by 3x3 Prewitt by L1 norm -- scaled to [0-255]
// void ComputeGradientMapByPrewitt(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height);

// /// Gradient by 3x3 Prewitt by L2 norm -- scaled to [0-255]
// void ComputeGradientMapByPrewittL2(unsigned char *smoothCh1Img,
//                                    unsigned char *smoothCh2Img,
//                                    unsigned char *smoothCh3Img, short *gradImg,
//                                    unsigned char *dirImg, int width,
//                                    int height);

// /// Gradient by 5x5 Prewitt -- scaled to [0-255]
// void ComputeGradientMapByPrewitt5x5(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height);

// /// Gradient by 7x7 Prewitt -- scaled to [0-255]
// void ComputeGradientMapByPrewitt7x7(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height);

// // 3x3 Sobel -- scaled to [0-255]
// void ComputeGradientMapBySobel(unsigned char *smoothCh1Img,
//                                unsigned char *smoothCh2Img,
//                                unsigned char *smoothCh3Img, short *gradImg,
//                                unsigned char *dirImg, int width, int height);

// // 3x3 Sobel -- scaled to [0-255]
// void ComputeGradientMapBySobel5x5(unsigned char *smoothCh1Img,
//                                   unsigned char *smoothCh2Img,
//                                   unsigned char *smoothCh3Img, short *gradImg,
//                                   unsigned char *dirImg, int width, int height);

// /// Gradient by 3x3 DiZenzo's method -- scaled to [0-255]
// void ComputeGradientMapByDiZenzo(unsigned char *smoothCh1Img,
//                                  unsigned char *smoothCh2Img,
//                                  unsigned char *smoothCh3Img, short *gradImg,
//                                  unsigned char *dirImg, int width, int height);

// /// Gradient by 3x3 DiZenzo's method -- scaled to [0-255] -- Computes 4
// /// directions instead of just 2
// void ComputeGradientMapByDiZenzo4Dirs(unsigned char *smoothCh1Img,
//                                       unsigned char *smoothCh2Img,
//                                       unsigned char *smoothCh3Img,
//                                       short *gradImg, unsigned char *dirImg,
//                                       int width, int height);

// /// Gradient by 5x5 DiZenzo's method -- scaled to [0-255]
// void ComputeGradientMapByDiZenzo5x5(unsigned char *smoothCh1Img,
//                                     unsigned char *smoothCh2Img,
//                                     unsigned char *smoothCh3Img, short *gradImg,
//                                     unsigned char *dirImg, int width,
//                                     int height);

// /// Gradient by 3x3 EMD -- scaled to [0-255]
// void ComputeGradientMapByEMD(unsigned char *smoothCh1Img,
//                              unsigned char *smoothCh2Img,
//                              unsigned char *smoothCh3Img, short *gradImg,
//                              unsigned char *dirImg, int width, int height);

// /// Gradient by 5x5 EMD -- scaled to [0-255]
// void ComputeGradientMapByEMD5x5(unsigned char *smoothCh1Img,
//                                 unsigned char *smoothCh2Img,
//                                 unsigned char *smoothCh3Img, short *gradImg,
//                                 unsigned char *dirImg, int width, int height);

// // By Saez's method -- scaled to [0-255]
// void ComputeGradientMapByCIE94(unsigned char *smoothCh1Img,
//                                unsigned char *smoothCh2Img,
//                                unsigned char *smoothCh3Img, short *gradImg,
//                                unsigned char *dirImg, int width, int height);

// void ComputeGradientMapByCIEDE2000(unsigned char *smoothCh1Img,
//                                    unsigned char *smoothCh2Img,
//                                    unsigned char *smoothCh3Img, short *gradImg,
//                                    unsigned char *dirImg, int width,
//                                    int height);

#endif