#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "stag/ED/EdgeMap.h"

void RGB2LabDeneme(unsigned char *redImg, unsigned char *greenImg,
                   unsigned char *blueImg, unsigned char *LImg,
                   unsigned char *aImg, unsigned char *bImg, int width,
                   int height);

void MyRGB2LabFast(unsigned char *redImg, unsigned char *greenImg,
                   unsigned char *blueImg, unsigned char *LImg,
                   unsigned char *aImg, unsigned char *bImg, int width,
                   int height);

void MyRGB2Lab(unsigned char *redImg, unsigned char *greenImg,
               unsigned char *blueImg, unsigned char *LImg, unsigned char *aImg,
               unsigned char *bImg, int width, int height);

void RGB2Lab(IplImage *rgbImg, IplImage *labImg);

void RGB2Lab2(unsigned char *redImg, unsigned char *greenImg,
              unsigned char *blueImg, unsigned char *LImg, unsigned char *aImg,
              unsigned char *bImg, int width, int height);

void StdRGB2Lab(unsigned char *redImg, unsigned char *greenImg,
                unsigned char *blueImg, unsigned char *LImg,
                unsigned char *aImg, unsigned char *bImg, int width,
                int height);

void StdRGB2LabOne(unsigned char r, unsigned char g, unsigned char bl,
                   double *L, double *a, double *b);

void RGB2LabOpenCV(unsigned char *redImg, unsigned char *greenImg,
                   unsigned char *blueImg, unsigned char *LImg,
                   unsigned char *aImg, unsigned char *bImg, int width,
                   int height);

void RGB2Luv(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *LImg, unsigned char *uImg,
             unsigned char *vImg, int width, int height);

void RGB2HSL(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *HImg, unsigned char *SImg,
             unsigned char *LImg, int width, int height);

void RGB2YUV(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *YImg, unsigned char *UImg,
             unsigned char *VImg, int width, int height);

void DumpGradImage(char *file, short *gradImg, int width, int height);
void DumpGradImage(char *file, short *gradImg, int width, int height,
                   int thresh);

void DumpEdgeSegments(char *file, EdgeMap *map);

void ColorEdgeSegments(EdgeMap *map, unsigned char *colorImg,
                       unsigned char *srcImg = NULL);
void ColorEdgeSegments(char *file, EdgeMap *map, unsigned char *srcImg = NULL);

void ShowJointPoints(char *file, EdgeMap *map, unsigned char *jointPoints,
                     unsigned char *srcImg = NULL);

struct DirectoryEntry {
  char filename[100];
};
int GetFilenamesInDirectory(char *dirname, DirectoryEntry *items);

/// Scales a given image
unsigned char *ScaleImage(unsigned char *srcImg, int width, int height,
                          double scale, int *pw, int *ph);

///-------------------------------------------------------------------------------------------
/// Color generator
///
struct ColorGenerator {
  int color;

  ColorGenerator() { color = 0; }
  void getNextColor(int *r, int *g, int *b) {
    switch (color) {
      case 0:
        *r = 255;
        *g = 0;
        *b = 0;
        break;
      case 1:
        *r = 0;
        *g = 255;
        *b = 0;
        break;
      case 2:
        *r = 0;
        *g = 0;
        *b = 255;
        break;
      case 3:
        *r = 255;
        *g = 255;
        *b = 0;
        break;
      case 4:
        *r = 0;
        *g = 255;
        *b = 255;
        break;
      case 5:
        *r = 255;
        *g = 0;
        *b = 255;
        break;
      case 6:
        *r = 255;
        *g = 128;
        *b = 0;
        break;
      case 7:
        *r = 255;
        *g = 0;
        *b = 128;
        break;
      case 8:
        *r = 128;
        *g = 255;
        *b = 0;
        break;
      case 9:
        *r = 0;
        *g = 255;
        *b = 128;
        break;
      case 10:
        *r = 128;
        *g = 0;
        *b = 255;
        break;
      case 11:
        *r = 0;
        *g = 128;
        *b = 255;
        break;
      case 12:
        *r = 0;
        *g = 128;
        *b = 128;
        break;
      case 13:
        *r = 128;
        *g = 0;
        *b = 128;
        break;
      case 14:
        *r = 128;
        *g = 128;
        *b = 0;
        break;
    }  // end-switch

    color++;
    if (color > 14) color = 0;
  }  // end-getNextColor
};

#endif