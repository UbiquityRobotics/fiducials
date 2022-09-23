#ifndef EDGE_MAP_H
#define EDGE_MAP_H

#include <memory.h>

enum GradientOperator {
  PREWITT_OPERATOR = 101,
  SOBEL_OPERATOR = 102,
  SCHARR_OPERATOR = 103
};

struct Pixel {
  int r, c;
};

struct EdgeSegment {
  Pixel *pixels;  // Pointer to the pixels array
  int noPixels;   // # of pixels in the edge map
};

struct EdgeMap {
 public:
  int width, height;       // Width & height of the image
  unsigned char *edgeImg;  // BW edge map

  Pixel *pixels;  // Edge map in edge segment form
  EdgeSegment *segments;
  int noSegments;

 public:
  // constructor
  EdgeMap(int w, int h) {
    width = w;
    height = h;

    edgeImg = new unsigned char[width * height];

    pixels = new Pixel[width * height];
    segments = new EdgeSegment[width * height];
    noSegments = 0;
  }  // end-EdgeMap

  // Destructor
  ~EdgeMap() {
    delete edgeImg;
    delete pixels;
    delete segments;
  }  // end-~EdgeMap

  void ConvertEdgeSegments2EdgeImg() {
    memset(edgeImg, 0, width * height);

    for (int i = 0; i < noSegments; i++) {
      for (int j = 0; j < segments[i].noPixels; j++) {
        int r = segments[i].pixels[j].r;
        int c = segments[i].pixels[j].c;

        edgeImg[r * width + c] = 255;
      }  // end-for
    }    // end-for
  }      // end-ConvertEdgeSegments2EdgeImg

  EdgeMap *clone() {
    EdgeMap *map2 = new EdgeMap(width, height);
    map2->noSegments = noSegments;

    Pixel *pix = map2->pixels;

    for (int i = 0; i < noSegments; i++) {
      map2->segments[i].noPixels = segments[i].noPixels;
      map2->segments[i].pixels = pix;
      pix += segments[i].noPixels;

      for (int j = 0; j < segments[i].noPixels; j++) {
        map2->segments[i].pixels[j] = segments[i].pixels[j];
      }  // end-for
    }    // end-for

    return map2;
  }  // end-clone
};

#endif