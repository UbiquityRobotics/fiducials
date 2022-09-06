#ifndef VALIDATE_EDGE_SEGMENTS_H
#define VALIDATE_EDGE_SEGMENTS_H

#include "stag/ED/EdgeMap.h"

/// Validate the edge segments using the Helmholtz principle
void ValidateEdgeSegments(EdgeMap *map, unsigned char *srcImg,
                          double divForTestSegment);

/// Using LSD
// void ValidateEdgeSegments2(EdgeMap *map, unsigned char *srcImg,
//                            double divForTestSegment);

// /// Use multiple div values for better results (used in contour computation)
// int ValidateEdgeSegmentsMultipleDiv(EdgeMap *map, unsigned char *srcImg,
//                                     unsigned char *maps[], int noMaps);

// /// Validate the edge segments using the Helmholtz principle (for color images)
// /// channel1, channel2 and channel3 images Prewitt
// void ValidateEdgeSegments(EdgeMap *map, unsigned char *c1Img,
//                           unsigned char *c2Img, unsigned char *c3Img,
//                           double divForTestSegment);

// /// Validate the edge segments using the Helmholtz principle (for color images)
// /// channel1, channel2 and channel3 images with LSD
// void ValidateEdgeSegments2(EdgeMap *map, unsigned char *c1Img,
//                            unsigned char *c2Img, unsigned char *c3Img,
//                            double divForTestSegment);

// /// Use multiple div values for better results (used in contour computation)
// /// op=1 for Prewitt, 2 for LSD
// int ValidateEdgeSegmentsMultipleDiv(EdgeMap *map, unsigned char *c1Img,
//                                     unsigned char *c2Img, unsigned char *c3Img,
//                                     unsigned char *maps[], int noMaps,
//                                     int op = 1);

// /// Validation with an already computed gradient map
// void ValidateEdgeSegmentsWithGradientMap(EdgeMap *map, short *gradImg,
//                                          double divForTestSegment);

// /// Use multiple div values for better results (used in contour computation)
// int ValidateEdgeSegmentsWithGradientMapMultipleDiv(EdgeMap *map, short *gradImg,
//                                                    unsigned char *maps[],
//                                                    int noMaps);

#endif