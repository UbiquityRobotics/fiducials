#ifndef ED_INTERNALS_H
#define ED_INTERNALS_H

#include "stag/ED/EdgeMap.h"

/// Computes the anchors & links them. Returns the edge map
EdgeMap *DoDetectEdgesByED(short *gradImg, unsigned char *dirImg, int width,
                           int height, int GRADIENT_THRESH,
                           int ANCHOR_THRESH = 0);
// EdgeMap *DoDetectEdgesByED4Dirs(short *gradImg, unsigned char *dirImg,
//                                 int width, int height, int GRADIENT_THRESH,
//                                 int ANCHOR_THRESH = 0);

/// Alternative ED. Just uses gradImg to linking the anchors.
/// All points in the gradient map >= GRADIENT_THRESH are assumed to be anchors.
/// Starts anchor linking with the pixel having the max. gradient value,
/// and walks to the neighboring pixel having the greatest gradient value.
/// Edge directions are NOT used during anchor linking
// EdgeMap *DoDetectEdgesByED(short *gradImg, int width, int height,
//                            int GRADIENT_THRESH);

#endif