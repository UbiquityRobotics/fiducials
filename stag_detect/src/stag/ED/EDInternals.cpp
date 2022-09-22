#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "stag/ED/EDInternals.h"

/// Special defines
#define EDGE_VERTICAL 1
#define EDGE_HORIZONTAL 2
#define EDGE_45 3
#define EDGE_135 4

#define TEMP_PIXEL 253
#define ANCHOR_PIXEL 254
#define EDGE_PIXEL 255

//#define LEFT  1
//#define RIGHT 2
//#define UP    3
//#define DOWN  4

#define UP_LEFT 1  // diagonal
#define UP 2
#define UP_RIGHT 3
#define RIGHT 4
#define DOWN_RIGHT 5
#define DOWN 6
#define DOWN_LEFT 7
#define LEFT 8

struct StackNode {
  int r, c;    // starting pixel
  int parent;  // parent chain (-1 if no parent)
  int dir;     // direction where you are supposed to go
};

// Used during Edge Linking
struct Chain {
  short dir;           // Direction of the chain
  unsigned short len;  // # of pixels in the chain
  short parent;        // Parent of this node (-1 if no parent)
  short children[2];   // Children of this node (-1 if no children)
  Pixel *pixels;       // Pointer to the beginning of the pixels array
};

///-----------------------------------------------------------------------------------
/// Compute anchor points
///
static void ComputeAnchorPoints(short *gradImg, unsigned char *dirImg,
                                EdgeMap *map, int GRADIENT_THRESH,
                                int ANCHOR_THRESH, int SCAN_INTERVAL) {
  int width = map->width;
  int height = map->height;

  unsigned char *edgeImg = map->edgeImg;
  memset(edgeImg, 0, width * height);

  for (int i = 2; i < height - 2; i++) {
    int start = 2;
    int inc = 1;
    if (i % SCAN_INTERVAL != 0) {
      start = SCAN_INTERVAL;
      inc = SCAN_INTERVAL;
    }

    for (int j = start; j < width - 2; j += inc) {
      if (gradImg[i * width + j] < GRADIENT_THRESH) continue;

      if (dirImg[i * width + j] == EDGE_VERTICAL) {
        // vertical edge
        int diff1 = gradImg[i * width + j] - gradImg[i * width + j - 1];
        int diff2 = gradImg[i * width + j] - gradImg[i * width + j + 1];
        if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
          edgeImg[i * width + j] = ANCHOR_PIXEL;

      } else {
        // horizontal edge
        int diff1 = gradImg[i * width + j] - gradImg[(i - 1) * width + j];
        int diff2 = gradImg[i * width + j] - gradImg[(i + 1) * width + j];
        if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
          edgeImg[i * width + j] = ANCHOR_PIXEL;
      }  // end-else
    }    // end-for-inner
  }      // end-for-outer
}  // end-ComputeAnchorPoints

///-----------------------------------------------------------------------------------
/// Compute anchor points using 4 directions (0, 45, 90 and 135 degree edges)
///
// static void ComputeAnchorPoints4Dirs(short *gradImg, unsigned char *dirImg,
//                                      EdgeMap *map, int GRADIENT_THRESH,
//                                      int ANCHOR_THRESH, int SCAN_INTERVAL) {
//   int width = map->width;
//   int height = map->height;

//   unsigned char *edgeImg = map->edgeImg;
//   memset(edgeImg, 0, width * height);

//   for (int i = 2; i < height - 2; i++) {
//     int start = 2;
//     int inc = 1;
//     if (i % SCAN_INTERVAL != 0) {
//       start = SCAN_INTERVAL;
//       inc = SCAN_INTERVAL;
//     }

//     for (int j = start; j < width - 2; j += inc) {
//       if (gradImg[i * width + j] < GRADIENT_THRESH) continue;

//       if (dirImg[i * width + j] == EDGE_VERTICAL) {
//         // vertical edge
//         int diff1 = gradImg[i * width + j] - gradImg[i * width + j - 1];
//         int diff2 = gradImg[i * width + j] - gradImg[i * width + j + 1];
//         if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
//           edgeImg[i * width + j] = ANCHOR_PIXEL;

//       } else if (dirImg[i * width + j] == EDGE_HORIZONTAL) {
//         // horizontal edge
//         int diff1 = gradImg[i * width + j] - gradImg[(i - 1) * width + j];
//         int diff2 = gradImg[i * width + j] - gradImg[(i + 1) * width + j];
//         if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
//           edgeImg[i * width + j] = ANCHOR_PIXEL;

//       } else if (dirImg[i * width + j] == EDGE_45) {
//         // 45 degree edge
//         int diff1 = gradImg[i * width + j] - gradImg[(i - 1) * width + j - 1];
//         int diff2 = gradImg[i * width + j] - gradImg[(i + 1) * width + j + 1];
//         if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
//           edgeImg[i * width + j] = ANCHOR_PIXEL;

//       } else {  // if (dirImg[i*width+j] == EDGE_135){
//         // 45 degree edge
//         int diff1 = gradImg[i * width + j] - gradImg[(i - 1) * width + j + 1];
//         int diff2 = gradImg[i * width + j] - gradImg[(i + 1) * width + j - 1];
//         if (diff1 >= ANCHOR_THRESH && diff2 >= ANCHOR_THRESH)
//           edgeImg[i * width + j] = ANCHOR_PIXEL;
//       }  // end-else
//     }    // end-for-inner
//   }      // end-for-outer
// }  // end-ComputeAnchorPoints4Dirs

///--------------------------------------------------------------------------
/// Computes anchor offsets & sorts them
///
static int *SortAnchorsByGradValue(short *gradImg, EdgeMap *map,
                                   int *pNoAnchors) {
  int width = map->width;
  int height = map->height;

  int SIZE = 128 * 256;
  int *C = new int[SIZE];
  memset(C, 0, sizeof(int) * SIZE);

  // Count the number of grad values
  for (int i = 1; i < height - 1; i++) {
    for (int j = 1; j < width - 1; j++) {
      if (map->edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

      int grad = gradImg[i * width + j];
      C[grad]++;
    }  // end-for
  }    // end-for

  // Compute indices
  for (int i = 1; i < SIZE; i++) C[i] += C[i - 1];

  int noAnchors = C[SIZE - 1];
  int *A = new int[noAnchors];
  memset(A, 0, sizeof(int) * noAnchors);

  for (int i = 1; i < height - 1; i++) {
    for (int j = 1; j < width - 1; j++) {
      if (map->edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

      int grad = gradImg[i * width + j];
      int index = --C[grad];
      A[index] = i * width + j;  // anchor's offset
    }                            // end-for
  }                              // end-for

  delete C;

  *pNoAnchors = noAnchors;
  return A;
}  // end-SortAnchorsByGradValue

///--------------------------------------------------------------------------
/// Computes the length of the longest chain
///
static int LongestChain(Chain *chains, int root) {
  if (root == -1 || chains[root].len == 0) return 0;

  int len0 = 0;
  if (chains[root].children[0] != -1)
    len0 = LongestChain(chains, chains[root].children[0]);

  int len1 = 0;
  if (chains[root].children[1] != -1)
    len1 = LongestChain(chains, chains[root].children[1]);

  int max = 0;

  if (len0 >= len1) {
    max = len0;
    chains[root].children[1] = -1;

  } else {
    max = len1;
    chains[root].children[0] = -1;
  }  // end-else

  return chains[root].len + max;
}  // end-LongestChain

///-----------------------------------------------------------
/// Retrieves the chain nos from the tree
///
static int RetrieveChainNos(Chain *chains, int root, int chainNos[]) {
  int count = 0;

  while (root != -1) {
    chainNos[count] = root;
    count++;

    if (chains[root].children[0] != -1)
      root = chains[root].children[0];
    else
      root = chains[root].children[1];
  }  // end-while

  return count;
}  // end-RetrieveChainNos

///-----------------------------------------------------------------------------------
/// Join anchor points and compute segment chains at the same time
///
// static void JoinAnchorPoints(short *gradImg, unsigned char *dirImg,
//                              EdgeMap *map, int GRADIENT_THRESH,
//                              int MIN_PATH_LEN) {
//   int width = map->width;
//   int height = map->height;

//   unsigned char *edgeImg = map->edgeImg;
//   int *chainNos = new int[(width + height) * 8];

//   Pixel *pixels = new Pixel[width * height];
//   StackNode *stack = new StackNode[width * height];
//   Chain *chains = new Chain[width * height];

//   int noSegments = 0;
//   int totalPixels = 0;

//   for (int i = 2; i < height - 2; i++) {
//     for (int j = 2; j < width - 2; j++) {
//       if (edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

//       chains[0].len = 0;
//       chains[0].parent = -1;
//       chains[0].dir = 0;
//       chains[0].children[0] = chains[0].children[1] = -1;
//       chains[0].pixels = NULL;

//       int noChains = 1;
//       int len = 0;
//       int duplicatePixelCount = 0;

//       int top = -1;  // top of the stack

//       if (dirImg[i * width + j] == EDGE_VERTICAL) {
//         stack[++top].r = i;
//         stack[top].c = j;
//         stack[top].dir = DOWN;
//         stack[top].parent = 0;

//         stack[++top].r = i;
//         stack[top].c = j;
//         stack[top].dir = UP;
//         stack[top].parent = 0;

//       } else {
//         stack[++top].r = i;
//         stack[top].c = j;
//         stack[top].dir = RIGHT;
//         stack[top].parent = 0;

//         stack[++top].r = i;
//         stack[top].c = j;
//         stack[top].dir = LEFT;
//         stack[top].parent = 0;
//       }  // end-else

//       // While the stack is not empty
//     StartOfWhile:
//       while (top >= 0) {
//         int r = stack[top].r;
//         int c = stack[top].c;
//         int dir = stack[top].dir;
//         int parent = stack[top].parent;
//         top--;

//         if (edgeImg[r * width + c] != EDGE_PIXEL) duplicatePixelCount++;

//         chains[noChains].dir = dir;  // traversal direction
//         chains[noChains].parent = parent;
//         chains[noChains].children[0] = chains[noChains].children[1] = -1;

//         int chainLen = 0;
//         chains[noChains].pixels = &pixels[len];

//         pixels[len].r = r;
//         pixels[len].c = c;
//         len++;
//         chainLen++;

//         if (dir == LEFT) {
//           while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//             edgeImg[r * width + c] = EDGE_PIXEL;

//             // The edge is horizontal. Look LEFT
//             //
//             //   A
//             //   B x
//             //   C
//             //
//             // cleanup up & down pixels
//             if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//               edgeImg[(r - 1) * width + c] = 0;
//             if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//               edgeImg[(r + 1) * width + c] = 0;

//             // Look if there is an edge pixel in the neighbors
//             if (edgeImg[r * width + c - 1] >= ANCHOR_PIXEL) {
//               c--;
//             } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
//               r--;
//               c--;
//             } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
//               r++;
//               c--;
//             } else {
//               // else -- follow max. pixel to the LEFT
//               int A = gradImg[(r - 1) * width + c - 1];
//               int B = gradImg[r * width + c - 1];
//               int C = gradImg[(r + 1) * width + c - 1];

//               if (A > B) {
//                 if (A > C)
//                   r--;
//                 else
//                   r++;
//               } else if (C > B)
//                 r++;
//               c--;
//             }  // end-else

//             if (edgeImg[r * width + c] == EDGE_PIXEL ||
//                 gradImg[r * width + c] < GRADIENT_THRESH) {
//               if (chainLen > 0) {
//                 chains[noChains].len = chainLen;
//                 chains[parent].children[0] = noChains;
//                 noChains++;
//               }  // end-if
//               goto StartOfWhile;
//             }  // end-else

//             pixels[len].r = r;
//             pixels[len].c = c;
//             len++;
//             chainLen++;
//           }  // end-while

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//           len--;
//           chainLen--;

//           chains[noChains].len = chainLen;
//           chains[parent].children[0] = noChains;
//           noChains++;

//         } else if (dir == RIGHT) {
//           while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//             edgeImg[r * width + c] = EDGE_PIXEL;

//             // The edge is horizontal. Look RIGHT
//             //
//             //     A
//             //   x B
//             //     C
//             //
//             // cleanup up&down pixels
//             if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//               edgeImg[(r + 1) * width + c] = 0;
//             if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//               edgeImg[(r - 1) * width + c] = 0;

//             // Look if there is an edge pixel in the neighbors
//             if (edgeImg[r * width + c + 1] >= ANCHOR_PIXEL) {
//               c++;
//             } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
//               r++;
//               c++;
//             } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
//               r--;
//               c++;
//             } else {
//               // else -- follow max. pixel to the RIGHT
//               int A = gradImg[(r - 1) * width + c + 1];
//               int B = gradImg[r * width + c + 1];
//               int C = gradImg[(r + 1) * width + c + 1];

//               if (A > B) {
//                 if (A > C)
//                   r--;  // A
//                 else
//                   r++;  // C
//               } else if (C > B)
//                 r++;  // C
//               c++;
//             }  // end-else

//             if (edgeImg[r * width + c] == EDGE_PIXEL ||
//                 gradImg[r * width + c] < GRADIENT_THRESH) {
//               if (chainLen > 0) {
//                 chains[noChains].len = chainLen;
//                 chains[parent].children[1] = noChains;
//                 noChains++;
//               }  // end-if
//               goto StartOfWhile;
//             }  // end-else

//             pixels[len].r = r;
//             pixels[len].c = c;
//             len++;
//             chainLen++;
//           }  // end-while

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;  // Go down
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;  // Go up
//           stack[top].parent = noChains;

//           len--;
//           chainLen--;

//           chains[noChains].len = chainLen;
//           chains[parent].children[1] = noChains;
//           noChains++;

//         } else if (dir == UP) {
//           while (dirImg[r * width + c] == EDGE_VERTICAL) {
//             edgeImg[r * width + c] = EDGE_PIXEL;

//             // The edge is vertical. Look UP
//             //
//             //   A B C
//             //     x
//             //
//             // Cleanup left & right pixels
//             if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//               edgeImg[r * width + c - 1] = 0;
//             if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//               edgeImg[r * width + c + 1] = 0;

//             // Look if there is an edge pixel in the neighbors
//             if (edgeImg[(r - 1) * width + c] >= ANCHOR_PIXEL) {
//               r--;
//             } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
//               r--;
//               c--;
//             } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
//               r--;
//               c++;
//             } else {
//               // else -- follow the max. pixel UP
//               int A = gradImg[(r - 1) * width + c - 1];
//               int B = gradImg[(r - 1) * width + c];
//               int C = gradImg[(r - 1) * width + c + 1];

//               if (A > B) {
//                 if (A > C)
//                   c--;
//                 else
//                   c++;
//               } else if (C > B)
//                 c++;
//               r--;
//             }  // end-else

//             if (edgeImg[r * width + c] == EDGE_PIXEL ||
//                 gradImg[r * width + c] < GRADIENT_THRESH) {
//               if (chainLen > 0) {
//                 chains[noChains].len = chainLen;
//                 chains[parent].children[0] = noChains;
//                 noChains++;
//               }  // end-if
//               goto StartOfWhile;
//             }  // end-else

//             pixels[len].r = r;
//             pixels[len].c = c;
//             len++;
//             chainLen++;
//           }  // end-while

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//           len--;
//           chainLen--;

//           chains[noChains].len = chainLen;
//           chains[parent].children[0] = noChains;
//           noChains++;

//         } else {  // dir == DOWN
//           while (dirImg[r * width + c] == EDGE_VERTICAL) {
//             edgeImg[r * width + c] = EDGE_PIXEL;

//             // The edge is vertical
//             //
//             //     x
//             //   A B C
//             //
//             // cleanup side pixels
//             if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//               edgeImg[r * width + c + 1] = 0;
//             if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//               edgeImg[r * width + c - 1] = 0;

//             // Look if there is an edge pixel in the neighbors
//             if (edgeImg[(r + 1) * width + c] >= ANCHOR_PIXEL) {
//               r++;
//             } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
//               r++;
//               c++;
//             } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
//               r++;
//               c--;
//             } else {
//               // else -- follow the max. pixel DOWN
//               int A = gradImg[(r + 1) * width + c - 1];
//               int B = gradImg[(r + 1) * width + c];
//               int C = gradImg[(r + 1) * width + c + 1];

//               if (A > B) {
//                 if (A > C)
//                   c--;  // A
//                 else
//                   c++;  // C
//               } else if (C > B)
//                 c++;  // C
//               r++;
//             }  // end-else

//             if (edgeImg[r * width + c] == EDGE_PIXEL ||
//                 gradImg[r * width + c] < GRADIENT_THRESH) {
//               if (chainLen > 0) {
//                 chains[noChains].len = chainLen;
//                 chains[parent].children[1] = noChains;
//                 noChains++;
//               }  // end-if
//               goto StartOfWhile;
//             }  // end-else

//             pixels[len].r = r;
//             pixels[len].c = c;
//             len++;
//             chainLen++;
//           }  // end-while

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//           len--;
//           chainLen--;

//           chains[noChains].len = chainLen;
//           chains[parent].children[1] = noChains;
//           noChains++;
//         }  // end-else

//       }  // end-while

//       if (len - duplicatePixelCount < MIN_PATH_LEN) {
//         for (int k = 0; k < len; k++) {
//           edgeImg[pixels[k].r * width + pixels[k].c] = 0;
//         }  // end-for

//       } else {
//         map->segments[noSegments].pixels = &map->pixels[totalPixels];

//         int totalLen = LongestChain(chains, chains[0].children[1]);
//         int noSegmentPixels = 0;

//         if (totalLen > 0) {
//           // Retrieve the chainNos
//           int count = RetrieveChainNos(chains, chains[0].children[1], chainNos);

//           // Copy these pixels in the reverse order
//           for (int k = count - 1; k >= 0; k--) {
//             int chainNo = chainNos[k];

// #if 1
//             /* See if we can erase some pixels from the last chain. This is for
//              * cleanup */
//             int fr = chains[chainNo].pixels[chains[chainNo].len - 1].r;
//             int fc = chains[chainNo].pixels[chains[chainNo].len - 1].c;

//             int index = noSegmentPixels - 2;
//             while (index >= 0) {
//               int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//               int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//               if (dr <= 1 && dc <= 1) {
//                 // neighbors. Erase last pixel
//                 noSegmentPixels--;
//                 index--;
//               } else
//                 break;
//             }  // end-while

//             if (chains[chainNo].len > 1) {
//               fr = chains[chainNo].pixels[chains[chainNo].len - 2].r;
//               fc = chains[chainNo].pixels[chains[chainNo].len - 2].c;

//               int dr = abs(
//                   fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//               int dc = abs(
//                   fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//               if (dr <= 1 && dc <= 1) chains[chainNo].len--;
//             }  // end-if
// #endif

//             for (int l = chains[chainNo].len - 1; l >= 0; l--) {
//               map->segments[noSegments].pixels[noSegmentPixels++] =
//                   chains[chainNo].pixels[l];
//             }  // end-for

//             chains[chainNo].len = 0;  // Mark as copied
//           }                           // end-for
//         }                             // end-if

//         totalLen = LongestChain(chains, chains[0].children[0]);
//         if (totalLen > 1) {
//           // Retrieve the chainNos
//           int count = RetrieveChainNos(chains, chains[0].children[0], chainNos);

//           // Copy these chains in the forward direction. Skip the first pixel of
//           // the first chain due to repetition with the last pixel of the
//           // previous chain
//           int lastChainNo = chainNos[0];
//           chains[lastChainNo].pixels++;
//           chains[lastChainNo].len--;

//           for (int k = 0; k < count; k++) {
//             int chainNo = chainNos[k];

// #if 1
//             /* See if we can erase some pixels from the last chain. This is for
//              * cleanup */
//             int fr = chains[chainNo].pixels[0].r;
//             int fc = chains[chainNo].pixels[0].c;

//             int index = noSegmentPixels - 2;
//             while (index >= 0) {
//               int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//               int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//               if (dr <= 1 && dc <= 1) {
//                 // neighbors. Erase last pixel
//                 noSegmentPixels--;
//                 index--;
//               } else
//                 break;
//             }  // end-while

//             int startIndex = 0;
//             int chainLen = chains[chainNo].len;
//             if (chainLen > 1) {
//               int fr = chains[chainNo].pixels[1].r;
//               int fc = chains[chainNo].pixels[1].c;

//               int dr = abs(
//                   fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//               int dc = abs(
//                   fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//               if (dr <= 1 && dc <= 1) {
//                 startIndex = 1;
//               }
//             }  // end-if
// #endif

//             /* Start a new chain & copy pixels from the new chain */
//             for (int l = startIndex; l < chains[chainNo].len; l++) {
//               map->segments[noSegments].pixels[noSegmentPixels++] =
//                   chains[chainNo].pixels[l];
//             }  // end-for

//             chains[chainNo].len = 0;  // Mark as copied
//           }                           // end-for
//         }                             // end-if

//         map->segments[noSegments].noPixels = noSegmentPixels;
//         totalPixels += noSegmentPixels;

//         // See if the first pixel can be cleaned up
//         int fr = map->segments[noSegments].pixels[1].r;
//         int fc = map->segments[noSegments].pixels[1].c;

//         int dr =
//             abs(fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//         int dc =
//             abs(fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//         if (dr <= 1 && dc <= 1) {
//           map->segments[noSegments].pixels++;
//           map->segments[noSegments].noPixels--;
//         }  // end-if

//         noSegments++;

//         // Copy the rest of the long chains here
//         for (int k = 2; k < noChains; k++) {
//           if (chains[k].len < 2) continue;

//           totalLen = LongestChain(chains, k);

//           // If long enough, copy
//           if (totalLen >= 10) {
//             map->segments[noSegments].pixels = &map->pixels[totalPixels];

//             // Retrieve the chainNos
//             int count = RetrieveChainNos(chains, k, chainNos);

//             // Copy the pixels
//             noSegmentPixels = 0;
//             for (int k = 0; k < count; k++) {
//               int chainNo = chainNos[k];

// #if 1
//               /* See if we can erase some pixels from the last chain. This is
//                * for cleanup */
//               int fr = chains[chainNo].pixels[0].r;
//               int fc = chains[chainNo].pixels[0].c;

//               int index = noSegmentPixels - 2;
//               while (index >= 0) {
//                 int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//                 int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//                 if (dr <= 1 && dc <= 1) {
//                   // neighbors. Erase last pixel
//                   noSegmentPixels--;
//                   index--;
//                 } else
//                   break;
//               }  // end-while

//               int startIndex = 0;
//               int chainLen = chains[chainNo].len;
//               if (chainLen > 1) {
//                 int fr = chains[chainNo].pixels[1].r;
//                 int fc = chains[chainNo].pixels[1].c;

//                 int dr = abs(
//                     fr -
//                     map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//                 int dc = abs(
//                     fc -
//                     map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//                 if (dr <= 1 && dc <= 1) {
//                   startIndex = 1;
//                 }
//               }  // end-if
// #endif
//               /* Start a new chain & copy pixels from the new chain */
//               for (int l = startIndex; l < chains[chainNo].len; l++) {
//                 map->segments[noSegments].pixels[noSegmentPixels++] =
//                     chains[chainNo].pixels[l];
//               }  // end-for

//               chains[chainNo].len = 0;  // Mark as copied
//             }                           // end-for

//             map->segments[noSegments].noPixels = noSegmentPixels;
//             totalPixels += noSegmentPixels;

//             noSegments++;
//           }  // end-if
//         }    // end-for

//       }  // end-else

//     }  // end-for-inner
//   }    // end-for-outer

//   map->noSegments = noSegments;

//   delete chains;
//   delete stack;
//   delete pixels;
//   delete chainNos;
// }  // end-JoinAnchorPoints

///-----------------------------------------------------------------------------------
/// Join anchors starting with the anchor having the maximum gradient value.
/// To do this, we need to first sort the anchors
///
/// static
void JoinAnchorPointsUsingSortedAnchors(short *gradImg, unsigned char *dirImg,
                                        EdgeMap *map, int GRADIENT_THRESH,
                                        int MIN_PATH_LEN) {
  int width = map->width;
  int height = map->height;

  unsigned char *edgeImg = map->edgeImg;
  int *chainNos = new int[(width + height) * 8];

  Pixel *pixels = new Pixel[width * height];
  StackNode *stack = new StackNode[width * height];
  Chain *chains = new Chain[width * height];

  // First Sort the anchors
  int noAnchors;
  int *A = SortAnchorsByGradValue(gradImg, map, &noAnchors);

  // Now join the anchors starting with the anchor having the greatest gradient
  // value
  int noSegments = 0;
  int totalPixels = 0;

  for (int k = noAnchors - 1; k >= 0; k--) {
    int pixelOffset = A[k];

    int i = pixelOffset / width;
    int j = pixelOffset % width;

    if (edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

    chains[0].len = 0;
    chains[0].parent = -1;
    chains[0].dir = 0;
    chains[0].children[0] = chains[0].children[1] = -1;
    chains[0].pixels = NULL;

    int noChains = 1;
    int len = 0;
    int duplicatePixelCount = 0;

    int top = -1;  // top of the stack

    if (dirImg[i * width + j] == EDGE_VERTICAL) {
      stack[++top].r = i;
      stack[top].c = j;
      stack[top].dir = DOWN;
      stack[top].parent = 0;

      stack[++top].r = i;
      stack[top].c = j;
      stack[top].dir = UP;
      stack[top].parent = 0;

    } else {
      stack[++top].r = i;
      stack[top].c = j;
      stack[top].dir = RIGHT;
      stack[top].parent = 0;

      stack[++top].r = i;
      stack[top].c = j;
      stack[top].dir = LEFT;
      stack[top].parent = 0;
    }  // end-else

    // While the stack is not empty
  StartOfWhile:
    while (top >= 0) {
      int r = stack[top].r;
      int c = stack[top].c;
      int dir = stack[top].dir;
      int parent = stack[top].parent;
      top--;

      if (edgeImg[r * width + c] != EDGE_PIXEL) duplicatePixelCount++;

      chains[noChains].dir = dir;  // traversal direction
      chains[noChains].parent = parent;
      chains[noChains].children[0] = chains[noChains].children[1] = -1;

      int chainLen = 0;
      chains[noChains].pixels = &pixels[len];

      pixels[len].r = r;
      pixels[len].c = c;
      len++;
      chainLen++;

      if (dir == LEFT) {
        while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
          edgeImg[r * width + c] = EDGE_PIXEL;

          // The edge is horizontal. Look LEFT
          //
          //   A
          //   B x
          //   C
          //
          // cleanup up & down pixels
          if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
            edgeImg[(r - 1) * width + c] = 0;
          if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
            edgeImg[(r + 1) * width + c] = 0;

          // Look if there is an edge pixel in the neighbors
          if (edgeImg[r * width + c - 1] >= ANCHOR_PIXEL) {
            c--;
          } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
            r--;
            c--;
          } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
            r++;
            c--;
          } else {
            // else -- follow max. pixel to the LEFT
            int A = gradImg[(r - 1) * width + c - 1];
            int B = gradImg[r * width + c - 1];
            int C = gradImg[(r + 1) * width + c - 1];

            if (A > B) {
              if (A > C)
                r--;
              else
                r++;
            } else if (C > B)
              r++;
            c--;
          }  // end-else

          if (edgeImg[r * width + c] == EDGE_PIXEL ||
              gradImg[r * width + c] < GRADIENT_THRESH) {
            if (chainLen > 0) {
              chains[noChains].len = chainLen;
              chains[parent].children[0] = noChains;
              noChains++;
            }  // end-if
            goto StartOfWhile;
          }  // end-else

          pixels[len].r = r;
          pixels[len].c = c;
          len++;
          chainLen++;
        }  // end-while

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = DOWN;
        stack[top].parent = noChains;

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = UP;
        stack[top].parent = noChains;

        len--;
        chainLen--;

        chains[noChains].len = chainLen;
        chains[parent].children[0] = noChains;
        noChains++;

      } else if (dir == RIGHT) {
        while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
          edgeImg[r * width + c] = EDGE_PIXEL;

          // The edge is horizontal. Look RIGHT
          //
          //     A
          //   x B
          //     C
          //
          // cleanup up&down pixels
          if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
            edgeImg[(r + 1) * width + c] = 0;
          if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
            edgeImg[(r - 1) * width + c] = 0;

          // Look if there is an edge pixel in the neighbors
          if (edgeImg[r * width + c + 1] >= ANCHOR_PIXEL) {
            c++;
          } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
            r++;
            c++;
          } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
            r--;
            c++;
          } else {
            // else -- follow max. pixel to the RIGHT
            int A = gradImg[(r - 1) * width + c + 1];
            int B = gradImg[r * width + c + 1];
            int C = gradImg[(r + 1) * width + c + 1];

            if (A > B) {
              if (A > C)
                r--;  // A
              else
                r++;  // C
            } else if (C > B)
              r++;  // C
            c++;
          }  // end-else

          if (edgeImg[r * width + c] == EDGE_PIXEL ||
              gradImg[r * width + c] < GRADIENT_THRESH) {
            if (chainLen > 0) {
              chains[noChains].len = chainLen;
              chains[parent].children[1] = noChains;
              noChains++;
            }  // end-if
            goto StartOfWhile;
          }  // end-else

          pixels[len].r = r;
          pixels[len].c = c;
          len++;
          chainLen++;
        }  // end-while

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = DOWN;  // Go down
        stack[top].parent = noChains;

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = UP;  // Go up
        stack[top].parent = noChains;

        len--;
        chainLen--;

        chains[noChains].len = chainLen;
        chains[parent].children[1] = noChains;
        noChains++;

      } else if (dir == UP) {
        while (dirImg[r * width + c] == EDGE_VERTICAL) {
          edgeImg[r * width + c] = EDGE_PIXEL;

          // The edge is vertical. Look UP
          //
          //   A B C
          //     x
          //
          // Cleanup left & right pixels
          if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
            edgeImg[r * width + c - 1] = 0;
          if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
            edgeImg[r * width + c + 1] = 0;

          // Look if there is an edge pixel in the neighbors
          if (edgeImg[(r - 1) * width + c] >= ANCHOR_PIXEL) {
            r--;
          } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
            r--;
            c--;
          } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
            r--;
            c++;
          } else {
            // else -- follow the max. pixel UP
            int A = gradImg[(r - 1) * width + c - 1];
            int B = gradImg[(r - 1) * width + c];
            int C = gradImg[(r - 1) * width + c + 1];

            if (A > B) {
              if (A > C)
                c--;
              else
                c++;
            } else if (C > B)
              c++;
            r--;
          }  // end-else

          if (edgeImg[r * width + c] == EDGE_PIXEL ||
              gradImg[r * width + c] < GRADIENT_THRESH) {
            if (chainLen > 0) {
              chains[noChains].len = chainLen;
              chains[parent].children[0] = noChains;
              noChains++;
            }  // end-if
            goto StartOfWhile;
          }  // end-else

          pixels[len].r = r;
          pixels[len].c = c;
          len++;
          chainLen++;
        }  // end-while

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = RIGHT;
        stack[top].parent = noChains;

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = LEFT;
        stack[top].parent = noChains;

        len--;
        chainLen--;

        chains[noChains].len = chainLen;
        chains[parent].children[0] = noChains;
        noChains++;

      } else {  // dir == DOWN
        while (dirImg[r * width + c] == EDGE_VERTICAL) {
          edgeImg[r * width + c] = EDGE_PIXEL;

          // The edge is vertical
          //
          //     x
          //   A B C
          //
          // cleanup side pixels
          if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
            edgeImg[r * width + c + 1] = 0;
          if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
            edgeImg[r * width + c - 1] = 0;

          // Look if there is an edge pixel in the neighbors
          if (edgeImg[(r + 1) * width + c] >= ANCHOR_PIXEL) {
            r++;
          } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
            r++;
            c++;
          } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
            r++;
            c--;
          } else {
            // else -- follow the max. pixel DOWN
            int A = gradImg[(r + 1) * width + c - 1];
            int B = gradImg[(r + 1) * width + c];
            int C = gradImg[(r + 1) * width + c + 1];

            if (A > B) {
              if (A > C)
                c--;  // A
              else
                c++;  // C
            } else if (C > B)
              c++;  // C
            r++;
          }  // end-else

          if (edgeImg[r * width + c] == EDGE_PIXEL ||
              gradImg[r * width + c] < GRADIENT_THRESH) {
            if (chainLen > 0) {
              chains[noChains].len = chainLen;
              chains[parent].children[1] = noChains;
              noChains++;
            }  // end-if
            goto StartOfWhile;
          }  // end-else

          pixels[len].r = r;
          pixels[len].c = c;
          len++;
          chainLen++;
        }  // end-while

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = RIGHT;
        stack[top].parent = noChains;

        stack[++top].r = r;
        stack[top].c = c;
        stack[top].dir = LEFT;
        stack[top].parent = noChains;

        len--;
        chainLen--;

        chains[noChains].len = chainLen;
        chains[parent].children[1] = noChains;
        noChains++;
      }  // end-else

    }  // end-while

    if (len - duplicatePixelCount < MIN_PATH_LEN) {
      for (int k = 0; k < len; k++) {
        edgeImg[pixels[k].r * width + pixels[k].c] = 0;
      }  // end-for

    } else {
      map->segments[noSegments].pixels = &map->pixels[totalPixels];

      int totalLen = LongestChain(chains, chains[0].children[1]);
      int noSegmentPixels = 0;

      if (totalLen > 0) {
        // Retrieve the chainNos
        int count = RetrieveChainNos(chains, chains[0].children[1], chainNos);

        // Copy these pixels in the reverse order
        for (int k = count - 1; k >= 0; k--) {
          int chainNo = chainNos[k];

#if 1
          /* See if we can erase some pixels from the last chain. This is for
           * cleanup */
          int fr = chains[chainNo].pixels[chains[chainNo].len - 1].r;
          int fc = chains[chainNo].pixels[chains[chainNo].len - 1].c;

          int index = noSegmentPixels - 2;
          while (index >= 0) {
            int dr = abs(fr - map->segments[noSegments].pixels[index].r);
            int dc = abs(fc - map->segments[noSegments].pixels[index].c);

            if (dr <= 1 && dc <= 1) {
              // neighbors. Erase last pixel
              noSegmentPixels--;
              index--;
            } else
              break;
          }  // end-while

          if (chains[chainNo].len > 1) {
            fr = chains[chainNo].pixels[chains[chainNo].len - 2].r;
            fc = chains[chainNo].pixels[chains[chainNo].len - 2].c;

            int dr = abs(
                fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
            int dc = abs(
                fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

            if (dr <= 1 && dc <= 1) chains[chainNo].len--;
          }  // end-if
#endif

          for (int l = chains[chainNo].len - 1; l >= 0; l--) {
            map->segments[noSegments].pixels[noSegmentPixels++] =
                chains[chainNo].pixels[l];
          }  // end-for

          chains[chainNo].len = 0;  // Mark as copied
        }                           // end-for
      }                             // end-if

      totalLen = LongestChain(chains, chains[0].children[0]);
      if (totalLen > 1) {
        // Retrieve the chainNos
        int count = RetrieveChainNos(chains, chains[0].children[0], chainNos);

        // Copy these chains in the forward direction. Skip the first pixel of
        // the first chain due to repetition with the last pixel of the previous
        // chain
        int lastChainNo = chainNos[0];
        chains[lastChainNo].pixels++;
        chains[lastChainNo].len--;

        for (int k = 0; k < count; k++) {
          int chainNo = chainNos[k];

#if 1
          /* See if we can erase some pixels from the last chain. This is for
           * cleanup */
          int fr = chains[chainNo].pixels[0].r;
          int fc = chains[chainNo].pixels[0].c;

          int index = noSegmentPixels - 2;
          while (index >= 0) {
            int dr = abs(fr - map->segments[noSegments].pixels[index].r);
            int dc = abs(fc - map->segments[noSegments].pixels[index].c);

            if (dr <= 1 && dc <= 1) {
              // neighbors. Erase last pixel
              noSegmentPixels--;
              index--;
            } else
              break;
          }  // end-while

          int startIndex = 0;
          int chainLen = chains[chainNo].len;
          if (chainLen > 1) {
            int fr = chains[chainNo].pixels[1].r;
            int fc = chains[chainNo].pixels[1].c;

            int dr = abs(
                fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
            int dc = abs(
                fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

            if (dr <= 1 && dc <= 1) {
              startIndex = 1;
            }
          }  // end-if
#endif

          /* Start a new chain & copy pixels from the new chain */
          for (int l = startIndex; l < chains[chainNo].len; l++) {
            map->segments[noSegments].pixels[noSegmentPixels++] =
                chains[chainNo].pixels[l];
          }  // end-for

          chains[chainNo].len = 0;  // Mark as copied
        }                           // end-for
      }                             // end-if

      map->segments[noSegments].noPixels = noSegmentPixels;
      totalPixels += noSegmentPixels;

      // See if the first pixel can be cleaned up
      int fr = map->segments[noSegments].pixels[1].r;
      int fc = map->segments[noSegments].pixels[1].c;

      int dr =
          abs(fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
      int dc =
          abs(fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

      if (dr <= 1 && dc <= 1) {
        map->segments[noSegments].pixels++;
        map->segments[noSegments].noPixels--;
      }  // end-if

      noSegments++;

      // Copy the rest of the long chains here
      for (int k = 2; k < noChains; k++) {
        if (chains[k].len < 2) continue;

        totalLen = LongestChain(chains, k);

        // If long enough, copy
        //          if (totalLen >= 12){
        if (totalLen >= 10) {
          map->segments[noSegments].pixels = &map->pixels[totalPixels];

          // Retrieve the chainNos
          int count = RetrieveChainNos(chains, k, chainNos);

          // Copy the pixels
          noSegmentPixels = 0;
          for (int k = 0; k < count; k++) {
            int chainNo = chainNos[k];

#if 1
            /* See if we can erase some pixels from the last chain. This is for
             * cleanup */
            int fr = chains[chainNo].pixels[0].r;
            int fc = chains[chainNo].pixels[0].c;

            int index = noSegmentPixels - 2;
            while (index >= 0) {
              int dr = abs(fr - map->segments[noSegments].pixels[index].r);
              int dc = abs(fc - map->segments[noSegments].pixels[index].c);

              if (dr <= 1 && dc <= 1) {
                // neighbors. Erase last pixel
                noSegmentPixels--;
                index--;
              } else
                break;
            }  // end-while

            int startIndex = 0;
            int chainLen = chains[chainNo].len;
            if (chainLen > 1) {
              int fr = chains[chainNo].pixels[1].r;
              int fc = chains[chainNo].pixels[1].c;

              int dr = abs(
                  fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
              int dc = abs(
                  fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

              if (dr <= 1 && dc <= 1) {
                startIndex = 1;
              }
            }  // end-if
#endif
            /* Start a new chain & copy pixels from the new chain */
            for (int l = startIndex; l < chains[chainNo].len; l++) {
              map->segments[noSegments].pixels[noSegmentPixels++] =
                  chains[chainNo].pixels[l];
            }  // end-for

            chains[chainNo].len = 0;  // Mark as copied
          }                           // end-for

          map->segments[noSegments].noPixels = noSegmentPixels;
          totalPixels += noSegmentPixels;

          noSegments++;
        }  // end-if
      }    // end-for

    }  // end-else

  }  // end-for-outer

  map->noSegments = noSegments;

  delete A;
  delete chains;
  delete stack;
  delete pixels;
  delete chainNos;
}  // end-JoinAnchorPointsUsingSortedAnchors

///-----------------------------------------------------------------------------------
/// Join anchors starting with the anchor having the maximum gradient value.
/// To do this, we need to first sort the anchors
/// Uses 4 directions during anchor join (0, 45, 90 and 135 degrees)
///
// static
// void JoinAnchorPointsUsingSortedAnchors4Dirs(short *gradImg,
//                                              unsigned char *dirImg,
//                                              EdgeMap *map, int GRADIENT_THRESH,
//                                              int MIN_PATH_LEN) {
//   int width = map->width;
//   int height = map->height;

//   unsigned char *edgeImg = map->edgeImg;
//   int *chainNos = new int[(width + height) * 8];

//   Pixel *pixels = new Pixel[width * height];
//   StackNode *stack = new StackNode[width * height];
//   Chain *chains = new Chain[width * height];

//   // First Sort the anchors
//   int noAnchors;
//   int *A = SortAnchorsByGradValue(gradImg, map, &noAnchors);

//   // Now join the anchors starting with the anchor having the greatest gradient
//   // value
//   int noSegments = 0;
//   int totalPixels = 0;

//   for (int k = noAnchors - 1; k >= 0; k--) {
//     int pixelOffset = A[k];

//     int i = pixelOffset / width;
//     int j = pixelOffset % width;

//     if (edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

//     chains[0].len = 0;
//     chains[0].parent = -1;
//     chains[0].dir = 0;
//     chains[0].children[0] = chains[0].children[1] = -1;
//     chains[0].pixels = NULL;

//     int noChains = 1;
//     int len = 0;
//     int duplicatePixelCount = 0;

//     int top = -1;  // top of the stack

//     if (dirImg[i * width + j] == EDGE_VERTICAL) {
//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = DOWN;
//       stack[top].parent = 0;

//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = UP;
//       stack[top].parent = 0;

//     } else if (dirImg[i * width + j] == EDGE_HORIZONTAL) {
//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = RIGHT;
//       stack[top].parent = 0;

//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = LEFT;
//       stack[top].parent = 0;

//     } else if (dirImg[i * width + j] == EDGE_45) {
//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = UP_RIGHT;
//       stack[top].parent = 0;

//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = DOWN_LEFT;
//       stack[top].parent = 0;

//     } else {  // if (dirImg[i*width+j] == EDGE_135){
//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = UP_LEFT;
//       stack[top].parent = 0;

//       stack[++top].r = i;
//       stack[top].c = j;
//       stack[top].dir = DOWN_RIGHT;
//       stack[top].parent = 0;
//     }  // end-else

//     // While the stack is not empty
//   StartOfWhile:
//     while (top >= 0) {
//       int r = stack[top].r;
//       int c = stack[top].c;
//       int dir = stack[top].dir;
//       int parent = stack[top].parent;
//       top--;

//       if (edgeImg[r * width + c] != EDGE_PIXEL) duplicatePixelCount++;

//       chains[noChains].dir = dir;  // traversal direction
//       chains[noChains].parent = parent;
//       chains[noChains].children[0] = chains[noChains].children[1] = -1;

//       int chainLen = 0;
//       chains[noChains].pixels = &pixels[len];

//       pixels[len].r = r;
//       pixels[len].c = c;
//       len++;
//       chainLen++;

//       if (dir == LEFT) {
//         while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is horizontal. Look LEFT
//           //
//           //   A
//           //   B x
//           //   C
//           //
//           // cleanup up & down pixels
//           if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c] = 0;
//           if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[r * width + c - 1] >= ANCHOR_PIXEL) {
//             c--;
//           } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r--;
//             c--;
//           } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r++;
//             c--;
//           } else {
//             // else -- follow max. pixel to the LEFT
//             int A = gradImg[(r - 1) * width + c - 1];
//             int B = gradImg[r * width + c - 1];
//             int C = gradImg[(r + 1) * width + c - 1];

//             if (A > B) {
//               if (A > C)
//                 r--;
//               else
//                 r++;
//             } else if (C > B)
//               r++;
//             c--;
//           }  // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[0] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_45) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[0] = noChains;
//         noChains++;

//       } else if (dir == RIGHT) {
//         while (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is horizontal. Look RIGHT
//           //
//           //     A
//           //   x B
//           //     C
//           //
//           // cleanup up&down pixels
//           if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c] = 0;
//           if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[r * width + c + 1] >= ANCHOR_PIXEL) {
//             c++;
//           } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r++;
//             c++;
//           } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r--;
//             c++;
//           } else {
//             // else -- follow max. pixel to the RIGHT
//             int A = gradImg[(r - 1) * width + c + 1];
//             int B = gradImg[r * width + c + 1];
//             int C = gradImg[(r + 1) * width + c + 1];

//             if (A > B) {
//               if (A > C)
//                 r--;  // A
//               else
//                 r++;  // C
//             } else if (C > B)
//               r++;  // C
//             c++;
//           }  // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[1] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_45) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[1] = noChains;
//         noChains++;

//       } else if (dir == UP) {
//         while (dirImg[r * width + c] == EDGE_VERTICAL) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is vertical. Look UP
//           //
//           //   A B C
//           //     x
//           //
//           // Cleanup left & right pixels
//           if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c - 1] = 0;
//           if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c + 1] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r - 1) * width + c] >= ANCHOR_PIXEL) {
//             r--;
//           } else if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r--;
//             c--;
//           } else if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r--;
//             c++;
//           } else {
//             // else -- follow the max. pixel UP
//             int A = gradImg[(r - 1) * width + c - 1];
//             int B = gradImg[(r - 1) * width + c];
//             int C = gradImg[(r - 1) * width + c + 1];

//             if (A > B) {
//               if (A > C)
//                 c--;
//               else
//                 c++;
//             } else if (C > B)
//               c++;
//             r--;
//           }  // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[0] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_45) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[0] = noChains;
//         noChains++;

//       } else if (dir == DOWN) {
//         while (dirImg[r * width + c] == EDGE_VERTICAL) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is vertical
//           //
//           //     x
//           //   A B C
//           //
//           // cleanup side pixels
//           if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c + 1] = 0;
//           if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c - 1] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r + 1) * width + c] >= ANCHOR_PIXEL) {
//             r++;
//           } else if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r++;
//             c++;
//           } else if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r++;
//             c--;
//           } else {
//             // else -- follow the max. pixel DOWN
//             int A = gradImg[(r + 1) * width + c - 1];
//             int B = gradImg[(r + 1) * width + c];
//             int C = gradImg[(r + 1) * width + c + 1];

//             if (A > B) {
//               if (A > C)
//                 c--;  // A
//               else
//                 c++;  // C
//             } else if (C > B)
//               c++;  // C
//             r++;
//           }  // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[1] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_45) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[1] = noChains;
//         noChains++;

//       } else if (dir == UP_RIGHT) {
//         while (dirImg[r * width + c] == EDGE_45) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is 45 degrees
//           //
//           //     C B
//           //     x A
//           //
//           // cleanup side pixels
//           if (edgeImg[(r - 1) * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c - 1] = 0;
//           if (edgeImg[(r + 1) * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c + 1] = 0;
//           if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c - 1] = 0;
//           if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r - 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r--;
//             c++;
//           } else if (edgeImg[(r - 1) * width + c] >= ANCHOR_PIXEL) {
//             r--;
//           } else if (edgeImg[r * width + c + 1] >= ANCHOR_PIXEL) {
//             c++;
//           } else {
//             // else -- follow the max. pixel UP-RIGHT
//             int A = gradImg[r * width + c + 1];
//             int B = gradImg[(r - 1) * width + c + 1];
//             int C = gradImg[(r - 1) * width + c];

//             if (A > B) {
//               if (A > C)
//                 c++;  // A
//               else
//                 r--;  // C
//             } else if (C > B)
//               r--;  // C
//             else {
//               r--;
//               c++;
//             }  // B
//           }    // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[0] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[0] = noChains;
//         noChains++;

//       } else if (dir == DOWN_LEFT) {
//         while (dirImg[r * width + c] == EDGE_45) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is 45 degrees
//           //
//           //     A x
//           //     B C
//           //
//           // cleanup side pixels
//           if (edgeImg[(r - 1) * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c - 1] = 0;
//           if (edgeImg[(r + 1) * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c + 1] = 0;
//           if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c + 1] = 0;
//           if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r + 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r++;
//             c--;
//           } else if (edgeImg[(r + 1) * width + c] >= ANCHOR_PIXEL) {
//             r++;
//           } else if (edgeImg[r * width + c - 1] >= ANCHOR_PIXEL) {
//             c--;
//           } else {
//             // else -- follow the max. pixel DOWN-LEFT
//             int A = gradImg[r * width + c - 1];
//             int B = gradImg[(r + 1) * width + c - 1];
//             int C = gradImg[(r + 1) * width + c];

//             if (A > B) {
//               if (A > C)
//                 c--;  // A
//               else
//                 r++;  // C
//             } else if (C > B)
//               r++;  // C
//             else {
//               r++;
//               c--;
//             }  // B
//           }    // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[1] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_135){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_LEFT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_RIGHT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[1] = noChains;
//         noChains++;

//       } else if (dir == UP_LEFT) {
//         while (dirImg[r * width + c] == EDGE_135) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is 135 degrees
//           //
//           //     B C
//           //     A x
//           //
//           // cleanup side pixels
//           if (edgeImg[(r - 1) * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c + 1] = 0;
//           if (edgeImg[(r + 1) * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c - 1] = 0;
//           if (edgeImg[r * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c + 1] = 0;
//           if (edgeImg[(r + 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r - 1) * width + c - 1] >= ANCHOR_PIXEL) {
//             r--;
//             c--;
//           } else if (edgeImg[(r - 1) * width + c] >= ANCHOR_PIXEL) {
//             r--;
//           } else if (edgeImg[r * width + c - 1] >= ANCHOR_PIXEL) {
//             c--;
//           } else {
//             // else -- follow the max. pixel UP-LEFT
//             int A = gradImg[r * width + c - 1];
//             int B = gradImg[(r - 1) * width + c - 1];
//             int C = gradImg[(r - 1) * width + c];

//             if (A > B) {
//               if (A > C)
//                 c--;  // A
//               else
//                 r--;  // C
//             } else if (C > B)
//               r--;  // C
//             else {
//               r--;
//               c--;
//             }  // B
//           }    // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[0] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_45){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[0] = noChains;
//         noChains++;

//       } else {  // if (dir == DOWN_RIGHT){
//         while (dirImg[r * width + c] == EDGE_135) {
//           edgeImg[r * width + c] = EDGE_PIXEL;

//           // The edge is 135 degrees
//           //
//           //     x C
//           //     A B
//           //
//           // cleanup side pixels
//           if (edgeImg[(r - 1) * width + c + 1] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c + 1] = 0;
//           if (edgeImg[(r + 1) * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[(r + 1) * width + c - 1] = 0;
//           if (edgeImg[r * width + c - 1] == ANCHOR_PIXEL)
//             edgeImg[r * width + c - 1] = 0;
//           if (edgeImg[(r - 1) * width + c] == ANCHOR_PIXEL)
//             edgeImg[(r - 1) * width + c] = 0;

//           // Look if there is an edge pixel in the neighbors
//           if (edgeImg[(r + 1) * width + c + 1] >= ANCHOR_PIXEL) {
//             r++;
//             c++;
//           } else if (edgeImg[(r + 1) * width + c] >= ANCHOR_PIXEL) {
//             r++;
//           } else if (edgeImg[r * width + c + 1] >= ANCHOR_PIXEL) {
//             c++;
//           } else {
//             // else -- follow the max. pixel DOWN-RIGHT
//             int A = gradImg[r * width + c + 1];
//             int B = gradImg[(r + 1) * width + c + 1];
//             int C = gradImg[(r + 1) * width + c];

//             if (A > B) {
//               if (A > C)
//                 c++;  // A
//               else
//                 r++;  // C
//             } else if (C > B)
//               r++;  // C
//             else {
//               r++;
//               c++;
//             }  // B
//           }    // end-else

//           if (edgeImg[r * width + c] == EDGE_PIXEL ||
//               gradImg[r * width + c] < GRADIENT_THRESH) {
//             if (chainLen > 0) {
//               chains[noChains].len = chainLen;
//               chains[parent].children[1] = noChains;
//               noChains++;
//             }  // end-if
//             goto StartOfWhile;
//           }  // end-else

//           pixels[len].r = r;
//           pixels[len].c = c;
//           len++;
//           chainLen++;
//         }  // end-while

//         if (dirImg[r * width + c] == EDGE_HORIZONTAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = LEFT;
//           stack[top].parent = noChains;

//         } else if (dirImg[r * width + c] == EDGE_VERTICAL) {
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP;
//           stack[top].parent = noChains;

//         } else {  // if (dirImg[r*width+c] == EDGE_45){
//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = UP_RIGHT;
//           stack[top].parent = noChains;

//           stack[++top].r = r;
//           stack[top].c = c;
//           stack[top].dir = DOWN_LEFT;
//           stack[top].parent = noChains;
//         }  // end-else

//         len--;
//         chainLen--;

//         chains[noChains].len = chainLen;
//         chains[parent].children[1] = noChains;
//         noChains++;
//       }  // end-else

//     }  // end-while

//     if (len - duplicatePixelCount < MIN_PATH_LEN) {
//       for (int k = 0; k < len; k++) {
//         edgeImg[pixels[k].r * width + pixels[k].c] = 0;
//       }  // end-for

//     } else {
//       map->segments[noSegments].pixels = &map->pixels[totalPixels];

//       int totalLen = LongestChain(chains, chains[0].children[1]);
//       int noSegmentPixels = 0;

//       if (totalLen > 0) {
//         // Retrieve the chainNos
//         int count = RetrieveChainNos(chains, chains[0].children[1], chainNos);

//         // Copy these pixels in the reverse order
//         for (int k = count - 1; k >= 0; k--) {
//           int chainNo = chainNos[k];

// #if 1
//           /* See if we can erase some pixels from the last chain. This is for
//            * cleanup */
//           int fr = chains[chainNo].pixels[chains[chainNo].len - 1].r;
//           int fc = chains[chainNo].pixels[chains[chainNo].len - 1].c;

//           int index = noSegmentPixels - 2;
//           while (index >= 0) {
//             int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//             int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//             if (dr <= 1 && dc <= 1) {
//               // neighbors. Erase last pixel
//               noSegmentPixels--;
//               index--;
//             } else
//               break;
//           }  // end-while

//           if (chains[chainNo].len > 1) {
//             fr = chains[chainNo].pixels[chains[chainNo].len - 2].r;
//             fc = chains[chainNo].pixels[chains[chainNo].len - 2].c;

//             int dr = abs(
//                 fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//             int dc = abs(
//                 fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//             if (dr <= 1 && dc <= 1) chains[chainNo].len--;
//           }  // end-if
// #endif

//           for (int l = chains[chainNo].len - 1; l >= 0; l--) {
//             map->segments[noSegments].pixels[noSegmentPixels++] =
//                 chains[chainNo].pixels[l];
//           }  // end-for

//           chains[chainNo].len = 0;  // Mark as copied
//         }                           // end-for
//       }                             // end-if

//       totalLen = LongestChain(chains, chains[0].children[0]);
//       if (totalLen > 1) {
//         // Retrieve the chainNos
//         int count = RetrieveChainNos(chains, chains[0].children[0], chainNos);

//         // Copy these chains in the forward direction. Skip the first pixel of
//         // the first chain due to repetition with the last pixel of the previous
//         // chain
//         int lastChainNo = chainNos[0];
//         chains[lastChainNo].pixels++;
//         chains[lastChainNo].len--;

//         for (int k = 0; k < count; k++) {
//           int chainNo = chainNos[k];

// #if 1
//           /* See if we can erase some pixels from the last chain. This is for
//            * cleanup */
//           int fr = chains[chainNo].pixels[0].r;
//           int fc = chains[chainNo].pixels[0].c;

//           int index = noSegmentPixels - 2;
//           while (index >= 0) {
//             int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//             int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//             if (dr <= 1 && dc <= 1) {
//               // neighbors. Erase last pixel
//               noSegmentPixels--;
//               index--;
//             } else
//               break;
//           }  // end-while

//           int startIndex = 0;
//           int chainLen = chains[chainNo].len;
//           if (chainLen > 1) {
//             int fr = chains[chainNo].pixels[1].r;
//             int fc = chains[chainNo].pixels[1].c;

//             int dr = abs(
//                 fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//             int dc = abs(
//                 fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//             if (dr <= 1 && dc <= 1) {
//               startIndex = 1;
//             }
//           }  // end-if
// #endif

//           /* Start a new chain & copy pixels from the new chain */
//           for (int l = startIndex; l < chains[chainNo].len; l++) {
//             map->segments[noSegments].pixels[noSegmentPixels++] =
//                 chains[chainNo].pixels[l];
//           }  // end-for

//           chains[chainNo].len = 0;  // Mark as copied
//         }                           // end-for
//       }                             // end-if

//       map->segments[noSegments].noPixels = noSegmentPixels;
//       totalPixels += noSegmentPixels;

//       // See if the first pixel can be cleaned up
//       int fr = map->segments[noSegments].pixels[1].r;
//       int fc = map->segments[noSegments].pixels[1].c;

//       int dr =
//           abs(fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//       int dc =
//           abs(fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//       if (dr <= 1 && dc <= 1) {
//         map->segments[noSegments].pixels++;
//         map->segments[noSegments].noPixels--;
//       }  // end-if

//       noSegments++;

//       // Copy the rest of the long chains here
//       for (int k = 2; k < noChains; k++) {
//         if (chains[k].len < 2) continue;

//         totalLen = LongestChain(chains, k);

//         // If long enough, copy
//         //          if (totalLen >= 12){
//         if (totalLen >= 10) {
//           map->segments[noSegments].pixels = &map->pixels[totalPixels];

//           // Retrieve the chainNos
//           int count = RetrieveChainNos(chains, k, chainNos);

//           // Copy the pixels
//           noSegmentPixels = 0;
//           for (int k = 0; k < count; k++) {
//             int chainNo = chainNos[k];

// #if 1
//             /* See if we can erase some pixels from the last chain. This is for
//              * cleanup */
//             int fr = chains[chainNo].pixels[0].r;
//             int fc = chains[chainNo].pixels[0].c;

//             int index = noSegmentPixels - 2;
//             while (index >= 0) {
//               int dr = abs(fr - map->segments[noSegments].pixels[index].r);
//               int dc = abs(fc - map->segments[noSegments].pixels[index].c);

//               if (dr <= 1 && dc <= 1) {
//                 // neighbors. Erase last pixel
//                 noSegmentPixels--;
//                 index--;
//               } else
//                 break;
//             }  // end-while

//             int startIndex = 0;
//             int chainLen = chains[chainNo].len;
//             if (chainLen > 1) {
//               int fr = chains[chainNo].pixels[1].r;
//               int fc = chains[chainNo].pixels[1].c;

//               int dr = abs(
//                   fr - map->segments[noSegments].pixels[noSegmentPixels - 1].r);
//               int dc = abs(
//                   fc - map->segments[noSegments].pixels[noSegmentPixels - 1].c);

//               if (dr <= 1 && dc <= 1) {
//                 startIndex = 1;
//               }
//             }  // end-if
// #endif
//             /* Start a new chain & copy pixels from the new chain */
//             for (int l = startIndex; l < chains[chainNo].len; l++) {
//               map->segments[noSegments].pixels[noSegmentPixels++] =
//                   chains[chainNo].pixels[l];
//             }  // end-for

//             chains[chainNo].len = 0;  // Mark as copied
//           }                           // end-for

//           map->segments[noSegments].noPixels = noSegmentPixels;
//           totalPixels += noSegmentPixels;

//           noSegments++;
//         }  // end-if
//       }    // end-for

//     }  // end-else

//   }  // end-for-outer

//   map->noSegments = noSegments;

//   delete A;
//   delete chains;
//   delete stack;
//   delete pixels;
//   delete chainNos;
// }  // end-JoinAnchorPointsUsingSortedAnchors4Dirs

///----------------------------------------------------------------------------------------------
/// Detect edges by edge drawing method
///
EdgeMap *DoDetectEdgesByED(short *gradImg, unsigned char *dirImg, int width,
                           int height, int GRADIENT_THRESH, int ANCHOR_THRESH) {
  // Check parameters for sanity
  if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
  if (ANCHOR_THRESH < 0) ANCHOR_THRESH = 0;
  int SCAN_INTERVAL = 1;  // 2
  int MIN_PATH_LEN = 10;  // 10 pixels

  // Edge map to be returned
  EdgeMap *map = new EdgeMap(width, height);

  /*------------ COMPUTE ANCHORS -------------------*/
  ComputeAnchorPoints(gradImg, dirImg, map, GRADIENT_THRESH, ANCHOR_THRESH,
                      SCAN_INTERVAL);

  /*------------ JOIN ANCHORS -------------------*/
  JoinAnchorPointsUsingSortedAnchors(gradImg, dirImg, map, GRADIENT_THRESH,
                                     MIN_PATH_LEN);
  //  JoinAnchorPoints(gradImg, dirImg, map, GRADIENT_THRESH, MIN_PATH_LEN);

  return map;
}  // DoDetectEdgesByED

///----------------------------------------------------------------------------------------------
/// Detect edges by edge drawing method
///
// EdgeMap *DoDetectEdgesByED4Dirs(short *gradImg, unsigned char *dirImg,
//                                 int width, int height, int GRADIENT_THRESH,
//                                 int ANCHOR_THRESH) {
//   // Check parameters for sanity
//   if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
//   if (ANCHOR_THRESH < 0) ANCHOR_THRESH = 0;
//   int SCAN_INTERVAL = 1;  // 2
//   int MIN_PATH_LEN = 10;  // 10 pixels

//   // Edge map to be returned
//   EdgeMap *map = new EdgeMap(width, height);

//   /*------------ COMPUTE ANCHORS -------------------*/
//   ComputeAnchorPoints4Dirs(gradImg, dirImg, map, GRADIENT_THRESH, ANCHOR_THRESH,
//                            SCAN_INTERVAL);

//   /*------------ JOIN ANCHORS -------------------*/
//   JoinAnchorPointsUsingSortedAnchors4Dirs(gradImg, dirImg, map, GRADIENT_THRESH,
//                                           MIN_PATH_LEN);

//   return map;
// }  // DoDetectEdgesByED4Dirs

/**************************** ALTERNATIVE ED THAT DOES NOT USE EDGE DIRECTIONS
 * FOR LINKING ***********************/
// #if 0
// ///-------------------------- Original EDWalk ------------------------------
// ///-------------------------------------------------------------------------
// /// Walks from (r, c) using the grad image (uses 8 directions)
// ///
// int EDWalk(short *gradImg, EdgeMap *map, int GRADIENT_THRESH, int r, int c, int dir, Pixel *pixels){
//   unsigned char *edgeImg = map->edgeImg;

//   int width = map->width;
//   int height = map->height;

//   int count = 0;
//   while (1){
//     if (r<=0 || r>=height-1) return count;
//     if (c<=0 || c>=width-1) return count;

//     pixels[count].r = r;
//     pixels[count].c = c;
//     count++;

//     edgeImg[r*width+c] = EDGE_PIXEL;

//     int maxGrad = GRADIENT_THRESH-1;
//     int nextDir = -1;
//     int grad;
//     int nr=r, nc=c;

//     if        (dir == UP_LEFT){
//       // Left
//       if (edgeImg[r*width+c-1] == EDGE_PIXEL) return count;  
//       grad = gradImg[r*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c-1; nextDir = LEFT;}

//       // Up
//       if (edgeImg[(r-1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c; nextDir = UP;}

//       // Up-Left
//       if (edgeImg[(r-1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c-1; nextDir = UP_LEFT;}

//     } else if (dir == UP){
//       // Up
//       if (edgeImg[(r-1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c; nextDir = UP;}

//       // Up-Left
//       if (edgeImg[(r-1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c-1; nextDir = UP_LEFT;}

//       // Up-Right
//       if (edgeImg[(r-1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c+1; nextDir = UP_RIGHT;}

//     } else if (dir == UP_RIGHT){
//       // Up
//       if (edgeImg[(r-1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c; nextDir = UP;}

//       // Right
//       if (edgeImg[r*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[r*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c+1; nextDir = RIGHT;}

//       // Up-Right
//       if (edgeImg[(r-1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c+1; nextDir = UP_RIGHT;}

//     } else if (dir == RIGHT){
//       // Right
//       if (edgeImg[r*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[r*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c+1; nextDir = RIGHT;}

//       // Up-Right
//       if (edgeImg[(r-1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c+1; nextDir = UP_RIGHT;}

//       // Down-Right
//       if (edgeImg[(r+1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c+1; nextDir = DOWN_RIGHT;}

//     } else if (dir == DOWN_RIGHT){
//       // Right
//       if (edgeImg[r*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[r*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c+1; nextDir = RIGHT;}

//       // Down
//       if (edgeImg[(r+1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c; nextDir = DOWN;}

//       // Down-Right
//       if (edgeImg[(r+1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c+1; nextDir = DOWN_RIGHT;}

//     } else if (dir == DOWN){
//       // Down
//       if (edgeImg[(r+1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c; nextDir = DOWN;}

//       // Down-Right
//       if (edgeImg[(r+1)*width+c+1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c+1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c+1; nextDir = DOWN_RIGHT;}

//       // Down-Left
//       if (edgeImg[(r+1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c-1; nextDir = DOWN_LEFT;}

//     } else if (dir == DOWN_LEFT){
//       // Down
//       if (edgeImg[(r+1)*width+c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c; nextDir = DOWN;}

//       // Left
//       if (edgeImg[r*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[r*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c-1; nextDir = LEFT;}

//       // Down-Left
//       if (edgeImg[(r+1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c-1; nextDir = DOWN_LEFT;}

//     } else { // (dir == LEFT){
//       // Left
//       if (edgeImg[r*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[r*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r; nc = c-1; nextDir = LEFT;}

//       // Down-Left
//       if (edgeImg[(r+1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r+1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r+1; nc = c-1; nextDir = DOWN_LEFT;}

//       // Up-Left
//       if (edgeImg[(r-1)*width+c-1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r-1)*width+c-1];
//       if (grad > maxGrad){maxGrad = grad; nr = r-1; nc = c-1; nextDir = UP_LEFT;}
//     } // end-else 

//     if (nextDir < 0) break;
//     r = nr;
//     c = nc;
//     dir = nextDir;
//   } //end-while

//   return count;
// } //end-EDWalk
// #else
// ///------------------------------ New EDWalk -------------------------------
// ///-------------------------------------------------------------------------
// /// Walks from (r, c) using the grad image (uses 8 directions)
// ///
// int EDWalk(short *gradImg, EdgeMap *map, int GRADIENT_THRESH, int r, int c,
//            int dir, Pixel *pixels) {
//   unsigned char *edgeImg = map->edgeImg;

//   int width = map->width;
//   int height = map->height;

//   int count = 0;
//   while (1) {
//     if (r <= 0 || r >= height - 1) return count;
//     if (c <= 0 || c >= width - 1) return count;

//     pixels[count].r = r;
//     pixels[count].c = c;
//     count++;

//     edgeImg[r * width + c] = EDGE_PIXEL;

//     int maxGrad = GRADIENT_THRESH - 1;
//     int nextDir = -1;
//     int grad;
//     int nr = r, nc = c;

//     if (dir == UP_LEFT) {
//       // Left
//       if (edgeImg[r * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c - 1;
//         nextDir = LEFT;
//       }

//       // Up
//       if (edgeImg[(r - 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c;
//         nextDir = UP;
//       }

//       // Up-Left
//       if (edgeImg[(r - 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c - 1;
//         nextDir = UP_LEFT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up-right & down-left & pick the max
//         if (gradImg[(r - 1) * width + c + 1] >
//             gradImg[(r + 1) * width + c - 1]) {
//           grad = gradImg[(r - 1) * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c + 1;
//             nextDir = UP_RIGHT;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c - 1;
//             nextDir = DOWN_LEFT;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == UP) {
//       // Up
//       if (edgeImg[(r - 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c;
//         nextDir = UP;
//       }

//       // Up-Left
//       if (edgeImg[(r - 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c - 1;
//         nextDir = UP_LEFT;
//       }

//       // Up-Right
//       if (edgeImg[(r - 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c + 1;
//         nextDir = UP_RIGHT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at left & right & pick the max
//         if (gradImg[r * width + c - 1] > gradImg[r * width + c + 1]) {
//           grad = gradImg[r * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r;
//             nc = c - 1;
//             nextDir = LEFT;
//           }
//         } else {
//           grad = gradImg[r * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r;
//             nc = c + 1;
//             nextDir = RIGHT;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == UP_RIGHT) {
//       // Up
//       if (edgeImg[(r - 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c;
//         nextDir = UP;
//       }

//       // Right
//       if (edgeImg[r * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c + 1;
//         nextDir = RIGHT;
//       }

//       // Up-Right
//       if (edgeImg[(r - 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c + 1;
//         nextDir = UP_RIGHT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up-left & down-right & pick the max
//         if (gradImg[(r - 1) * width + c - 1] >
//             gradImg[(r + 1) * width + c + 1]) {
//           grad = gradImg[(r - 1) * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c - 1;
//             nextDir = UP_LEFT;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c + 1;
//             nextDir = DOWN_RIGHT;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == RIGHT) {
//       // Right
//       if (edgeImg[r * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c + 1;
//         nextDir = RIGHT;
//       }

//       // Up-Right
//       if (edgeImg[(r - 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c + 1;
//         nextDir = UP_RIGHT;
//       }

//       // Down-Right
//       if (edgeImg[(r + 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c + 1;
//         nextDir = DOWN_RIGHT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up & down & pick the max
//         if (gradImg[(r - 1) * width + c] > gradImg[(r + 1) * width + c]) {
//           grad = gradImg[(r - 1) * width + c];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c;
//             nextDir = UP;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c;
//             nextDir = DOWN;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == DOWN_RIGHT) {
//       // Right
//       if (edgeImg[r * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c + 1;
//         nextDir = RIGHT;
//       }

//       // Down
//       if (edgeImg[(r + 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c;
//         nextDir = DOWN;
//       }

//       // Down-Right
//       if (edgeImg[(r + 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c + 1;
//         nextDir = DOWN_RIGHT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up-right & down-left & pick the max
//         if (gradImg[(r - 1) * width + c + 1] >
//             gradImg[(r + 1) * width + c - 1]) {
//           grad = gradImg[(r - 1) * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c + 1;
//             nextDir = UP_RIGHT;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c - 1;
//             nextDir = DOWN_LEFT;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == DOWN) {
//       // Down
//       if (edgeImg[(r + 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c;
//         nextDir = DOWN;
//       }

//       // Down-Right
//       if (edgeImg[(r + 1) * width + c + 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c + 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c + 1;
//         nextDir = DOWN_RIGHT;
//       }

//       // Down-Left
//       if (edgeImg[(r + 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c - 1;
//         nextDir = DOWN_LEFT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at left & right & pick the max
//         if (gradImg[r * width + c - 1] > gradImg[r * width + c + 1]) {
//           grad = gradImg[r * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r;
//             nc = c - 1;
//             nextDir = LEFT;
//           }
//         } else {
//           grad = gradImg[r * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r;
//             nc = c + 1;
//             nextDir = RIGHT;
//           }
//         }  // end-else
//       }    // end-if

//     } else if (dir == DOWN_LEFT) {
//       // Down
//       if (edgeImg[(r + 1) * width + c] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c;
//         nextDir = DOWN;
//       }

//       // Left
//       if (edgeImg[r * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c - 1;
//         nextDir = LEFT;
//       }

//       // Down-Left
//       if (edgeImg[(r + 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c - 1;
//         nextDir = DOWN_LEFT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up-left & down-right & pick the max
//         if (gradImg[(r - 1) * width + c - 1] >
//             gradImg[(r + 1) * width + c + 1]) {
//           grad = gradImg[(r - 1) * width + c - 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c - 1;
//             nextDir = UP_LEFT;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c + 1];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c + 1;
//             nextDir = DOWN_RIGHT;
//           }
//         }  // end-else
//       }    // end-if

//     } else {  // (dir == LEFT){
//       // Left
//       if (edgeImg[r * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[r * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r;
//         nc = c - 1;
//         nextDir = LEFT;
//       }

//       // Down-Left
//       if (edgeImg[(r + 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r + 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r + 1;
//         nc = c - 1;
//         nextDir = DOWN_LEFT;
//       }

//       // Up-Left
//       if (edgeImg[(r - 1) * width + c - 1] == EDGE_PIXEL) return count;
//       grad = gradImg[(r - 1) * width + c - 1];
//       if (grad > maxGrad) {
//         maxGrad = grad;
//         nr = r - 1;
//         nc = c - 1;
//         nextDir = UP_LEFT;
//       }

//       if (nextDir < 0) {
//         // If still not found, look at up & down & pick the max
//         if (gradImg[(r - 1) * width + c] > gradImg[(r + 1) * width + c]) {
//           grad = gradImg[(r - 1) * width + c];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r - 1;
//             nc = c;
//             nextDir = UP;
//           }
//         } else {
//           grad = gradImg[(r + 1) * width + c];
//           if (grad > maxGrad) {
//             maxGrad = grad;
//             nr = r + 1;
//             nc = c;
//             nextDir = DOWN;
//           }
//         }  // end-else
//       }    // end-if
//     }      // end-else

//     if (nextDir < 0) break;
//     if (edgeImg[nr * width + nc] == EDGE_PIXEL) return count;

//     r = nr;
//     c = nc;
//     dir = nextDir;
//   }  // end-while

//   return count;
// }  // end-EDWalk
// #endif

// ///------------------------------------------------------------------------------------------------
// /// Alternative anchor linking algorithm that works only by walking over the
// /// maximas of the gradImg. Does not use edge directions
// ///
// void JoinAnchorPointsUsingSortedAnchors(short *gradImg, EdgeMap *map,
//                                         int GRADIENT_THRESH, int MIN_PATH_LEN) {
//   int width = map->width;
//   int height = map->height;

//   unsigned char *edgeImg = map->edgeImg;
//   Pixel *tmpPixels = new Pixel[width * height];

//   int noSegments = 0;
//   int totalLen = 0;

//   // Sort the anchors
//   int noAnchors;
//   int *A = SortAnchorsByGradValue(gradImg, map, &noAnchors);

//   // Go over the anchors in sorted order
//   for (int k = noAnchors - 1; k >= 0; k--) {
//     int pixelOffset = A[k];

//     int i = pixelOffset / width;
//     int j = pixelOffset % width;

//     if (edgeImg[i * width + j] != ANCHOR_PIXEL) continue;

//     int dir1 = LEFT;
//     int dir2 = RIGHT;
//     int maxGrad = gradImg[i * width + j - 1];

//     if (gradImg[(i - 1) * width + j] > maxGrad) {
//       maxGrad = gradImg[(i - 1) * width + j];
//       dir1 = UP;
//       dir2 = DOWN;
//     }
//     if (gradImg[(i + 1) * width + j] > maxGrad) {
//       maxGrad = gradImg[(i + 1) * width + j];
//       dir1 = DOWN;
//       dir2 = UP;
//     }
//     if (gradImg[i * width + j + 1] > maxGrad) {
//       maxGrad = gradImg[i * width + j + 1];
//       dir1 = RIGHT;
//       dir2 = LEFT;
//     }

//     if (gradImg[(i - 1) * width + j - 1] > maxGrad) {
//       maxGrad = gradImg[(i - 1) * width + j - 1];
//       dir1 = UP_LEFT;
//       dir2 = DOWN_RIGHT;
//     }
//     if (gradImg[(i - 1) * width + j + 1] > maxGrad) {
//       maxGrad = gradImg[(i - 1) * width + j + 1];
//       dir1 = UP_RIGHT;
//       dir2 = DOWN_LEFT;
//     }

//     if (gradImg[(i + 1) * width + j - 1] > maxGrad) {
//       maxGrad = gradImg[(i + 1) * width + j - 1];
//       dir1 = DOWN_LEFT;
//       dir2 = UP_RIGHT;
//     }
//     if (gradImg[(i + 1) * width + j + 1] > maxGrad) {
//       maxGrad = gradImg[(i + 1) * width + j + 1];
//       dir1 = DOWN_RIGHT;
//       dir2 = UP_LEFT;
//     }

//     int len1 = EDWalk(gradImg, map, GRADIENT_THRESH, i, j, dir1, tmpPixels);
//     int len2 =
//         EDWalk(gradImg, map, GRADIENT_THRESH, i, j, dir2, tmpPixels + len1);

//     if (len1 + len2 - 1 < MIN_PATH_LEN) continue;

//     map->segments[noSegments].pixels = map->pixels + totalLen;
//     int len = 0;
//     for (int k = len1 - 1; k >= 1; k--) {
//       map->segments[noSegments].pixels[len] = tmpPixels[k];
//       len++;
//     }  // end-for

//     for (int k = len1; k < len1 + len2; k++) {
//       map->segments[noSegments].pixels[len] = tmpPixels[k];
//       len++;
//     }  // end-for

//     map->segments[noSegments].noPixels = len;
//     noSegments++;
//     totalLen += len;
//   }  // end-for

//   map->noSegments = noSegments;

//   delete tmpPixels;
//   delete A;
// }  // end-JoinAnchorPointsUsingSortedAnchors

// ///----------------------------------------------------------------------------------------------
// /// Alternative ED. Just uses gradImg to linking the anchors.
// /// All points in the gradient map >= GRADIENT_THRESH are assumed to be anchors.
// /// Starts anchor linking with the pixel having the max. gradient value,
// /// and walks to the neighboring pixel having the greatest gradient value.
// /// Edge directions are NOT used during anchor linking
// ///
// EdgeMap *DoDetectEdgesByED(short *gradImg, int width, int height,
//                            int GRADIENT_THRESH) {
//   // Check parameters for sanity
//   if (GRADIENT_THRESH < 1) GRADIENT_THRESH = 1;
//   int MIN_PATH_LEN = 10;  // 10 pixels

//   // Edge map to be returned
//   EdgeMap *map = new EdgeMap(width, height);

//   /*------------ COMPUTE ANCHORS -------------------*/
//   memset(map->edgeImg, 0, width * height);

// #if 1
//   for (int i = 0; i < width * height; i++) {
//     if (gradImg[i] >= GRADIENT_THRESH) map->edgeImg[i] = ANCHOR_PIXEL;
//   }  // end-for
// #else
//   for (int i = 1; i < height - 1; i++) {
//     for (int j = 1; j < width - 1; j++) {
//       if (gradImg[i * width + j] < GRADIENT_THRESH) continue;

//       // Assume that a pixel is an anchor only if it is a local maxima in 8
//       // neighborhood
//       if (gradImg[(i - 1) * width + j - 1] > gradImg[i * width + j]) continue;
//       if (gradImg[(i - 1) * width + j] > gradImg[i * width + j]) continue;
//       if (gradImg[(i - 1) * width + j + 1] > gradImg[i * width + j]) continue;

//       if (gradImg[i * width + j - 1] > gradImg[i * width + j]) continue;
//       if (gradImg[i * width + j + 1] > gradImg[i * width + j]) continue;

//       if (gradImg[(i + 1) * width + j - 1] > gradImg[i * width + j]) continue;
//       if (gradImg[(i + 1) * width + j] > gradImg[i * width + j]) continue;
//       if (gradImg[(i + 1) * width + j + 1] > gradImg[i * width + j]) continue;

//       map->edgeImg[i * width + j] = ANCHOR_PIXEL;
//     }  // end-for
//   }    // end-for
// #endif

//   /*------------ JOIN ANCHORS -------------------*/
//   JoinAnchorPointsUsingSortedAnchors(gradImg, map, GRADIENT_THRESH,
//                                      MIN_PATH_LEN);

//   return map;
// }  // end-DoDetectEdgesByED