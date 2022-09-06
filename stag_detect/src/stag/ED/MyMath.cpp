#include <math.h>
#include "stag/ED/MyMath.h"

#define PI 3.14159265358979323846

///----------------------------------------------
/// Fast arctan2 using a lookup table
///
#define MAX_LUT_SIZE 1024
static double LUT[MAX_LUT_SIZE + 1];

double myAtan2(double yy, double xx) {
  static bool tableInited = false;
  if (!tableInited) {
    for (int i = 0; i <= MAX_LUT_SIZE; i++) {
      LUT[i] = atan((double)i / MAX_LUT_SIZE);
    }  // end-for

    tableInited = true;
  }  // end-if

  double y = fabs(yy);
  double x = fabs(xx);

#define EPSILON 0.0001
  if (x < EPSILON) {
    if (y < EPSILON)
      return 0.0;
    else
      return PI / 2;
  }  // end-if

  bool invert = false;
  if (y > x) {
    double t = x;
    x = y;
    y = t;
    invert = true;
  }  // end-if

  double ratio = y / x;
  double angle = LUT[(int)(ratio * MAX_LUT_SIZE)];

  if (xx >= 0) {
    if (yy >= 0) {
      // I. quadrant
      if (invert) angle = PI / 2 - angle;

    } else {
      // IV. quadrant
      if (invert == false)
        angle = PI - angle;
      else
        angle = PI / 2 + angle;
    }  // end-else

  } else {
    if (yy >= 0) {
      /// II. quadrant
      if (invert == false)
        angle = PI - angle;
      else
        angle = PI / 2 + angle;

    } else {
      /// III. quadrant
      if (invert) angle = PI / 2 - angle;
    }  // end-else
  }    // end-else

  return angle;
}  // end-myAtan2

///---------------------------------------------------------
/// Fast square root functions. Up to 6% error
///
float fastsqrt(float val) {
  union {
    int tmp;
    float val;
  } u;
  u.val = val;
  u.tmp -= 1 << 23; /* Remove last bit so 1.0 gives 1.0 */
  /* tmp is now an approximation to logbase2(val) */
  u.tmp >>= 1;      /* divide by 2 */
  u.tmp += 1 << 29; /* add 64 to exponent: (e+127)/2 =(e/2)+63, */
  /* that represents (e/2)-64 but want e/2 */
  return u.val;
}  // end-fastsqrt

///------------------------------------------------------------------
/// Fast square root functions -- This gives a better approximation: 3.5% error
///
float fastsqrt2(float f) {
  int *tmp = (int *)&f;
  (*tmp) = (1 << 29) + ((*tmp) >> 1) - (1 << 22) - 0x4C000;
  //  (*tmp) = (1<<29) + ((*tmp) >> 1) - (1<<22);
  return f;
}  // end-fastsqrt2

///------------------------------------------------------------------
/// Fast square root for a double variable
///
double fastsqrt(double y) {
  double x, z, tempf;
  unsigned long *tfptr = ((unsigned long *)&tempf) + 1;

  tempf = y;
  *tfptr = (0xbfcdd90a - *tfptr) >> 1; /* estimate of 1/sqrt(y) */
  x = tempf;
  z = y * 0.5;                       /* hoist out the �/2�    */
  x = (1.5 * x) - (x * x) * (x * z); /* iteration formula     */
  x = (1.5 * x) - (x * x) * (x * z);
  x = (1.5 * x) - (x * x) * (x * z);
  x = (1.5 * x) - (x * x) * (x * z);
  x = (1.5 * x) - (x * x) * (x * z);
  return x * y;
}  // end-fastsqrt

/* Copyright (C) 1997 by Vesa Karvonen. All rights reserved.
 **
 ** Use freely as long as my copyright is retained.
 */

#undef PI