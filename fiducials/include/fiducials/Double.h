// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(DOUBLE_H_INCLUDED)
#define DOUBLE_H_INCLUDED 1


#include "Integer.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief *Double* is a double precision (64-bits) floating point number.
typedef double Double;
// *Double* routines:

extern Double Double__absolute(Double value);
extern Double Double__angle_between(Double from_angle, Double to_angle);
extern Double Double__angle_normalize(Double angle);
extern Double Double__arc_tangent2(Double y, Double x);
extern Double Double__maximum(Double a, Double b);
extern Double Double__minimum(Double a, Double b);
extern Integer Double__compare(Double double1, Double double2);
extern Double Double__cosine(Double angle);
extern Double Double__sine(Double angle);
extern Double Double__square_root(Double square);

#ifdef __cplusplus
}
#endif
#endif // !defined(DOUBLE_H_INCLUDED)

