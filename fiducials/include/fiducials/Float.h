// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FLOAT_H_INCLUDED)
#define FLOAT_H_INCLUDED 1


#include "Integer.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief *Float* is a float precision (64-bits) floating point number.
typedef float Float;
// *Float* routines:

extern Float Float__angle_normalize(Float angle);
extern Integer Float__compare(Float float1, Float float2);
extern Float Float__cosine(Float angle);
extern Float Float__sine(Float angle);
extern Float Float__square_root(Float square);

#ifdef __cplusplus
}
#endif
#endif // !defined(FLOAT_H_INCLUDED)

