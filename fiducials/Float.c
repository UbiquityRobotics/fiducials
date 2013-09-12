// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <math.h>

#include "Float.h"

/// @brief Return an angle in radians between -pi and pi.
/// @param angle to normalize.
/// @returns normalized angle.
///
/// *Float__angle_normalize*() will normalize *angle* to be between -pi an pi.
/// It is assumed that *angle* is between -3pi and +3pi.

Float Float__angle_normalize(Float angle) {
    Float pi = (Float)3.14159265;
    if (angle > pi) {
	angle -= pi + pi;
    } else if (angle < -pi) {
	angle += pi + pi;
    }
    return angle;
}

/// @brief Returns the sort order of *float1* to *float2*.
/// @param float1 is the first *Float* to compare.
/// @param float2 is the second *Float* to compare.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Float__compare*() will return -1 if *float1* is less than *float2*,
/// 0 if they are equal, and 1 otherwise.

Integer Float__compare(Float float1, Float float2) {
    Integer result = 0;
    if (float1 < float2) {
        result = -1;
    } else if (float1 > float2) {
      result = 1;
    }
    return result;
}

/// @brief Return the cosine of *angle*
/// @param angle in radians.
/// @returns cosine of *angle*.
///
/// *Float__size*() returns the cosine of *angle* where *angle* is in units
/// of radians.

Float Float__cosine(Float angle) {
    return cosf(angle);
}

/// @brief Return the sine of *angle*
/// @param angle in radians.
/// @returns sine of *angle*.
///
/// *Float__size*() returns the sine of *angle* where *angle* is in units
/// of radians.

Float Float__sine(Float angle) {
    return sinf(angle);
}

/// @brief Return the square root of *square*.
/// @param square is the value to take the square root of.
/// @returns square root of *square*.
///
/// *Float__square_root*() will return the square root of *square*.

Float Float__square_root(Float square) {
    return sqrtf(square);
}
