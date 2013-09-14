// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <math.h>

#include "Double.h"

/// @brief returns the absolute value of *value*.
/// @param value to compute absolute value of.
/// @returns absolute value of *value*.
///
/// *Double__absolute*() will return the absolute value of *value*.

Double Double__absolute(Double value) {
    return fabs(value);
}

/// @brief returns the normalized difference of two angles.
/// @param from_angle is the starting angle.
/// @param to_angle is the ending angle.
/// @returns normalized angle difference.
///
/// *Double__angle_between*() returns the normalized angle of *to_angle*
/// minus *from_angle*.

Double Double__angle_between(Double from_angle, Double to_angle) {
   return Double__angle_normalize(to_angle - from_angle);
}

/// @brief Return an angle in radians between -pi and pi.
/// @param angle to normalize.
/// @returns normalized angle.
///
/// *Double__angle_normalize*() will normalize *angle* to be between -pi an pi.
/// It is assumed that *angle* is between -3pi and +3pi.

Double Double__angle_normalize(Double angle) {
    Double pi = (Double)3.14159265358979323846264;
    if (angle > pi) {
	angle -= pi + pi;
    } else if (angle < -pi) {
	angle += pi + pi;
    }
    return angle;
}

/// @brief Return the arc tangent of *y*/*x*.
/// @param y is the numerator.
/// @param x is the denominator.
/// @returns the arc tangent.
///
/// *Double__arc_tangent2*() will return the arc tangent of *y*/*x* orienting
/// the returned angle to point to (*x*, *y*) from the origin.

Double Double__arc_tangent2(Double y, Double x) {
    return atan2(y, x);
}

/// @brief Returns the sort order of *float1* to *float2*.
/// @param float1 is the first *Double* to compare.
/// @param float2 is the second *Double* to compare.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Double__compare*() will return -1 if *float1* is less than *float2*,
/// 0 if they are equal, and 1 otherwise.

Integer Double__compare(Double float1, Double float2) {
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
/// *Double__size*() returns the cosine of *angle* where *angle* is in units
/// of radians.

Double Double__cosine(Double angle) {
    return cos(angle);
}

/// @brief Return the sine of *angle*
/// @param angle in radians.
/// @returns sine of *angle*.
///
/// *Double__size*() returns the sine of *angle* where *angle* is in units
/// of radians.

Double Double__sine(Double angle) {
    return sin(angle);
}

/// @brief Return the square root of *square*.
/// @param square is the value to take the square root of.
/// @returns square root of *square*.
///
/// *Double__square_root*() will return the square root of *square*.

Double Double__square_root(Double square) {
    return sqrt(square);
}

