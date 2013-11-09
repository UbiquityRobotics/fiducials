// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(LOCATION_H_INCLUDED)
#define LOCATION_H_INCLUDED 1

/// @brief *Location* represents an X/Y/Bearing location.

typedef struct Location__Struct *Location;

#include "Double.h"
#include "Unsigned.h"

/// @brief A *Location_Struct* represents an X/Y/Bearing location.
struct Location__Struct {
    /// @brief X coordinate.
    Double x;

    /// @brief Y coordinate.
    Double y;

    /// @brief Bearing in radians.
    Double bearing;

    /// @brief Index counter;
    Unsigned index;

    Double goodness;
};

extern Location Location__create(
  Double x, Double y, Double bearing, Double goodness, Unsigned index);

#endif // !defined(LOCATION_H_INCLUDED)
