// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(LOCATION_H_INCLUDED)
#define LOCATION_H_INCLUDED 1

#include "Double.hpp"

/// @brief *Location* represents an X/Y/Bearing location.

typedef struct Location__Struct *Location;

/// @brief A *Location_Struct* represents an X/Y/Bearing location.
struct Location__Struct {
    /// @brief Goodness coefficient of location (closer to 0.0 is better.)
    Double goodness;

    /// @brief Tag identifier.
    unsigned int id;

    /// @brief Index counter.
    unsigned int index;

    /// @brief Bearing in radians.
    Double bearing;

    /// @brief X coordinate.
    Double x;

    /// @brief Y coordinate.
    Double y;

};

extern void Location__free(Location location);
extern Location Location__create(unsigned int id,
  Double x, Double y, Double bearing, Double goodness, unsigned int index);

#endif // !defined(LOCATION_H_INCLUDED)
