// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(LOCATION_H_INCLUDED)
#define LOCATION_H_INCLUDED 1

/// @brief *Location* represents an X/Y/Bearing location.

typedef struct Location__Struct *Location;

/// @brief A *Location_Struct* represents an X/Y/Bearing location.
struct Location__Struct {
    /// @brief Goodness coefficient of location (closer to 0.0 is better.)
    double goodness;

    /// @brief Tag identifier.
    unsigned int id;

    /// @brief Index counter.
    unsigned int index;

    /// @brief Bearing in radians.
    double bearing;

    /// @brief X coordinate.
    double x;

    /// @brief Y coordinate.
    double y;

};

extern void Location__free(Location location);
extern Location Location__create(unsigned int id,
  double x, double y, double bearing, double goodness, unsigned int index);

#endif // !defined(LOCATION_H_INCLUDED)
