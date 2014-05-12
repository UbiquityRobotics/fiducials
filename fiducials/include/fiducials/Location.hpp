// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(LOCATION_H_INCLUDED)
#define LOCATION_H_INCLUDED 1

/// @brief *Location* represents an X/Y/Bearing location.

/// @brief A *Location_Struct* represents an X/Y/Bearing location.
class Location {
  public:
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

    Location(unsigned int _id, double _x, double _y, double _bearing, 
        double _goodness, unsigned int _index) : goodness(_goodness),
      id(_id), index(_index), bearing(_bearing), x(_x), y(_y) {}
};

#endif // !defined(LOCATION_H_INCLUDED)
