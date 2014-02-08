// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Location.h"
#include "Double.h"
#include "Unsigned.h"
#include "Memory.h"

// *Location* routines:

/// @brief Create a new *Location* object.
/// @param id is the tag closest to the robot location.
/// @param x is the X cooridinate of the robot.
/// @param y is the X cooridinate of the robot.
/// @param bearing is the robot bearing.
/// @param goodness is how close the robot is to the tag.
/// @param index is the parent location list that contains this location.
/// @returns a new *Location* object
///
/// *Location__create*() create and initialize a new *Location* object that
/// contains *id*, *x*, *y*, *bearing*, *goodness*, and *index*.

Location Location__create(Unsigned id,
  Double x, Double y, Double bearing, Double goodness, Unsigned index) {
    Location location = Memory__new(Location, "Location__create");
    location->bearing = bearing;
    location->goodness = goodness;
    location->id = id;
    location->index = index;
    location->x = x;
    location->y = y;
    return location;
}

/// @brief Release memory for *location*.
/// @param location to free memory for.
///
/// *Location__free*() will release the storage for *location*.

void Location__free(Location location) {
  //Memory__free((Memory)location);
}


