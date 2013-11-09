// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Location.h"
#include "Double.h"
#include "Unsigned.h"
#include "Memory.h"

// *Location* routines:

Location Location__create(
  Double x, Double y, Double bearing, Double goodness, Unsigned index) {
    Location location = Memory__new(Location);
    location->x = x;
    location->y = y;
    location->bearing = bearing;
    location->index = index;
    location->goodness = goodness;
    return location;
}


