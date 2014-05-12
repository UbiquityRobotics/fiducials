// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "Bounding_Box.hpp"
#include "Memory.hpp"
#include <algorithm>

BoundingBox::BoundingBox() {
  reset();
}

/// @brief Resets the contents of *bounding_box* to empty.
/// @param bounding_box to be reset.
///
/// *Bounding_Box__reset*() will reset *bounding_box* to the empty state.

void BoundingBox::reset() {
    double big = 123456789.0;
    maximum_x = -big;
    minimum_x = big;
    maximum_y = -big;
    minimum_y = big;
}

/// @brief Adds the point (*x*, *y) to *bounding_box*.
/// @param bounding_box to be updated.
/// @param x is the X value to update.
/// @param y is the Y value to update.
///
/// *Bounding_Box__update*() will update the contents of *bounding_box* to
/// enclose (*x*, *y*).

void BoundingBox::update(double x, double y) {
    maximum_x = std::max(maximum_x, x);
    minimum_x = std::min(minimum_x, x);
    maximum_y = std::max(maximum_y, y);
    minimum_y = std::min(minimum_y, y);
}

