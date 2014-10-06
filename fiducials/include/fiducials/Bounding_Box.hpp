// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(BOUNDING_BOX_H_INCLUDED)
#define BOUNDING_BOX_H_INCLUDED 1


class BoundingBox {
  private:
    double maximum_x;
    double minimum_x;
    double maximum_y;
    double minimum_y;

  public:
    // @brief Create a new empty *BoundingBox* object
    BoundingBox();

    // @brief Resets the contents of this *BoundingBox* to empty
    void reset();

    // @brief Add the point (*x*, *y*) to this *BoundingBox*
    // @param x is the X value to update
    // @param y is the Y value to update
    void update(double x, double y);

    double min_x() { return minimum_x; }
    double max_x() { return maximum_x; }
    double min_y() { return minimum_y; }
    double max_y() { return maximum_y; }
};

#endif // !defined(BOUNDING_BOX_H_INCLUDED)
