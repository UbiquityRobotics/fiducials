// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(SVG_H_INCLUDED)
#define SVG_H_INCLUDED 1

#include <assert.h>

#include "Bounding_Box.hpp"
#include "File.hpp"
#include "Memory.hpp"

/// @brief *SVG_Struct* is the data structure for representing Scalable
/// Vector Graphics.

class SVG {
  private:
    /// @brief The output stream.
    File stream;

    /// @brief Hight of SVG image.
    double height;

    /// @brief Width of SVG image.
    double width;

    /// @brief Units to use for SVG values.
    String_Const units;

    /// @brief Amount to offset X by.
    double x_offset;

    /// @brief Amount to scale X by.
    double x_scale;

    /// @brief Amount to offset Y by.
    double y_offset;

    /// @brief Amount to scale Y by.
    double y_scale;

  public:
    SVG(String_Const base_name,
        double width, double height, double x_scale, double y_scale,
        String_Const units);

    ~SVG();
    void setOffsets(double x_offset, double y_offset);

    void cartesian_scale(double x_width, double y_height, BoundingBox *bounding_box);
    void line(double x1, double y1, double x2, double y2, String_Const stroke);
    void rectangle(double x, double y,
        double width, double height, String_Const stroke_color,
        String_Const fill_color);
    void text(String_Const message, double x, double y, String_Const font_family,
        unsigned int font_size);
};



#endif // !defined(SVG_H_INCLUDED)
