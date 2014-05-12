// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(SVG_H_INCLUDED)
#define SVG_H_INCLUDED 1

#include <assert.h>

#include "Bounding_Box.hpp"
#include "Double.hpp"
#include "File.hpp"
#include "Memory.hpp"
#include "Unsigned.hpp"

/// @brief *SVG* is a Scalable Vector Graphics object.
typedef struct SVG__Struct *SVG;

/// @brief *SVG_Struct* is the data structure for representing Scalable
/// Vector Graphics.

struct SVG__Struct {
    /// @brief The output stream.
    File stream;

    /// @brief Hight of SVG image.
    Double height;

    /// @brief Width of SVG image.
    Double width;

    /// @brief Units to use for SVG values.
    String_Const units;

    /// @brief Amount to offset X by.
    Double x_offset;

    /// @brief Amount to scale X by.
    Double x_scale;

    /// @brief Amount to offset Y by.
    Double y_offset;

    /// @brief Amount to scale Y by.
    Double y_scale;
};

// External declarations:

extern void SVG__cartesian_scale(
  SVG svg, Double x_width, Double y_height, Bounding_Box bounding_box);
extern void SVG__close(SVG svg);
extern void SVG__line(SVG svg,
  Double x1, Double y1, Double x2, Double y2, String_Const stroke);
extern SVG SVG__open(String_Const base_name,
  Double width, Double height, Double x_scale, Double y_scale,
  String_Const units);
extern void SVG__rectangle(SVG svg, Double x, Double y,
  Double width, Double height, String_Const stroke_color,
  String_Const fill_color);
extern void SVG__text(SVG svg,
  String_Const message, Double x, Double y, String_Const font_family,
  Unsigned font_size);


#endif // !defined(SVG_H_INCLUDED)
