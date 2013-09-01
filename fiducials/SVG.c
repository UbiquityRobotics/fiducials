// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include "SVG.h"

/// @brief Close SVG object.
/// @param svg object to close
///
/// *SVG__close*() will close *svg*.

void SVG__close(SVG svg) {
    // Close *svg*:
    File svg_stream = svg->stream;
    File__format(svg_stream, "</svg>\n");
    File__close(svg_stream);
    svg->stream = (File)0;
}

/// @brief Draw a line from (*x1*, *y1*) to (*x2*, *y2*) using *stroke*.
/// @param svg is the *SVG* object to draw line to.
/// @param x1 is X coordinate of first point.
/// @param y1 is Y coordinate of first point.
/// @param x2 is X coordinate of second point.
/// @param y2 is Y coordinate of second point.
/// @param stroke is stroke string.
///
/// *SVG__line*() will draw a line to *svg* from (*x1*, *y1*) to (*x2*, *y2*)
/// using *stroke*.

void SVG__line(SVG svg,
  Double x1, Double y1, Double x2, Double y2, String stroke) {
    // Extract some values from *svg*:
    File svg_stream = svg->stream;
    Double x_offset = svg->x_offset;
    Double y_offset = svg->y_offset;
    Double x_scale = svg->x_scale;
    Double y_scale = svg->y_scale;
    String units = svg->units;

    // Output "<line ... />" to *svg_stream*:
    File__format(svg_stream,
      "<line x1=\"%f%s\" y1=\"%f%s\"",
      (x1 + x_offset) * x_scale, units, (y1 + y_offset) * y_scale, units);
    File__format(svg_stream, 
      " x2=\"%f%s\" y2=\"%f%s\"", 
      (x2 + x_offset) * x_scale, units, (y2 + y_offset) * y_scale, units);
    File__format(svg_stream,
       " style=\"stroke:%s\"/>\n", stroke);
}

/// @brief Open an SVG object:
/// @param base_name is the base name of the file name without the .svg.
/// @param width is the width of the SVG.
/// @param height is the height of the SVG.
/// @param x_scale is the amount is scale in X.
/// @param y_scale is the amount to scale in Y.
/// @param units is the units to use.
/// @returns a new *SVG* object.
///
/// *SVG__open*() will create and return a new *SVG* for writing out
/// scable vector graphics.

SVG SVG__open(String base_name,
  Double width, Double height, Double x_scale, Double y_scale, String units) {
    // Get *svg_stream* opened:
    SVG svg = (SVG)0;
    String file_name = String__format("%s.svg", base_name);
    File svg_stream = File__open(file_name, "w");
    if (svg_stream == (File)0) {
	File__format(stderr, "Unable to open %s.svg\n", base_name);
    } else {
        // Allocate and load up *svg*:
	svg = Memory__new(SVG);
	svg->height = height;
	svg->stream = svg_stream;
	svg->width = width;
	svg->units = units;
	svg->x_scale = x_scale;
	svg->y_scale = y_scale;
	svg->x_offset = 0.0;
	svg->y_offset = 0.0;

	// Ouput the header for *svg*:
	File__format(svg_stream,
	  "<?xml version=\"1.0\" standalone=\"no\"?>\n\n");
	File__format(svg_stream,
	  "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
	File__format(svg_stream,
          " \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n\n");
	File__format(svg_stream,
	  "<svg width=\"%f%s\" height=\"%f%s\"\n",
	  width * x_scale, units, height * y_scale, units);
	File__format(svg_stream,
	 " version=\"1.1\"\n");
	File__format(svg_stream,
	 " xmlns=\"http://www.w3.org/2000/svg\">\n\n");
    }

    // Clean up and return:
    String__free(file_name);
    return svg;
}

/// @brief Draw a *width* by *height* rectangle at (*x*, *y*).
/// @param svg is the *SVG* object to draw rectangle to.
/// @param x is the upper left X coordinate.
/// @param y is the uppper left Y coordinate.
/// @param width is the rectangle width.
/// @param height is the rectangle height.
/// @param stroke_color is the exterior line color.
/// @param fill_color is the interior fill color.
///
/// *SVG__rectangle* will draw a *width* by *height* rectangle at (*x*, *y*)
/// with *stroke_color* and *fill_color* specifying the external line color
/// and internal fill color respectivily.

void SVG__rectangle(SVG svg, Double x, Double y,
  Double width, Double height, String stroke_color, String fill_color) {
    // Grab some values from svg:
    File svg_stream = svg->stream;
    Double x_offset = svg->x_offset;
    Double y_offset = svg->y_offset;
    Double x_scale = svg->x_scale;
    Double y_scale = svg->y_scale;
    String units = svg->units;

    // Output "<rect ... />" to *svg_stream*:
    Double x_final = (x + x_offset) * x_scale;
    Double y_final = (y + y_offset) * y_scale;
    File__format(svg_stream, 
      "<rect x=\"%f%s\" y=\"%f%s\"", x_final, units, y_final, units);
    File__format(svg_stream,
      " width=\"%f%s\" height=\"%f%s\"",
      width * x_scale,  units, height * y_scale, units);
    File__format(svg_stream,
      " style=\"stroke:%s; fill:%s\"/>\n", stroke_color, fill_color);
}

/// @brief draw *message* at (*x*, *y*).
/// @param x is the X coordinate to draw text at.
/// @param y is the Y coordinate of draw text at.
/// @param font_size is the font size.
/// @param font_family is the font family to use.
///
/// *SVG__text*() will draw *message* at (*x*, *y*) with *font_size* font
/// of type *font_family*.

void SVG__text(SVG svg,
  String message, Double x, Double y, String font_family, Unsigned font_size) {
    // Grab some values from *svg*:
    File svg_stream = svg->stream;
    Double x_offset = svg->x_offset;
    Double y_offset = svg->y_offset;
    Double x_scale = svg->x_scale;
    Double y_scale = svg->y_scale;
    String units = svg->units;

    File__format(svg_stream,
      "<text x=\"%f%s\" y=\"%f%s\"",
      (x + x_offset) * x_scale, units, (y + y_offset) * y_scale, units);
    File__format(svg_stream,
      " style=\"font-family:%s; font-size:%d\">", font_family, font_size);
    File__format(svg_stream, "%s</text>\n", message);
}
