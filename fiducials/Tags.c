// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "SVG.h"

Integer main(void) {
    File__format(stdout, "Hello!\n");
    return 0;
}

/// @brief Draw a rectangular section of bits:
/// @param cell_width is the cell width and height.
/// @param first_column is the first column of rectantle.
/// @param first_row is the first column of rectantle.
/// @param last_column is the last row to draw the bit at.
/// @param last_row is the last column to draw the bit at.
/// @param border is true if the tag has a border.
///
/// *SVG__tag_bits*() will draw a row or column of black bits from
/// (*first_column*, *first_row*) to (*last_column*, *last_row*)
/// the 12 x 12 matrix that makes up a tag to *svg*.  If *border*
/// *false*, the matrix is offset by the left by one.

void SVG__tag_bits(SVG svg, Double cell_width, Unsigned first_column,
  Unsigned first_row, Unsigned last_column, Unsigned last_row, Logical border) {

    // Deal with *border*:
    Unsigned delta = 0;
    if (!border) {
	delta = 1;
    }

    // Deal with SVG using left-hand (rather than right-hand) Cartesian
    // coordinate system:
    first_row = 11 - first_row;
    last_row = 11 - last_row;

    // Make sure *first_column* < *last_column*:
    if (first_column > last_column) {
	Unsigned temporary_column = first_column;
	first_column = last_column;
	last_column = temporary_column;
	}

    // Make sure *first_row* < *last_row*:
    if (first_row > last_row) {
	Unsigned temporary_row = first_row;
	first_row = last_row;
	last_row = temporary_row;
    }

    // Output the appropriate rectangle to *svg*:
    String color = "black";
    SVG__rectangle(svg,
      (Double)(first_column - delta) * cell_width,
      (Double)(first_row - delta) * cell_width,
      (Double)(last_column - first_column + 1) * cell_width,
      (Double)(last_row - first_row + 1) * cell_width,
      color, color);
}

/// @brief Draw a bit at *row* and *column*.
/// @param cell_width is the cell width and height.
/// @param row is the row to draw the bit at.
/// @param column is the column to draw the bit at.
/// @param border is true if the tag has a border.
///
/// *SVG__tag_bit* draw a black bit at *row* and *column* in the 12 x 12
/// matrix that makes up a tag and output it to *svg*.  If *border* is *false*
/// the matrix is offset to the left by one.

void SVG__tag_bit(SVG svg,
  Double cell_width, Unsigned row, Unsigned column, Logical border) {
    SVG__tag_bits(svg, cell_width, row, column, row, column, border);
}

