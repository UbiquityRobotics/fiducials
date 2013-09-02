// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "Double.h"
#include "CRC.h"
#include "FEC.h"
#include "Logical.h"
#include "String.h"
#include "SVG.h"
#include "Unsigned.h"

extern void SVG__tag_write(/* Extractor extractor, */
  Unsigned tag_id, Unsigned tag_size, Logical border);
extern Integer main(Unsigned arguments_size, String arguments[]);
extern void SVG__tag_bit(SVG svg,
  Double cell_width, Unsigned row, Unsigned column, Logical border);
extern void SVG__tag_write(/* Extractor extractor, */
  Unsigned tag_id, Unsigned tag_size, Logical border);

Integer main(Unsigned arguments_size, String arguments[]) {
    if (arguments_size <= 1) {
        File__format(stderr, "Usage: tag_id...\n");
    } else {
	Logical border = 1;
	Unsigned tag_size = 160;
	for (Unsigned index = 1; index < arguments_size; index++) {
	    String tag_name = arguments[index];
	    Unsigned tag_number = String__to_unsigned(tag_name);
	    //File__format(stdout,
	    //  "[%d]: '%s' %d\n", index, tag_name, tag_number);
	    SVG__tag_write(tag_number, tag_size, border);
	}
    }
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

// This routine will write out an SVG file for {tag_id} that is
// {tag_size} millimeters square.  {border} specifies whether there
// is a black line drawn around the "white" border of the tag.

void SVG__tag_write(/* Extractor extractor, */
  Unsigned tag_id, Unsigned tag_size, Logical border) {

    Double cell_width = (Double)(tag_size) / 10.0;
    //Double offset = cell_width / 2.0;
    Double offset = 5.0;
    Double length = 10.0 * cell_width;
    Double length_plus = length + 5.0 * cell_width;

    // Open the file for writing:
    String base_name = String__format("tag%d", tag_id);
    SVG svg = SVG__open(base_name,
      length + 3.0 * cell_width, length_plus, 1.0, 1.0, "mm");
    assert (svg != (SVG)0);
    svg->x_offset = offset;
    svg->y_offset = offset + cell_width;

    // Initialize {tag_bytes} to contain 8 bytes of 0:
    Unsigned tag_bytes[8];
    for (Unsigned index = 0; index < 8; index++) {
	tag_bytes[index] = 0;
    }

    // Place the tag id into the tag id buffer.
    Unsigned id = tag_id;
    tag_bytes[1] = (id >> 8) & 0xff;
    tag_bytes[0] = id & 0xff;

    // Calculate the 16-bit CCITT CRC portion of the tag id buffer:
    Unsigned crc = CRC__compute(tag_bytes, 2);
    tag_bytes[3] = (crc >> 8) & 0xff;
    tag_bytes[2] = crc & 0xff;

    // Calculate the FEC portion of the tag id buffer:
    FEC fec = FEC__create(8, 4, 4);
    FEC__parity(fec, tag_bytes, 8);

    // Print a line border around everything:
    if (border) {
	Double x_or_y = length + 2.0 * cell_width;
	Double d = 2.0;

	String color = "black";
	Double x1 = 0.0;
	Double x2 = x_or_y;

	//  +--                                   --+
	Double y = -cell_width;
	SVG__line(svg, x1, y, x1 + d, y,     color);
	SVG__line(svg, x2, y, x2 - d, y,     color);

	//  +--                                   --+
	//  |                                       |
	y = 0.0;
	SVG__line(svg, x1, y, x1 + d, y,     color);
	SVG__line(svg, x1, y, x1,     y + d, color);
	SVG__line(svg, x2, y, x2 - d, y,     color);
	SVG__line(svg, x2, y, x2,     y + d, color);

	//  |                                       |
	//  +--                                   --+
	y = x_or_y;
	SVG__line(svg, x1, y, x1 + d, y,     color);
	SVG__line(svg, x1, y, x1,     y - d, color);
	SVG__line(svg, x2, y, x2 - d, y,     color);
	SVG__line(svg, x2, y, x2,     y - d, color);

	//  +--                                   --+
	y = x_or_y + cell_width;
	SVG__line(svg, x1, y, x1 + d, y,     color);
	SVG__line(svg, x2, y, x2 - d, y,     color);
    }

    // Print the bit border:
    // Lower row:
    SVG__tag_bits(svg, cell_width, 1, 1, 9, 1, border);
    // Right column:
    SVG__tag_bits(svg, cell_width, 10, 1, 10, 9, border);
    // Upper row:
    SVG__tag_bits(svg, cell_width, 2, 10, 10, 10, border);
    // Left column:
    SVG__tag_bits(svg, cell_width, 1, 2, 1, 10, border);

    // Print the tag data:
    for (Unsigned index = 0; index < 64; index++) {
	Unsigned row = (index >> 3) & 7;
	Unsigned column  = index & 7;
	Unsigned tag_byte = tag_bytes[row];
	if ((tag_byte & (1 << column)) != 0) {
	    SVG__tag_bit(svg, cell_width, column + 2, row + 2, border);
	}
    }

    // Put some text on the page:
    String tag_name = String__format("%d", tag_id);
    if (border) {
	SVG__text(svg, tag_name,
	  6.0 * cell_width, 12.25 * cell_width,
	  "ariel", (Unsigned)(cell_width) / 2);
    } else {
	SVG__text(svg, tag_name,
	  5.0 * cell_width, 12.25 * cell_width,
	  "ariel", (Unsigned)(cell_width) / 2);
    }
    String__free(tag_name);

    // Close *svg*:
    SVG__close(svg);
}

