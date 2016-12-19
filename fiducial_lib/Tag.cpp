// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <angles/angles.h>

#include "Bounding_Box.hpp"
#include "SVG.hpp"
#include "Tag.hpp"


/// @brief Returns the sort order of *tag1* to *tag2*.
/// @param tag1 is the first *Tag* to compare.
/// @param tag2 is the second *Tag* to compare.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Tag__compare*() will return -1 if *tag1* sorts before *tag2*, 0 if they
/// are equal, and 1 if *tag1* sorts after *tag2*.

int Tag::equal(Tag *tag1, Tag *tag2) {
  return tag1->id == tag2->id;
}

bool Tag::less(Tag *tag1, Tag *tag2) {
  return tag1->id < tag2->id;
}

/// @brief Create and return a new *Tag*.
/// @param id is the tag identifier.
/// @param map is the map associated with the tag.
/// @returns new *Tag*.
///
/// *Tag__create*() will create and return a *Tag* object with an identifier
/// of *id*.  This returned *Tag* is not *initialized* until *Tag__initialize*()
/// is called.

Tag::Tag(unsigned int id) :
    twist(0.0),
    diagonal(0.0),
    hop_count(0),
    id(id),
    initialized(false),
    x(0.0), y(0.0), 
    updated(true)
{

}

/// @brief Initialize *tag* contents.
/// @param tag to intialize.
/// @param twist is the fidicial twist relative to the floor X axis.
/// @param x is the floor X coordinate of the fiducial center.
/// @param y is the floor Y corrdinate of the fiducial center.
/// @param diagonal is diagonal distance across the *tag*.
/// @param visit is the visit number for the tree walker.
///
/// *Tag__initialize*() will initialize *tag* to contain *twist*, *x*,
/// and *y*.  *twist* is the fiducial twist relative to the floor X axis
/// measured in radians.  *x*, *y*, are measured in any
/// consistent set of units (millimeters, centimeters, meters, inches,
/// light years, etc.)  *visit* is used for the tree walker.
/// *diagonal* is in pixels.

void Tag::initialize(double twist, double x, double y, double diagonal,
    unsigned int visit) {
    this->diagonal = diagonal; 
    this->initialized = (bool)1;
    this->twist = twist;
    this->x = x;
    this->y = y;
    this->visit = visit;
}

/// @brief Writes *tag* out to *svg*.
/// @param tag to write out.
/// @param svg is the *SVG* object to use to write *tag* out to.
///
/// *Tag__svg_write*() will write *tag* out to *svg* in scalable vector
/// graphics format.

void Tag::svg_write(SVG *svg) {
  // Some constants:
    double pi = (double)3.14159265358979323846264;
    double half_pi = pi / 2.0;
    double quarter_pi = half_pi / 2.0;

    // Grab some values from *tag*:
    double half_diagonal = world_diagonal / 2.0;
    double twist = this->twist - quarter_pi;

    // Compute the 4 corners:
    double x1 = x + half_diagonal * cos(twist);
    double y1 = y + half_diagonal * sin(twist);
    double x2 = x + half_diagonal * cos(twist + half_pi);
    double y2 = y + half_diagonal * sin(twist + half_pi);
    double x3 = x + half_diagonal * cos(twist + pi);
    double y3 = y + half_diagonal * sin(twist + pi);
    double x4 = x + half_diagonal * cos(twist - half_pi);
    double y4 = y + half_diagonal * sin(twist - half_pi);

    // Plot the 4 sides:
    String_Const other_edge = "black";
    String_Const bottom_edge = "purple";
    svg->line(x1, y1, x2, y2, other_edge);
    svg->line(x2, y2, x3, y3, other_edge);
    svg->line(x3, y3, x4, y4, other_edge);
    svg->line(x4, y4, x1, y1, bottom_edge);

    // Plot the id number:
    //String id_text = String__format("%d", id);
    char id_text[20];
    (void)sprintf(id_text, "%d", id);
    svg->text(id_text, x, y, "ariel", 20);
}

/// @brief Writes *tag* out ot *out_file* in XML format.
/// @param tag to write out.
/// @param out_file to write to.
///
/// *Tag__write*() will write *tag* out to *out_file* in XML format.

void Tag::write(File out_file) {
    // We store angles in degress and convert to/from radians.
    double pi = (double)3.14159265358979323846264;
    double radians_to_degrees = 180.0 / pi;

    // Write out "<Tag ... >":
    File__format(out_file, " <Tag");
    File__format(out_file, " Id=\"%d\"", id);
    File__format(out_file, " Diagonal=\"%f\"", diagonal);
    File__format(out_file,
      " Twist=\"%f\"", twist * radians_to_degrees);
    File__format(out_file, " X=\"%f\"", x);
    File__format(out_file, " Y=\"%f\"", y);
    File__format(out_file, " Hop_Count=\"%d\"", hop_count);
    File__format(out_file, "/>\n");
}