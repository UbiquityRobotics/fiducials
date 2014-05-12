// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <angles/angles.h>

#include "Arc.hpp"
#include "Bounding_Box.hpp"
#include "SVG.hpp"
#include "Tag.hpp"

// *Tag* routines:

/// @brief Appends *arc* to *tag*.
/// @param tag to append to.
/// @param arc to append.
///
/// *Tag__arc_append*() will append *arc* to *tag*.

void Tag::arc_append(Arc * arc) {
    assert(arc->from_tag == this || arc->to_tag == this);
    arcs_.push_back(arc);
}

/// @brief Updates *bounding_box* to include the 4 corners of tag.
/// @param tag is the tag to use to update *bounding_box*.
/// @param bounding_box is the *Bounding_Box* to update.
///
/// *Tab__bounding_box_update*() will ensure that the 4 corners of
/// *tag* will be enclosed by *bounding_box* when it comes time to
/// graph *tag*.

void Tag::bounding_box_update(BoundingBox *bounding_box) {
    double half_diagonal = world_diagonal / 2.0;
    bounding_box->update(x - half_diagonal , y - half_diagonal);
    bounding_box->update(x + half_diagonal , y + half_diagonal);
}

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

Tag::Tag(unsigned int id, Map map) :
    twist(0.0),
    diagonal(0.0),
    hop_count(0),
    id(id),
    initialized(false),
    map(map),
    visit(map->visit),
    x(0.0), y(0.0), 
    updated(true)
{
    TagHeight * tag_height = Map__tag_height_lookup(map, id);
    world_diagonal = tag_height->world_diagonal;
    z = tag_height->z;
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

/// @brief Read in an XML <Tag ...> from *in_file* using *map*.
/// @param in_file to read from.
/// @param map to use for *Tag* associations.
/// @returns *Tag* that was read in.
///
/// *Tag__read*() will read in an XML <Tag ...> from *in_file* using *map*
/// for *Tag* assoiciations.  The resulting *Tag* is returned.

Tag * Tag::read(File in_file, Map map) {
    // Read in "<Tag .../>":
    File__tag_match(in_file, "Tag");
    unsigned int tag_id = (unsigned int)File__integer_attribute_read(in_file, "Id");
    double diagonal = File__double_attribute_read(in_file, "Diagonal");
    double pi = (double)3.14159265358979323846264;
    double twist = File__double_attribute_read(in_file, "Twist");
    double x = File__double_attribute_read(in_file, "X");
    double y = File__double_attribute_read(in_file, "Y");
    unsigned int hop_count =
      (unsigned int)File__integer_attribute_read(in_file, "Hop_Count");
    File__string_match(in_file, "/>\n");

    // Convert *twist* from *degrees_to_radians*:
    double degrees_to_radians = pi / 180.0;
    twist *= degrees_to_radians;

    // Grab some additional information about *tag_id* from *map*:
    TagHeight * tag_height = Map__tag_height_lookup(map, tag_id);

    // Load up *tag*:
    Tag * tag = Map__tag_lookup(map, tag_id);
    tag->initialize(twist, x, y, diagonal, map->visit);
    tag->hop_count = hop_count;
    tag->z = tag_height->z;

    return tag;
}

/// @brief Writes *tag* out to *svg*.
/// @param tag to write out.
/// @param svg is the *SVG* object to use to write *tag* out to.
///
/// *Tag__svg_write*() will write *tag* out to *svg* in scalable vector
/// graphics format.

void Tag::svg_write(SVG svg) {
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
    SVG__line(svg, x1, y1, x2, y2, other_edge);
    SVG__line(svg, x2, y2, x3, y3, other_edge);
    SVG__line(svg, x3, y3, x4, y4, other_edge);
    SVG__line(svg, x4, y4, x1, y1, bottom_edge);

    // Plot the id number:
    //String id_text = String__format("%d", id);
    char id_text[20];
    (void)sprintf(id_text, "%d", id);
    SVG__text(svg, id_text, x, y, "ariel", 20);
}


/// @brief Updates the position and orientation of *tag* using *arc*.
/// @param tag to update.
/// @param arc to use to find the arc to update from.
/// @param image is the current image being processed.
/// @param sequence_number is the image sequence number.
///
/// *Tag__update_via_arc*() will use the contents of *arc* to update
/// the position and oritation of *tag*.  The position is computed using
/// the "other" end of *arc*.

void Tag::update_via_arc(Arc *arc, CV_Image image,
    unsigned int sequence_number) {
    // Some values to use for radian/degree conversion:
    double pi = (double)3.14159265358979323846264;
    double r2d = 180.0 / pi;

    // Read out *arc* contents:
    Tag * from_tag = arc->from_tag;
    double arc_from_twist = arc->from_twist;
    double distance = arc->distance;
    Tag * to_tag = arc->to_tag;
    double arc_to_twist = arc->to_twist;

    // For debugging:
    //File__format(stderr,
    //  "Tag__update_via_arc: Arc[%d, %d]: d=%.4f, ft=%.1f, tt=%.1f)\n",
    //  from_tag->id, to_tag->id,
    //  distance, arc_from_twist * r2d, arc_to_twist * r2d);

    // Figure out whether *tag* is the *from* or *to* and conjugate if needed:
    if (this == from_tag) {
        // Compute the conjugate of *Arc* (see Arc.h for conjugate discussion):
        Tag * temporary_tag = from_tag;
        from_tag = to_tag;
        to_tag = temporary_tag;
        double temporary_twist = arc_from_twist;
        arc_from_twist = arc_to_twist;
        arc_to_twist = temporary_twist;

        // For debugging show the conjucate:
        //File__format(stderr,
        //  "Tag__update_via_arc: Arc'[%d, %d]: (d=%.4f, a=%.1f, t=%.1f)\n",
        //  from_tag->id, from_tag->id,
        //  distance, arc_from_twist * r2d, arc_to_twist * r2d);
    }
    assert (this == to_tag);
    assert (this != from_tag);

    // Grab the starting values from *from_tag*:
    double from_tag_twist = from_tag->twist;
    double from_tag_x = from_tag->x;
    double from_tag_y = from_tag->y;
    //File__format(stderr,
    //  "Tag__update_via_arc: From: id=%d x=%.2f y=%.2f twist=%.4f\n",
    //  from_tag->id, from_tag_x, from_tag_y, from_tag_twist * r2d);

    // Compute the new *Tag* values:
    double angle = angles::normalize_angle(-arc_from_twist + from_tag_twist);
    double to_tag_x = from_tag_x + distance * cos(angle);
    double to_tag_y = from_tag_y + distance * sin(angle);
    double to_tag_twist = angles::normalize_angle(angle - pi + arc_to_twist);

    // If *to_tag* values are to change
    if (to_tag->twist != to_tag_twist ||
      to_tag->x != to_tag_x || to_tag->y != to_tag_y) {
        // Load new values into *to_tag*:
        to_tag->twist = to_tag_twist;
        to_tag->x = to_tag_x;
        to_tag->y = to_tag_y;
        to_tag->updated = (bool)1;

        // Let any interested party know that tag values changed.
        Map map = to_tag->map;
        Map__tag_announce(map, to_tag, (bool)1, image, sequence_number);
    }

    //File__format(stderr, "To_Tag[id:%d x:%.2f y:%.2f tw:%.4f] angle=%.4f\n",
    //  to_tag->id, to_tag->x, to_tag->y, to_tag->twist * r2d, angle * r2d);
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

// *Tag_Height* routines:

/// @brief Compares *tag_height1* with *tag_height2*.
/// @param tag_height1 is the first *Tag_Height* object to compare.
/// @param tag_height2 is the second *Tag_Height* object to compare.
/// @returns true or false depending upon the comparison.
///
/// *Tag_Height__less*() will return true if *tag_height1* sorts before
/// *tag_height2*, and false otherwise.

bool TagHeight::less(TagHeight * tag_height1, TagHeight * tag_height2)
{
    return tag_height1->first_id < tag_height2->last_id;
}

/// @brief Reads in an a <Tag_Height .../> from *xml_in_file*.
/// @param xml_in_file is the file to read from.
/// @returns the resulting *Tag_Height* object.
///
/// *Tag_Height__xml_read*() will read in the a <Tag_Height .../> from
/// *xml_in_file* and return the resulting *Tag_Height* object.

TagHeight * TagHeight::xml_read(File xml_in_file) {
    // Read in "<Tag_Height .../>":
    File__tag_match(xml_in_file, "Tag_Height");
    unsigned int first_id =
      (unsigned int)File__integer_attribute_read(xml_in_file, "First_Id");
    unsigned int last_id =
      (unsigned int)File__integer_attribute_read(xml_in_file, "Last_Id");

    double World_Diagonal = 
       File__double_attribute_read(xml_in_file, "World_Diagonal");

    double z = File__double_attribute_read(xml_in_file, "Z");

    File__string_match(xml_in_file, "/>\n");

    // Load up *tag_height*:
    TagHeight * tag_height = new TagHeight();
    tag_height->world_diagonal = World_Diagonal;
    tag_height->first_id = first_id;
    tag_height->last_id = last_id;
    tag_height->z = z;

    return tag_height;
}
