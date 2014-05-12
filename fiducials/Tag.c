// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Arc.h"
#include "Bounding_Box.h"
#include "Double.h"
#include "List.h"
#include "SVG.h"
#include "Tag.h"
#include "Unsigned.h"

// *Tag* routines:

/// @brief Appends *arc* to *tag*.
/// @param tag to append to.
/// @param arc to append.
///
/// *Tag__arc_append*() will append *arc* to *tag*.

void Tag__arc_append(Tag tag, Arc arc) {
    assert(arc->from_tag == tag || arc->to_tag == tag);
    List__append(tag->arcs, (Memory)arc, "Tag__arc_append:List__append:arcs");
}

/// @brief Updates *bounding_box* to include the 4 corners of tag.
/// @param tag is the tag to use to update *bounding_box*.
/// @param bounding_box is the *Bounding_Box* to update.
///
/// *Tab__bounding_box_update*() will ensure that the 4 corners of
/// *tag* will be enclosed by *bounding_box* when it comes time to
/// graph *tag*.

void Tag__bounding_box_update(Tag tag, Bounding_Box bounding_box) {
    Double x = tag->x;
    Double y = tag->y;
    Double half_diagonal = tag->world_diagonal / 2.0;
    Bounding_Box__update(bounding_box, x - half_diagonal , y - half_diagonal);
    Bounding_Box__update(bounding_box, x + half_diagonal , y + half_diagonal);
}

/// @brief Returns the sort order of *tag1* to *tag2*.
/// @param tag1 is the first *Tag* to compare.
/// @param tag2 is the second *Tag* to compare.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Tag__compare*() will return -1 if *tag1* sorts before *tag2*, 0 if they
/// are equal, and 1 if *tag1* sorts after *tag2*.

Integer Tag__compare(Tag tag1, Tag tag2) {
    Unsigned id1 = tag1->id;
    Unsigned id2 = tag2->id;
    Integer result = Unsigned__compare(id1, id2);
    return result;
}

/// @brief Create and return a new *Tag*.
/// @param id is the tag identifier.
/// @param map is the map associated with the tag.
/// @returns new *Tag*.
///
/// *Tag__create*() will create and return a *Tag* object with an identifier
/// of *id*.  This returned *Tag* is not *initialized* until *Tag__initialize*()
/// is called.

Tag Tag__create(Unsigned id, Map map) {
    Tag_Height tag_height = Map__tag_height_lookup(map, id);
    Tag tag =  Memory__new(Tag, "Tag__create");
    tag->twist = (Double)0.0;
    tag->arcs = List__new("Tag__create:List__new:arcs"); // <Arc>
    tag->diagonal = 0.0;
    tag->hop_count = 0;
    tag->id = id;
    tag->initialized = (Logical)0;
    tag->map = map;
    tag->world_diagonal = tag_height->world_diagonal;
    tag->visit = map->visit;
    tag->x = (Double)0.0;
    tag->y = (Double)0.0;
    tag->z = tag_height->z;
    tag->updated = (Logical)1;
    return tag;
}

/// @brief Return true if *tag1* is equal to *tag2*.
/// @param tag1 is the first *Tag* to compare.
/// @param tag2 is the second *Tag* to compare.
/// @returns true if *tag1* equals *tag2*.
///
/// *Tag__equal*() will return true if *tag1* is equal to *tag2* and false
/// otherwise.

Logical Tag__equal(Tag tag1, Tag tag2) {
    return (Logical)(tag1->id == tag2->id);
}

/// @brief Releases *Tag* storage.
/// @param tag to release storage of.
///
/// *Tag__free*() will release the storage of *tag*.

void Tag__free(Tag tag) {
    //List arcs /* <Arc> */ = tag->arcs;
    //Unsigned arcs_size = List__size(arcs);
    //for (Unsigned index = 0; index < arcs_size; index++) {
    //	Arc arc = (Arc)List__fetch(arcs, index);
    //	Arc__free(arc);
    //}
    List__free(tag->arcs);
    Memory__free((Memory)tag);
}

/// @brief Return a hash for *tag*.
/// @param tag to hash.
/// @returns hash of *tag*.
///
/// *Tag__hash*() will return a hash of *tag*.

Unsigned Tag__hash(Tag tag) {
    return Unsigned__hash(tag->id);
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

void Tag__initialize(
  Tag tag, Double twist, Double x, Double y, Double diagonal, Unsigned visit) {
    tag->diagonal = diagonal; 
    tag->initialized = (Logical)1;
    tag->twist = twist;
    tag->x = x;
    tag->y = y;
    tag->visit = visit;
}

/// @brief Read in an XML <Tag ...> from *in_file* using *map*.
/// @param in_file to read from.
/// @param map to use for *Tag* associations.
/// @returns *Tag* that was read in.
///
/// *Tag__read*() will read in an XML <Tag ...> from *in_file* using *map*
/// for *Tag* assoiciations.  The resulting *Tag* is returned.

Tag Tag__read(File in_file, Map map) {
    // Read in "<Tag .../>":
    File__tag_match(in_file, "Tag");
    Unsigned tag_id = (Unsigned)File__integer_attribute_read(in_file, "Id");
    Double diagonal = File__double_attribute_read(in_file, "Diagonal");
    Double pi = (Double)3.14159265358979323846264;
    Double twist = File__double_attribute_read(in_file, "Twist");
    Double x = File__double_attribute_read(in_file, "X");
    Double y = File__double_attribute_read(in_file, "Y");
    Unsigned hop_count =
      (Unsigned)File__integer_attribute_read(in_file, "Hop_Count");
    File__string_match(in_file, "/>\n");

    // Convert *twist* from *degrees_to_radians*:
    Double degrees_to_radians = pi / 180.0;
    twist *= degrees_to_radians;

    // Grab some additional information about *tag_id* from *map*:
    Tag_Height tag_height = Map__tag_height_lookup(map, tag_id);

    // Load up *tag*:
    Tag tag = Map__tag_lookup(map, tag_id);
    Tag__initialize(tag, twist, x, y, diagonal, map->visit);
    tag->hop_count = hop_count;
    tag->z = tag_height->z;

    return tag;
}

/// @brief Sorts the contents of *tag*
/// @param tag to sort.
///
/// *Tag__sort*() will cause the contents of *tag*.

void Tag__sort(Tag tag) {
    List__sort(tag->arcs, (List__Compare__Routine)Arc__compare);
}

/// @brief Writes *tag* out to *svg*.
/// @param tag to write out.
/// @param svg is the *SVG* object to use to write *tag* out to.
///
/// *Tag__svg_write*() will write *tag* out to *svg* in scalable vector
/// graphics format.

void Tag__svg_write(Tag tag, SVG svg) {
  // Some constants:
    Double pi = (Double)3.14159265358979323846264;
    Double half_pi = pi / 2.0;
    Double quarter_pi = half_pi / 2.0;

    // Grab some values from *tag*:
    Unsigned id = tag->id;
    Double half_diagonal = tag->world_diagonal / 2.0;
    Double x = tag->x;
    Double y = tag->y;
    Double twist = tag->twist - quarter_pi;

    // Compute the 4 corners:
    double x1 = x + half_diagonal * Double__cosine(twist);
    double y1 = y + half_diagonal * Double__sine(twist);
    double x2 = x + half_diagonal * Double__cosine(twist + half_pi);
    double y2 = y + half_diagonal * Double__sine(twist + half_pi);
    double x3 = x + half_diagonal * Double__cosine(twist + pi);
    double y3 = y + half_diagonal * Double__sine(twist + pi);
    double x4 = x + half_diagonal * Double__cosine(twist - half_pi);
    double y4 = y + half_diagonal * Double__sine(twist - half_pi);

    // Plot the 4 sides:
    String other_edge = "black";
    String bottom_edge = "purple";
    SVG__line(svg, x1, y1, x2, y2, other_edge);
    SVG__line(svg, x2, y2, x3, y3, other_edge);
    SVG__line(svg, x3, y3, x4, y4, other_edge);
    SVG__line(svg, x4, y4, x1, y1, bottom_edge);

    // Plot the id number:
    //String id_text = String__format("%d", id);
    char id_text[20];
    (void)sprintf(id_text, "%d", id);
    SVG__text(svg, id_text, x, y, "ariel", 20);
    //String__free(id_text);
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

void Tag__update_via_arc(
  Tag tag, Arc arc, CV_Image image, Unsigned sequence_number) {
    // Some values to use for radian/degree conversion:
    Double pi = (Double)3.14159265358979323846264;
    Double r2d = 180.0 / pi;

    // Read out *arc* contents:
    Tag from_tag = arc->from_tag;
    Double arc_from_twist = arc->from_twist;
    Double distance = arc->distance;
    Tag to_tag = arc->to_tag;
    Double arc_to_twist = arc->to_twist;

    // For debugging:
    //File__format(stderr,
    //  "Tag__update_via_arc: Arc[%d, %d]: d=%.4f, ft=%.1f, tt=%.1f)\n",
    //  from_tag->id, to_tag->id,
    //  distance, arc_from_twist * r2d, arc_to_twist * r2d);

    // Figure out whether *tag* is the *from* or *to* and conjugate if needed:
    if (tag == from_tag) {
	// Compute the conjugate of *Arc* (see Arc.h for conjugate discussion):
	Tag temporary_tag = from_tag;
	from_tag = to_tag;
	to_tag = temporary_tag;
	Double temporary_twist = arc_from_twist;
	arc_from_twist = arc_to_twist;
	arc_to_twist = temporary_twist;

	// For debugging show the conjucate:
	//File__format(stderr,
	//  "Tag__update_via_arc: Arc'[%d, %d]: (d=%.4f, a=%.1f, t=%.1f)\n",
	//  from_tag->id, from_tag->id,
	//  distance, arc_from_twist * r2d, arc_to_twist * r2d);
    }
    assert (tag == to_tag);
    assert (tag != from_tag);

    // Grab the starting values from *from_tag*:
    Double from_tag_twist = from_tag->twist;
    Double from_tag_x = from_tag->x;
    Double from_tag_y = from_tag->y;
    //File__format(stderr,
    //  "Tag__update_via_arc: From: id=%d x=%.2f y=%.2f twist=%.4f\n",
    //  from_tag->id, from_tag_x, from_tag_y, from_tag_twist * r2d);

    // Compute the new *Tag* values:
    Double angle = Double__angle_normalize(-arc_from_twist + from_tag_twist);
    Double to_tag_x = from_tag_x + distance * Double__cosine(angle);
    Double to_tag_y = from_tag_y + distance * Double__sine(angle);
    Double to_tag_twist = Double__angle_normalize(angle - pi + arc_to_twist);

    // If *to_tag* values are to change
    if (to_tag->twist != to_tag_twist ||
      to_tag->x != to_tag_x || to_tag->y != to_tag_y) {
	// Load new values into *to_tag*:
	to_tag->twist = to_tag_twist;
	to_tag->x = to_tag_x;
	to_tag->y = to_tag_y;
        to_tag->updated = (Logical)1;

	// Let any interested party know that tag values changed.
	Map map = to_tag->map;
	Map__tag_announce(map, to_tag, (Logical)1, image, sequence_number);
    }

    //File__format(stderr, "To_Tag[id:%d x:%.2f y:%.2f tw:%.4f] angle=%.4f\n",
    //  to_tag->id, to_tag->x, to_tag->y, to_tag->twist * r2d, angle * r2d);
}

/// @brief Writes *tag* out ot *out_file* in XML format.
/// @param tag to write out.
/// @param out_file to write to.
///
/// *Tag__write*() will write *tag* out to *out_file* in XML format.

void Tag__write(Tag tag, File out_file) {
    // We store angles in degress and convert to/from radians.
    Double pi = (Double)3.14159265358979323846264;
    Double radians_to_degrees = 180.0 / pi;

    // Write out "<Tag ... >":
    File__format(out_file, " <Tag");
    File__format(out_file, " Id=\"%d\"", tag->id);
    File__format(out_file, " Diagonal=\"%f\"", tag->diagonal);
    File__format(out_file,
      " Twist=\"%f\"", tag->twist * radians_to_degrees);
    File__format(out_file, " X=\"%f\"", tag->x);
    File__format(out_file, " Y=\"%f\"", tag->y);
    File__format(out_file, " Hop_Count=\"%d\"", tag->hop_count);
    File__format(out_file, "/>\n");
}

// *Tag_Height* routines:

/// @brief Compares *tag_height1* with *tag_height2*.
/// @param tag_height1 is the first *Tag_Height* object to compare.
/// @param tag_height2 is the second *Tag_Height* object to compare.
/// @returns -1, 0, or 1 depending upon the comparison.
///
/// *Tag_Height__compare*() will return -1 if *tag_height1* sorts before
/// *tag_height2*, 0 if they are equal, and 1 otherwise.

Integer Tag_Height__compare(Tag_Height tag_height1, Tag_Height tag_height2)
{
    Integer result =
      Unsigned__compare(tag_height1->first_id, tag_height2->last_id);
    assert (result != 0);
    return result;
}

/// @brief Releases stoarge for *tag_height*.
/// @param tag_height to release storage of.
///
/// *Tag_Height__free*() will release the storate for *tag_height*.

void Tag_Height__free(Tag_Height tag_height) {
    Memory__free((Memory)tag_height);
}

/// @brief Reads in an a <Tag_Height .../> from *xml_in_file*.
/// @param xml_in_file is the file to read from.
/// @returns the resulting *Tag_Height* object.
///
/// *Tag_Height__xml_read*() will read in the a <Tag_Height .../> from
/// *xml_in_file* and return the resulting *Tag_Height* object.

Tag_Height Tag_Height__xml_read(File xml_in_file) {
    // Read in "<Tag_Height .../>":
    File__tag_match(xml_in_file, "Tag_Height");
    Unsigned first_id =
      (Unsigned)File__integer_attribute_read(xml_in_file, "First_Id");
    Unsigned last_id =
      (Unsigned)File__integer_attribute_read(xml_in_file, "Last_Id");

    Double World_Diagonal = 
       File__double_attribute_read(xml_in_file, "World_Diagonal");

    Double z = File__double_attribute_read(xml_in_file, "Z");

    File__string_match(xml_in_file, "/>\n");

    // Load up *tag_height*:
    Tag_Height tag_height = Memory__new(Tag_Height, "Tag_Height__xml_read");
    tag_height->world_diagonal = World_Diagonal;
    tag_height->first_id = first_id;
    tag_height->last_id = last_id;
    tag_height->z = z;

    return tag_height;
}
