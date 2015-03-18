// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved

#include <assert.h>

#include "Arc.hpp"
#include "File.hpp"
#include "Map.hpp"
#include "SVG.hpp"
#include "Tag.hpp"

// *Arc* routines:

/// @brief Return the sort order of *arc1* vs. *arc2*.
/// @param arc1 is the first *Arc* object.
/// @param arc2 is the first *Arc* object.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Arc__compare*() will return -1 if *arc1* sorts before *arc2*,
/// 0 if they are equal, and 1 if *arc1* sorts after *arc2*.

bool Arc::equal(Arc *arc1, Arc *arc2) {
    return Tag::equal(arc1->from_tag, arc2->from_tag) && 
      Tag::equal(arc1->to_tag, arc2->to_tag);
}

// return true if arc1 comees before arc2; else false
bool Arc::less(Arc *arc1, Arc *arc2) {
    if( Tag::less(arc1->from_tag, arc2->from_tag) ) {
      return true;
    } else if( Tag::equal(arc1->from_tag, arc2->from_tag) ) {
      return Tag::less(arc1->to_tag, arc2->to_tag);
    }
    return false;
}

/// @brief Create and return a new *Arc* object.
/// @param from_tag is the tag with the lower id.
/// @param from_twist is the amount the *from_tag* is twisted in radians.
/// @param distance between the tags.
/// @param to_tag is the tag with the higher id.
/// @param to_twist is the amount *to_tag* is twisted in radians.
/// @param goodness is the distance from camera center to tag point center.
/// @returns new *Arc* object
///
/// *Arc__create*() will create and return arc a new *Arc* object that
/// contains *from_tag*, *from_twist*, *distance*, *to_tag*, *to_twist*,
/// and *goodness*.

Arc::Arc(Tag *from_tag, double from_twist,
  double distance, Tag *to_tag, double to_twist, double goodness) {
    // Make sure *from* id is less that *to* id:
    if (from_tag->id > to_tag->id) {
        // Compute the conjugate *Arc* (see Arc.h):
        Tag *temporary_tag = from_tag;
        from_tag = to_tag;
        to_tag = temporary_tag;

        double temporary_twist = from_twist;
        from_twist = to_twist;
        to_twist = temporary_twist;
    }

    // Create and load *arc*:
    this->distance = distance;
    this->from_tag = from_tag;
    this->from_twist = from_twist;
    this->goodness = goodness;
    this->to_tag = to_tag;
    this->to_twist = to_twist;

    // Append *arc* to *from*, *to*, and *map*:
    from_tag->arc_append(this);
    to_tag->arc_append(this);
    Map__arc_append(from_tag->map, this);
}

/// @brief Return the distance sort order of *arc1* vs. *arc2*.
/// @param arc1 is the first *Arc* object.
/// @param arc2 is the second *Arc* object.
/// @returns true or false depending upon distance sort order.
///
/// *Arc__distance_compare*() will return true if the *arc1* distance is larger
/// than the *arc2* distance, otherwise false

bool Arc::distance_less(Arc *arc1, Arc *arc2) {
    if( arc1->distance > arc2->distance ) {
      // if arc1 has a greater distance, it should sort first(less)
      return true;
    } else if( arc1->distance == arc2->distance ) {
      unsigned int arc1_lowest_hop_count =
        std::min(arc1->from_tag->hop_count, arc1->to_tag->hop_count);
      unsigned int arc2_lowest_hop_count =
        std::min(arc2->from_tag->hop_count, arc2->to_tag->hop_count);
      if( arc1_lowest_hop_count > arc2_lowest_hop_count ) {
        // if distances are equal and arc1 has a greater hop count, it should
        // sort first(less)
        return true;
      }
    }
    return false;
}

/// @brief Returns a new *Arc* object.
/// @returns new *Arc* object.
///
/// *Arc__new*() will return a new *Arc*.

Arc::Arc() {
    distance = -1.0;
    from_tag = NULL;
    from_twist = 0.0;
    goodness = 123456789.0;
    in_tree = (bool)0;
    to_tag = NULL;
    to_twist = 0.0;
    visit = 0;
}

/// @brief Read in an XML <Arc.../> tag from *in_file*.
/// @param in_file is the file to read from.
/// @param map is contains the Tag associations.
/// @returns new *Arc* object.
///
/// *Arc__read*() will read in a <Arc.../> tag from *in_file*
/// and return the resulting *Arc* object.  *Tag* objects all looked
/// up using *map*.

Arc * Arc::read(File in_file, Map map) {
    // Read <Arc ... /> tag:
    File__tag_match(in_file, "Arc");
    unsigned int from_tag_id =
      (unsigned int)File__integer_attribute_read(in_file, "From_Tag_Id");
    double from_twist = File__double_attribute_read(in_file, "From_Twist");

    double distance = File__double_attribute_read(in_file, "Distance");
    unsigned int to_tag_id =
       (unsigned int)File__integer_attribute_read(in_file, "To_Tag_Id");
    double to_twist = File__double_attribute_read(in_file, "To_Twist");
    double goodness = File__double_attribute_read(in_file, "Goodness");
    bool in_tree = (bool)File__integer_attribute_read(in_file, "In_Tree");
    File__string_match(in_file, "/>\n");

    // Convert from degrees to radians:
    double pi = (double)3.14159265358979323846264;
    double degrees_to_radians = pi / 180.0;
    from_twist *= degrees_to_radians;
    to_twist *= degrees_to_radians;

    // Create and load *arc*:
    Tag * from_tag = Map__tag_lookup(map, from_tag_id);
    Tag * to_tag = Map__tag_lookup(map, to_tag_id);
    Arc * arc = Map__arc_lookup(map, from_tag, to_tag);
    
    // Load the data into *arc*:
    if (arc->goodness > goodness) {
        arc->update(from_twist, distance, to_twist, goodness);
        arc->in_tree = in_tree;
        Map__arc_announce(map, arc, (CV_Image)0, 0);
    }

    return arc;
}

/// @brief Draws *arc* into *svg*.
/// @param arc is the *Arc* to draw.
/// @param svg is the *SVG* object to draw it into.
///
/// *Arc__svg_write*() will draw *arc* into *svg*.

void Arc::svg_write(SVG *svg) {
    String_Const color = "green";
    if (in_tree) {
        color = "red";
    }
    svg->line(from_tag->x, from_tag->y, to_tag->x, to_tag->y, color);
}

/// @brief Updates the contents of *arc*.
/// @param arc to update.
/// @param from_twist is the amount the from tag is twisted in radians.
/// @param distance between the two tag centers.
/// @param to_twist is the amount the to tag is twisted in radians.
/// @param goodness the distence between the camera center and center point
///        between the two tag centers.
///
/// *Arc__update*() will load *from_twist*, *distance*, *to_twist*, and
/// *goodness* into *arc*.

void Arc::update(double from_twist, double distance, double to_twist,
    double goodness) {
  // Create and load *arc*:
  assert (this->from_tag->id < this->to_tag->id);
  this->from_twist = from_twist;
  assert (distance > 0.0);
  this->distance = distance;
  this->goodness = goodness;
  this->to_twist = to_twist;
}

/// @brief Write *arc* out to *out_file* in XML format.
/// @param arc to be written out.
/// @param out_file to write ot.
///
/// *Arc__write*() will write *arc* out to *out_file* as an
/// <Arc .../> tag.

void Arc::write(File out_file) {
    // We need to convert from radians to degrees:
    double pi = (double)3.14159265358979323846264;
    double radians_to_degrees = 180.0 / pi;
    double from_twist_degrees = from_twist * radians_to_degrees;
    double to_twist_degrees = to_twist * radians_to_degrees;

    // Output <Arc ... /> tag to *out_file*:
    File__format(out_file, " <Arc");
    File__format(out_file, " From_Tag_Id=\"%d\"", from_tag->id);
    File__format(out_file, " From_Twist=\"%f\"", from_twist_degrees);
    File__format(out_file, " Distance=\"%f\"", distance);
    File__format(out_file, " To_Tag_Id=\"%d\"", to_tag->id);
    File__format(out_file, " To_Twist=\"%f\"", to_twist_degrees);
    File__format(out_file, " Goodness=\"%f\"", goodness);
    File__format(out_file, " In_Tree=\"%d\"", in_tree);
    File__format(out_file, "/>\n");
}

