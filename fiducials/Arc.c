// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved

#include <assert.h>

#include "Arc.h"
#include "Double.h"
#include "File.h"
#include "Map.h"
#include "Tag.h"
#include "Unsigned.h"

// *Arc* routines:

/// @brief Return the sort order of *arc1* vs. *arc2*.
/// @param arc1 is the first *Arc* object.
/// @param arc2 is the first *Arc* object.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Arc__compare*() will return -1 if *arc1* sorts before *arc2*,
/// 0 if they are equal, and 1 if *arc1* sorts after *arc2*.

Integer Arc__compare(Arc arc1, Arc arc2) {
    Integer result = Tag__compare(arc1->from, arc2->from);
    if (result == 0) {
	result = Tag__compare(arc1->to, arc2->to);
    }
    return result;
}

/// @brief Return the distance sort order of *arc1* vs. *arc2*.
/// @param arc1 is the first *Arc* object.
/// @param arc2 is the second *Arc* object.
/// @returns -1, 0, or 1 depending upon distance sort order.
///
/// *Arc__distance_compare*() will return -1 if the *arc1* distance is larger
/// than the *arc2* distance, 0 if they are equal, and 1 otherwize.

Integer Arc__distance_compare(Arc arc1, Arc arc2) {
    Integer result = -Double__compare(arc1->distance, arc2->distance);
    if (result == 0) {
	Unsigned arc1_lowest_hop_count =
	  Unsigned__minimum(arc1->from->hop_count, arc1->to->hop_count);
	Unsigned arc2_lowest_hop_count =
	  Unsigned__minimum(arc2->from->hop_count, arc2->to->hop_count);
	result =
	  -Unsigned__compare(arc1_lowest_hop_count, arc2_lowest_hop_count);
    }
    return result;
}

/// @brief Create and return a new *Arc* object.
/// @param origin *Tag*.
/// @param target *Tag*.
/// @param distance between *origin* and *target*.
/// @param target_angle is angle from *origin* center to *target* center.
/// @param target_twist is twist from *origin* to *target*. 
/// @returns new *Arc* object.
///
/// *Arc__create*() will create and return arc a new *Arc* object that
/// contains *from*, *to*, *distance*, *angle*, *twist* and *goodness*.
/// Both *from* and *to* are *Tag* objects.  *distance* is the
/// distance between the fiducial centers measured in the consistent
/// set of distance units (e.g. mm, cm, meter, etc.)  *angle* is the
/// the angle measued in radians from the *from* "X Axis" (see Tag.h)
/// line that connects *to* to *frm*.  *twist* is the angle measures
/// amount that *to* is twisted relative to *from*.

Arc Arc__create(Tag from, Tag to,
  Double distance, Double angle, Double twist, Double goodness) {
    // Make sure *from* id is less that *to* id:
    if (from->id > to->id) {
        // Compute the conjugate *Arc* (see Arc.h):
	Tag temporary = from;
	from = to;
	to = temporary;
	Double pi = (Double)3.14159265358979323846264;
	angle = Double__angle_normalize(pi + angle - twist);
	twist = -twist;
    }

    // Create and load *arc*:
    Arc arc = Arc__new();
    arc->angle = angle;
    arc->distance = distance;
    arc->from = from;
    arc->goodness = goodness;
    arc->to = to;
    arc->twist = twist;

    // Append *arc* to *from*, *to*, and *map*:
    Tag__arc_append(from, arc);
    Tag__arc_append(to, arc);
    Map__arc_append(from->map, arc);

    return arc;
}

/// @brief Return true if *arc1* equals *arc2*.
/// @param arc1 is the first *Arc* object.
/// @param arc2 is the first *Arc* object.
/// @returns true if they are equal.
///
/// *Arc__equal*() will return true if *arc1* is equal to *arc2*.

Logical Arc__equal(Arc arc1, Arc arc2) {
    return (Logical)(Arc__compare(arc1, arc2) == 0);
}

/// @brief Returns a hash value for *arc*.
/// @param arc to hash.
/// @returns hash value for *arc*.
///
/// *Arc__hash*() will return a hash value for *arc*.

Unsigned Arc__hash(Arc arc) {
    return Tag__hash(arc->from) + Tag__hash(arc->to);
}

/// @brief Returns a new *Arc* object.
/// @returns new *Arc* object.
///
/// *Arc__new*() will return a new *Arc*.

Arc Arc__new(void) {
    Arc arc = Memory__new(Arc);
    arc->angle = 0.0;
    arc->distance = 0.0;
    arc->from = (Tag)0;
    arc->goodness = 123456789.0;
    arc->in_tree = (Logical)0;
    arc->to = (Tag)0;
    arc->twist = 0.0;
    arc->visit = 0;
    return arc;
}

/// @brief Read in an XML <Arc.../> tag from *in_file*.
/// @param in_file is the file to read from.
/// @param map is contains the Tag associations.
/// @returns new *Arc* object.
///
/// *Arc__read*() will read in a <Arc.../> tag from *in_file*
/// and return the resulting *Arc* object.  *Tag* objects all looked
/// up using *map*.

Arc Arc__read(File in_file, Map map) {
    // Read <Arc ... /> tag:
    File__tag_match(in_file, "Arc");
    Unsigned from_id = (Unsigned)File__integer_attribute_read(in_file, "From");
    Unsigned to_id = (Unsigned)File__integer_attribute_read(in_file, "To");
    Double distance = File__float_attribute_read(in_file, "Distance");
    Double angle = File__float_attribute_read(in_file, "Angle");
    Double twist = File__float_attribute_read(in_file, "Twist");
    Double goodness = File__float_attribute_read(in_file, "Goodness");
    Logical in_tree = (Logical)File__integer_attribute_read(in_file, "In_Tree");
    File__string_match(in_file, "/>\n");

    // Convert from degrees to radians:
    Double pi = (Double)3.14159265358979323846264;
    Double radians_to_degrees =  pi / 180.0;
    angle *= radians_to_degrees;
    twist *= radians_to_degrees;

    // Create and load *arc*:
    Tag from = Map__tag_lookup(map, from_id);
    Tag to = Map__tag_lookup(map, to_id);
    Arc arc = Arc__create(from, to, distance, angle, twist, goodness);
    arc->in_tree = in_tree;
    return arc;
}

/// @brief Updates the contenst of *arc*.
/// @param arc to update.
/// @param distance to load into *arc*.
/// @param angle to load into *arc*.
/// @param twist to load into *arc*.
/// @param goodness to load into *arc*.
///
/// *Arc__update*() will load *distance*, *angle*, *twist*, and *goodness*
/// into *arc*.

void Arc__update(Arc arc,
  Double distance, Double angle, Double twist, Double goodness) {
    // Create and load *arc*:
    assert (arc->from->id < arc->to->id);
    arc->angle = angle;
    arc->distance = distance;
    arc->goodness = goodness;
    arc->twist = twist;
}

/// @brief Write *arc* out to *out_file* in XML format.
/// @param arc to be written out.
/// @param out_file to write ot.
///
/// *Arc__write*() will write *arc* out to *out_file* as an
/// <Arc .../> tag.

void Arc__write(Arc arc, File out_file) {
    // We need to convert from radians to degrees:
    Double pi = (Double)3.14159265358979323846264;
    Double radians_to_degrees = 180.0 / pi;

    // Output <Arc ... /> tag to *out_file*:
    File__format(out_file, " <Arc");
    File__format(out_file, " From=\"%d\"", arc->from->id);
    File__format(out_file, " To=\"%d\"", arc->to->id);
    File__format(out_file, " Distance=\"%f\"", arc->distance);
    File__format(out_file, " Angle=\"%f\"", arc->angle * radians_to_degrees);
    File__format(out_file, " Twist=\"%f\"", arc->twist * radians_to_degrees);
    File__format(out_file, " Goodness=\"%f\"", arc->goodness);
    File__format(out_file, " In_Tree=\"%d\"", arc->in_tree);
    File__format(out_file, "/>\n");
}

