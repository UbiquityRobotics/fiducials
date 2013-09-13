// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Arc.h"
#include "Float.h"
#include "List.h"
#include "Tag.h"
#include "Unsigned.h"

// *Tag* routines:

/// @brief Appends *arc* to *tag*.
/// @param tag to append to.
/// @param arc to append.
///
/// *Tag__arc_append*() will append *arc* to *tag*.

void Tag__arc_append(Tag tag, Arc arc) {
    assert(arc->from == tag || arc->to == tag);
    List__append(tag->arcs, (Memory)arc);
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
/// @returns new *Tag*.
///
/// *Tag__create*() will create and return a *Tag* object with an identifier
/// of *id*.  This returned *Tag* is not *initialized* until *Tag__initialize*()
/// is called.

Tag Tag__create(Unsigned id, Map map) {
    Tag tag =  Memory__new(Tag);
    tag->twist = (Float)0.0;
    tag->arcs = List__new(); // <Arc>
    tag->initialized = (Logical)0;
    tag->id = id;
    tag->map = map;
    tag->visit = map->visit;
    tag->x = (Float)0.0;
    tag->y = (Float)0.0;
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

/// @brief Return a hash for *tag*.
/// @param *tag* to hash.
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
///
/// *Tag__initialize*() will initialize *tag* to contain *twist*, *x*,
/// and *y*.  *twist* is the fiducial twist relative to the floor X axis
/// measured in radians.  *x* and *y* are measured in any consistent set
/// of units (millimeters, centimeters, meters, inches, light seconds, etc.)

void Tag__initialize(Tag tag, Float twist, Float x, Float y, Unsigned visit) {
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
    Float twist = File__float_attribute_read(in_file, "Twist");
    Float x = File__float_attribute_read(in_file, "X");
    Float y = File__float_attribute_read(in_file, "Y");
    Unsigned hop_count =
      (Unsigned)File__integer_attribute_read(in_file, "Hop_Count");
    File__string_match(in_file, "/>\n");

    // Convert *twist* from *degrees_to_radians*:
    Float pi = (Float)3.14159265;
    Float degrees_to_radians = pi / 180.0;
    twist *= degrees_to_radians;

    // Load up *tag*:
    Tag tag = Map__tag_lookup(map, tag_id);
    Tag__initialize(tag, twist * degrees_to_radians, x, y, map->visit);
    tag->hop_count = hop_count;

    return tag;
}

/// @brief Sorts the contents of *tag*
/// @param tag to sort.
///
/// *Tag__sort*() will cause the contents of *tag*.

void Tag__sort(Tag tag) {
    List__sort(tag->arcs, (List__Compare__Routine)Arc__compare);
}

/// @brief Updates the position and orientation of *tag* using *arc*.
/// @param tag to update.
/// @param arc to use to find the arc to update from.
///
/// *Tag__update_via_arc*() will use the contents of *arc* to update
/// the position and oritation of *tag*.  The position is computed using
/// the "other" end of *arc*.

void Tag__update_via_arc(Tag tag, Arc arc) {
    // Some values to use for radian/degree conversion:
    Float pi = (Float)3.14159265;
    Float r2d = 180.0 / pi;

    // Read out *arc* contents:
    Tag arc_from = arc->from;
    Tag arc_to = arc->to;
    Float arc_distance = arc->distance;
    Float arc_angle = arc->angle;
    Float arc_twist = arc->twist;

    // For debugging:
    //File__format(stderr, "Arc[%d, %d]: (d=%.4f, a=%.1f, t=%.1f)\n",
    //  arc_from->id, arc_to->id,
    //  arc_distance, arc_angle * r2d, arc_twist * r2d);

    // Figure out whether *tag* is the *from* or *to* and adjust
    // angles accordingly:
    if (tag == arc_from) {
	// Compute the conjugate of *Arc* (see Arc.h for conjugate discussion):
	Tag temporary = arc_from;
	arc_from = arc_to;
	arc_to = temporary;
	arc_angle = Float__angle_normalize(pi + arc_angle - arc_twist);
	arc_twist = -arc_twist;

	// For debugging show the conjucate:
	//File__format(stderr, "Arc'[%d, %d]: (d=%.4f, a=%.1f, t=%.1f)\n",
	//  arc_from->id, arc->to->id,
	//  arc_distance, arc_angle * r2d, arc_twist * r2d);
    }
    assert (tag == arc_to);
    assert (tag != arc_from);

    // Grab the starting values from *arc_from*:
    Float arc_from_twist = arc_from->twist;
    Float arc_from_x = arc_from->x;
    Float arc_from_y = arc_from->y;

    // For debugging:
    //File__format(stderr, "Arc_From[%d]:(t=%.1f, x=%.1f, y=%.1f)\n",
    //  arc_from->id, arc_from_twist * r2d, arc_from_x, arc_from_y);

    // Compute the new *Tab* values:
    Float tag_twist = arc_from_twist + arc_twist;
    Float angle = Float__angle_normalize(arc_from_twist + arc_angle);
    Float tag_x = arc_from_x + arc_distance * Float__cosine(angle);
    Float tag_y = arc_from_y + arc_distance * Float__sine(angle);

    // For debugging:
    //File__format(stderr, "aa=%.1f aft=%.1f angle=%.1f\n",
    //  arc_angle * r2d, arc_from_twist * r2d, angle * r2d);
    //File__format(stderr, "Tag[%d]:(t=%.1f, x=%.1f, y=%.1f)\n\n",
    //  tag->id, tag_twist * r2d, tag_x, tag_y);

    // Load new values into *tag*:
    tag->twist = tag_twist;
    tag->x = tag_x;
    tag->y = tag_y;
}

/// @brief Writes *tag* out ot *out_file* in XML format.
/// @param *tag to write out.
/// @param *out_file* to write to.
///
/// *Tag__write*() will write *tag* out to *out_file* in XML format.

void Tag__write(Tag tag, File out_file) {
    // We store angles in degress and convert to/from radians.
    Float pi = (Float)3.14159265;
    Float radians_to_degrees = 180.0 / pi;

    // Write out "<Tag ... >":
    File__format(out_file, " <Tag");
    File__format(out_file, " Id=\"%d\"", tag->id);
    File__format(out_file,
      " Twist=\"%f\"", tag->twist * radians_to_degrees);
    File__format(out_file, " X=\"%f\"", tag->x);
    File__format(out_file, " Y=\"%f\"", tag->y);
    File__format(out_file, " Hop_Count=\"%d\"", tag->hop_count);
    File__format(out_file, "/>\n");
}

//void Tag__map_update(Tag tag, Float floor_x, Float floor_y,
//  Float floor_angle, Unsigned visit, Map map) {
//
//    // This routine will update {tag} to have a position of ({x},{y}) and
//    // a {bearing} angle relative to the floor coordinate X axis.
//    // This only happens if {tag.visit} is less than {visit}.  In addition,
//    // all arcs of {tag} are visited to update position and bearing.
//
//    Float pi = (Float)3.14159265;
//
//    // Do we need to update {tag}?:
//    if (tag->visit < visit) {
//	// Yes, we do.  Update the map fields:
//	Tag__initialize(tag, floor_angle, floor_x, floor_y, visit);
//
//	// Make sure that *arcs* is sorted:
//	List arcs = tag->arcs;
//	if (!tag->arcs_are_sorted) {
//	    // Sort *arcs* and remember that we did so:
//	    List__sort(arcs, (List__Compare__Routine)Arc__compare);
//	    tag->arcs_are_sorted = (Logical)1;
//	}
//
//	// Now visit each {Tag_Arc} in {arcs}:
//	Unsigned size = List__size(arcs);
//	for (Unsigned index = 0; index < size; index++) {
//	    Arc arc = (Arc)List__fetch(arcs, index);
//
//	    // {target_bearing} is the absolute direction of the bottom
//	    // edge of the target in floor coordinates:
//	    Float target_bearing =
//	      Float__angle_normalize(floor_angle + arc->target_twist);
//
//	    // We are at ({x}, {y}) and the new target is placed
//	    // at ({target_x}, {target_y}).
//	    Float target_angle =
//	      Float__angle_normalize(arc->target_angle + target_bearing);
//
//	    // Now we can compute ({target_x}, {target_y}):
//	    Float distance = arc->distance;
//	    Float target_x = floor_x + distance * Float__cosine(target_angle);
//	    Float target_y = floor_y + distance * Float__sine(target_angle);
//
//	    // Make sure ({target_x},{target_y}) is reasonable:
//	    if (target_x > 10000.0 || target_y > 10000.0) {
//		assert(0);
//	    } else {
//		Tag__map_update(arc->target, target_x, target_y,
//		  target_bearing, visit, map);
//	    }
//	}
//    }
//}

