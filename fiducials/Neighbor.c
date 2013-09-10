// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved

#include "Float.h"
#include "File.h"
#include "Map.h"
#include "Neighbor.h"
#include "Tag.h"
#include "Unsigned.h"

// *Neighbor* routines:

/// @brief Return the sort order of *neighbor1* vs. *neighbor2*.
/// @param neihbor1 is the first *Neighbor* object.
/// @param neihbor2 is the first *Neighbor* object.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Neighbor__compare*() will return -1 if *neighbor1* sorts before
/// *neighbor2*, 0 if they are equal, and 1 if *neighbor1* sorts after
/// *neighbor2*.

Integer Neighbor__compare(Neighbor neighbor1, Neighbor neighbor2) {
    Integer result = Tag__compare(neighbor1->origin, neighbor2->origin);
    if (result != 0) {
	result = Tag__compare(neighbor1->target, neighbor2->target);
    }
    return result;
}

/// @brief Create and return a new *Neighbor* object.
/// @param origin *Tag*.
/// @param target *Tag*.
/// @param distance between *origin* and *target*.
/// @param target_angle is angle from *origin* center to *target* center.
/// @param target_twist is twist from *origin* to *target*. 
/// @returns new *Neighbor* object.
///
/// *Neighbor__create*() will create and return neighbor a new *Neighbor*
/// object that contains *origin*, *target*, *distance*, *target_angle*,
/// *target_twist* and *goodness*.  Both *origin* and *target* are *Tag*
/// objects.  *distance* is the distance between the fiducial centers
/// measured in the consistent set of distance units (e.g. mm, cm, meter, etc.)
/// *target_angle* is the the angle measued in radians from the floor X
/// axis to the line that connects *origin* to *target*.  *target_twist*
/// is the angle measured in radians from the bottom edge of *origin* to
/// the bottom edge of *target*.

Neighbor Neighbor__create(Tag origin, Tag target, Float distance,
  Float target_angle, Float target_twist, Float goodness) {
    Neighbor neighbor = Memory__new(Neighbor);
    neighbor->distance = distance;
    neighbor->goodness = goodness;
    neighbor->target = target;
    neighbor->origin = origin;
    neighbor->target_angle = target_angle;
    neighbor->target_twist = target_twist;
    return neighbor;
}

/// @brief Read in an XML <Neighbor.../> tag from *in_file*.
/// @param in_file is the file to read from.
/// @param map is contains the Tag associations.
/// @returns new *Neighbor* object.
///
/// *Neighbor__read*() will read in a <Neighbor.../> tag from *in_file*
/// and return the resulting *Neighbor* object.  *Tag* objects all looked
/// up using *map*.

Neighbor Neighbor__read(File in_file, Map map) {
    // Read <Neighbor ... /> tag:
    File__tag_match(in_file, "Neighbor");
    Unsigned origin_id =
      (Unsigned)File__integer_attribute_read(in_file, "Origin");
    Unsigned target_id =
      (Unsigned)File__integer_attribute_read(in_file, "Target");
    Float distance = File__float_attribute_read(in_file, "Distance");
    Float target_angle = File__float_attribute_read(in_file, "Target_Angle");
    Float target_twist = File__float_attribute_read(in_file, "Target_Twist");
    Float goodness = File__float_attribute_read(in_file, "Goodness");
    File__string_match(in_file, ">\n");

    // Create and load *neighbor*:
    Tag origin = Map__tag_lookup(map, origin_id);
    Tag target = Map__tag_lookup(map, target_id);
    Neighbor neighbor = Neighbor__create(origin,
      target, distance, target_angle, target_twist, goodness);
    return neighbor;
}

/// @brief Write *neighbor* out to *out_file* in XML format.
/// @param neighbor to be written out.
/// @param out_file to write ot.
///
/// *Neighbor__write*() will write *neighbor* out to *out_file* as an
/// <Neighbor .../> tag.

void Neighbor__write(Neighbor neighbor, File out_file) {
    Float pi = (Float)3.14159265;
    Float radians_to_degrees = 180.0 / pi;

    File__format(out_file, "  <Neighbor");
    File__format(out_file, " Origin=\"%d\"", neighbor->origin);
    File__format(out_file, " Target=\"%d\"", neighbor->target);
    File__format(out_file, " Distance=\"%f\"", neighbor->distance);
    File__format(out_file,
      " Target_Angle=\"%f\"", neighbor->target_angle * radians_to_degrees);
    File__format(out_file,
      " Target_Twist=\"%f\"", neighbor->target_angle * radians_to_degrees);
    File__format(out_file, " Goodness=\"%f\"", neighbor->goodness);
    File__format(out_file, " />\n");
}

