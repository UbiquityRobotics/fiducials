// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include "Float.h"
#include "List.h"
#include "Neighbor.h"
#include "Tag.h"
#include "Unsigned.h"

// *Tag* routines:

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
    if (result == 0) {
	List neighbors1 = tag1->neighbors;
	List neighbors2 = tag2->neighbors;
	Unsigned neighbors1_size = List__size(neighbors1);
	Unsigned neighbors2_size = List__size(neighbors2);
	result = Unsigned__compare(neighbors1_size, neighbors2_size);
	if (result == 0) {
	    for (Unsigned index = 0; index < neighbors1_size; index++) {
		Neighbor neighbor1 = (Neighbor)List__fetch(neighbors1, index);
		Neighbor neighbor2 = (Neighbor)List__fetch(neighbors2, index);
		result = Neighbor__compare(neighbor1, neighbor2);
		if (result != 0) {
		    break;
		}
	    }
	}
    }

    return result;
}


/// @brief Create and return a new *Tag*.
/// @param id is the tag identifier.
/// @returns new *Tag*.
///
/// *Tag__create*() will create and return a *Tag* object with an identifier
/// of *id*.  This returned *Tag* is not *initialized* until *Tag__initialize*()
/// is called.

Tag Tag__create(Unsigned id) {
    Tag tag =  Memory__new(Tag);
    tag->initialized = (Logical)0;
    tag->floor_angle = (Float)0.0;
    tag->floor_x = (Float)0.0;
    tag->floor_y = (Float)0.0;
    tag->id = id;
    tag->neighbors = List__new();
    return tag;
}

/// @brief Initialize *tag* contents.
/// @param tag to intialize.
/// @param
/// @param floor_x is the X coordinate of the fiducial center.
/// @param floor_y is the Y corrdinate of the fiducial center.
///
/// *Tag__initialize*() will initialize *tag* to contain *floor_angle*,
/// *floor_x*, and *floor_y*.  *floor_angle* is the angle from the
/// floor X axis to the bottom/// fiducial edge measured in radians.
/// *floor_x* and *floor_y* are measured in any consistent set of units
/// (millimeters, centimeters, meters, inches, light seconds, etc.)

void Tag__initialize(Tag tag, Float floor_angle, Float floor_x, Float floor_y) {
    tag->initialized = (Logical)1;
    tag->floor_angle = floor_angle;
    tag->floor_x = floor_x;
    tag->floor_y = floor_y;
}

/// @brief Read in an XML <Tag ...> from *in_file* using *map*.
/// @param in_file to read from.
/// @param map to use for *Tag* associations.
/// @returns *Tag* that was read in.
///
/// *Tag__read*() will read in an XML <Tag ...> from *in_file* using *map*
/// for *Tag* assoiciations.  The resulting *Tag* is returned.

Tag Tag__read(File in_file, Map map) {
    // We store angles in degress and convert to/from radians.
    Float pi = (Float)3.14159265;
    Float degrees_to_radians = pi / 180.0;
    
    // Read in "<Tag ...>":
    File__tag_match(in_file, "Tag");
    Unsigned tag_id = (Unsigned)File__integer_attribute_read(in_file, "Id");
    Float floor_angle = File__float_attribute_read(in_file, "Floor_Angle");
    Float floor_x = File__float_attribute_read(in_file, "Floor_X");
    Float floor_y = File__float_attribute_read(in_file, "Floor_Y");
    Unsigned neighbors_size =
      File__integer_attribute_read(in_file, "Neighbors_Count");
    File__string_match(in_file, ">\n");

    // Load up *tag*:
    Tag tag = Map__tag_lookup(map, tag_id);
    Tag__initialize(tag, floor_angle * degrees_to_radians, floor_x, floor_y);

    // Read in the neighbors:
    List neighbors = tag->neighbors;
    for (Unsigned index = 0; index < neighbors_size; index++) {
	Neighbor neighbor = Neighbor__read(in_file, map);
	List__append(neighbors, neighbor);
    }

    // Read in "</Tag>":
    File__tag_match(in_file, "/Tag");
    File__string_match(in_file, ">\n");
    return tag;
}

/// @brief Sorts the contents of *tag*
/// @param tag to sort.
///
/// *Tag__sort*() will cause the contents of *tag*.

void Tag__sort(Tag tag) {
    List__sort(tag->neighbors, (List__Compare__Routine)Neighbor__compare);
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

    // Get the *size* of *neighbors*:
    List neighbors = tag->neighbors;
    Unsigned size = List__size(neighbors);

    // Write out "<Tag ... >":
    File__format(out_file, " <Tag");
    File__format(out_file, " Id=\"%d\"", tag->id);
    File__format(out_file,
      " Floor_Angle=\"%f\"", tag->floor_angle * radians_to_degrees);
    File__format(out_file, " Floor_X=\"%f\"", tag->floor_x);
    File__format(out_file, " Floor_Y=\"%f\"", tag->floor_y);
    File__format(out_file, " Neighbors_Count=\"%d\"", size);
    File__format(out_file, ">\n");

    // Write out each *neighbor*:
    for (Unsigned index = 0; index < size; index++) {
	Neighbor neighbor = (Neighbor)List__fetch(neighbors, index);
	Neighbor__write(neighbor, out_file);
    }

    // Write out the closing "</Tag>":
    File__format(out_file, " </Tag>\n");
}


