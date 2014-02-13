// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

/// @brief Global map of ceiling fiducial markers.
///
/// A *Map* consists of a list of *Tag* objects, where each Tag represents
/// the position and orientation of ceiling fiduical markers.  Each *Tag*
/// has zero one or more *Neighbor* objects that specify the distance and
/// orientation of *Tag* pairs.

typedef struct Map__Struct *Map_Doxygen_Fake_Out;

#include <assert.h>

#include "Arc.h"
#include "CV.h"
#include "Camera_Tag.h"
#include "Integer.h"
#include "File.h"
#include "List.h"
#include "Location.h"
#include "Map.h"
#include "Tag.h"
#include "Table.h"
#include "Unsigned.h"

// *Map* routines:

/// @brief Causes an arc announce callback routine to be called.
/// @param map is the parent *Map* object.
/// @param arc is the *Arc* object that has just been changed.
///
/// *Map__arc_announce*() will cause the arc announde call back routine
/// to be called for *arc*.

void Map__arc_announce(Map map,
  Arc arc, CV_Image image, Unsigned sequence_number) {
   Tag from_tag = arc->from_tag;
   Tag to_tag = arc->to_tag;
   map->arc_announce_routine(map->announce_object,
     from_tag->id, from_tag->x, from_tag->y, from_tag->z,
     to_tag->id, to_tag->x, to_tag->y, to_tag->z,
     arc->goodness, arc->in_tree);
   Map__image_log(map, image, sequence_number);
}

/// @brief Appends *arc* to *map*.
/// @param map to append to.
/// @param arc to append
///
/// *Map__arc_append*() will append *arc* to *map*.

void Map__arc_append(Map map, Arc arc) {
    List__append(map->all_arcs, arc, "Map__arc_append:List__append:all_arcs");
    map->changes_count += 1;
    map->is_changed = (Logical)1;
    map->is_saved = (Logical)0;
}

/// @brief Returns the *Arc* that contains *from_tag* and *to_tag*.
/// @param map that has the *Arc* table.
/// @param from_tag is the from *Tag*.
/// @param to_tag is the to *Tag*.
/// @returns the corresponding *Arc* object.
///
/// *Map__arc_lookup*() will return the *Arc* that contains *from_tag*
/// and *to_tag*.  If no such *Arc* exists yet, it is created.

Arc Map__arc_lookup(Map map, Tag from_tag, Tag to_tag) {
    // Make sure that *from_tag* has the lower id:
    if (from_tag->id > to_tag->id) {
	Tag temporary_tag = from_tag;
	from_tag = to_tag;
	to_tag = temporary_tag;
    }

    // See whether or not an *Arc* with these two tags preexists:
    Arc temporary_arc = map->temporary_arc;
    temporary_arc->from_tag = from_tag;
    temporary_arc->to_tag = to_tag;
    Table /* <Arc, Arc> */ arcs_table = map->arcs_table;
    Arc arc = (Arc)Table__lookup(arcs_table, (Memory)temporary_arc);
    if (arc == (Arc)0) {
	// No preexisting *Arc*; create one:
        arc = Arc__create(from_tag, 0.0, 0.0, to_tag, 0.0, 123456789.0);
	Table__insert(arcs_table, (Memory)arc, (Memory)arc);
    }
    return arc;
}

/// @brief Makes sure the *Arc* connecting *from* to *to* is up to date.
/// @brief map to use for *Arc* updating.
/// @brief from is the *Camera_Tag* to for one end of the *Arc*.
/// @brief to is the *Camera_Tag* to the other end of the *Arc*.
/// @brief image is the image that the *Camera_Tag*'s came from.
/// @breif sequence_number is the image sequence number.
/// @returns the number of *Arc*'s updated (1 or 0).
///
/// *Map__arc_update*() will create or update the *Arc* in *map* associated
/// with *from* and *to*.  *image* used to determine the frame size.

Unsigned Map__arc_update(Map map, Camera_Tag camera_from, Camera_Tag camera_to,
  CV_Image image, Unsigned sequence_number) {
    // Get the *width* and *height*:
    Integer rows = CV_Image__height_get(image);
    Integer columns = CV_Image__width_get(image);
    Double height = (Double)rows;
    Double width = (Double)columns;

    // Compute some constants:
    Double half_width = width / 2.0;
    Double half_height = height / 2.0;
    Double pi = 3.14159265358979323846264;
    Double r2d = 180.0 / pi;

    // Extract some field values from *camera_from*:
    Tag from_tag = camera_from->tag;
    Double camera_from_twist = camera_from->twist;
    Double camera_from_x = camera_from->x;
    Double camera_from_y = camera_from->y;

    // Extract some values from *from_tag*:
    Tag to_tag = camera_to->tag;
    Double camera_to_twist = camera_to->twist;
    Double camera_to_x = camera_to->x;
    Double camera_to_y = camera_to->y;

    // Find associated *Arc* that contains *from_tag* and *to_tag*:
    Arc arc = Map__arc_lookup(map, from_tag, to_tag);

    // Compute the polar distance (in pixels) and angle from the camera
    // center to the *from_tag* center:
    Double camera_from_dx = camera_from->x - half_width;
    Double camera_from_dy = camera_from->y - half_height;
    Double camera_from_polar_distance = Double__square_root(
      camera_from_dx * camera_from_dx + camera_from_dy * camera_from_dy);
    Double camera_from_polar_angle =
      Double__arc_tangent2(camera_from_dy, camera_from_dx);

    // Compute the polar_distance (in pixels) and angle from the camera
    // center to the *to_tag* center:
    Double camera_to_dx = camera_to_x - half_width;
    Double camera_to_dy = camera_to_y - half_height;
    Double camera_to_polar_distance = Double__square_root(
     camera_to_dx * camera_to_dx + camera_to_dy * camera_to_dy);
    Double camera_to_polar_angle =
      Double__arc_tangent2(camera_to_dy, camera_to_dx);

    // To minimize camera distortion effects, we want to use images where
    // *from* and *to* are about equidistant from the image center.  Thus,
    // we want to minimum the absolute value of the distance difference:
    Double goodness =
      Double__absolute(camera_from_polar_distance - camera_to_polar_distance);

    // Now see if the new *goodness* is better than the previous one:
    //File__format(stderr,
    //  "goodness=%.4f arc_goodness=%.4f\n", goodness, arc->goodness);
    Unsigned changed = 0;
    if (goodness < arc->goodness) {
	// We have a better *goodness* metric, compute the new values to
	// load into *arc*:

	// Get two *distance_from_pixel* values which may not be
	// the same because the fiducials are at different heights:
        Double from_distance_per_pixel = from_tag->distance_per_pixel;
        Double to_distance_per_pixel = to_tag->distance_per_pixel;

	// Now compute floor to/from X/Y's that coorrespond to the (X,Y)
	// projection of each tag center onto the floor as if the camera
	// is located at the floor origin:
	Double from_floor_x = from_distance_per_pixel *
	  camera_from_polar_distance * Double__cosine(camera_from_polar_angle);
	Double from_floor_y = from_distance_per_pixel *
	  camera_from_polar_distance * Double__sine(camera_from_polar_angle);
	Double to_floor_x = to_distance_per_pixel *
	  camera_to_polar_distance * Double__cosine(camera_to_polar_angle);
	Double to_floor_y = to_distance_per_pixel *
	  camera_to_polar_distance * Double__sine(camera_to_polar_angle);

	// Now we can compute the floor distance between the two two
	// projected points:
	Double floor_dx = from_floor_x - to_floor_x;
	Double floor_dy = from_floor_y - to_floor_y;
	Double floor_distance =
	  Double__square_root(floor_dx * floor_dx + floor_dy * floor_dy);

	// Compute *angle* to line segment connecting both tags:
	Double camera_dx = camera_to_x - camera_from_x;
	Double camera_dy = camera_to_y - camera_from_y;
 	Double arc_angle = Double__arc_tangent2(camera_dy, camera_dx);
	Double from_twist =
	  Double__angle_normalize(camera_from_twist - arc_angle);
	Double to_twist =
	  Double__angle_normalize(camera_to_twist + pi - arc_angle);

	// OLD: Compute the distance between *origin* and *to*:
	//Double distance_per_pixel = from_tag->distance_per_pixel;
	//Double camera_distance =
	//  Double__square_root(camera_dx * camera_dx + camera_dy * camera_dy);
	//Double old_floor_distance = camera_distance * distance_per_pixel;
	//File__format(stderr, "floor_distance=%.2f old_floor_distance=%.2f\n",
	//  floor_distance, old_floor_distance);

	//File__format(stderr,
	//  "Map__arc_update: camera_from_twist=%.2f camera_to_twist=%.2f\n",
 	//  camera_from_twist * r2d, camera_to_twist * r2d);
	//File__format(stderr,
	//  "Map__arc_update: arc_angle=%.2f from_twist=%.2f to_twist=%.2f\n",
	//  arc_angle * r2d, from_twist * r2d, to_twist * r2d);

	// Finally, upate *arc*:
	Arc__update(arc, from_twist, floor_distance, to_twist, goodness);
	map->changes_count += 1;
	map->is_changed = (Logical)1;
	map->is_saved = (Logical)0;

	// Let interested parties know that *arc* has been updated:
	Map__arc_announce(map, arc, image, sequence_number);

	changed = 1;
    }
    return changed;
}

/// @brief Returns -1, 0, 1 depending upon the sort order of *map1* and *map2*.
/// @param map1 is the first *Map* to compare.
/// @param map2 is the second *Map* to compare.
/// @returns sort order.
///
/// *Map__compare*() will compare *map1* to *map2* and return -1 if *map1*
/// sorts before *map2*, 0 if they are equal, and -1 if *map1* would sort
/// after *map2*.  Realistically, this routine is only used for testing
/// equality.

Integer Map__compare(Map map1, Map map2) {
    Integer result = 0;

    // First make sure all of the *Tag*'s match up:
    List /* <Tag> */ all_tags1 = map1->all_tags;
    List /* <Tag> */ all_tags2 = map2->all_tags;
    Unsigned all_tags1_size = List__size(all_tags1);
    Unsigned all_tags2_size = List__size(all_tags2);
    result = Unsigned__compare(all_tags1_size, all_tags2_size);
    if (result == 0) {
	// Visit each *Tag*:
	for (Unsigned index = 0; index < all_tags1_size; index++) {
	    Tag tag1 = (Tag)List__fetch(all_tags1, index);
	    Tag tag2 = (Tag)List__fetch(all_tags2, index);
	    result = Tag__compare(tag1, tag2);
	    if (result != 0) {
		break;
	    }
	}
    }

    // Second make sure all of the *Arc*'s match up:
    List /* <Tag> */ all_arcs1 = map1->all_arcs;
    List /* <Tag> */ all_arcs2 = map2->all_arcs;
    Unsigned all_arcs1_size = List__size(all_arcs1);
    Unsigned all_arcs2_size = List__size(all_arcs2);
    result = Unsigned__compare(all_arcs1_size, all_arcs2_size);
    if (result == 0) {
	// Visit each *Arc*:
	for (Unsigned index = 0; index < all_arcs1_size; index++) {
	    Arc arc1 = (Arc)List__fetch(all_arcs1, index);
	    Arc arc2 = (Arc)List__fetch(all_arcs2, index);
	    result = Arc__compare(arc1, arc2);
	    if (result != 0) {
		break;
	    }
	}
    }
    return result;
}

/// @brief Returns a new *Map*.
/// @param map_file_name is the name of the Map .xml file
/// @param announce_object is an opaque object that is passed into announce
///        routines.
/// @param arc_announce_routine is the arc callback routine.
/// @param tag_announce_routine is the tag callback routine.
/// @param tag_heights_file_name is the tag ceiling heights .xml file.
/// @param from is used for memory leak checking.
/// @returns a new *Map*.
///
/// *Map__create*() creates and returns an empty initialized *Map* object.

Map Map__create(String_Const map_file_name, void *announce_object,
  Fiducials_Arc_Announce_Routine arc_announce_routine,
  Fiducials_Tag_Announce_Routine tag_announce_routine,
  String_Const tag_heights_file_name, String from) {
    // Create and fill in *map*:
    Map map = Memory__new(Map, from);
    map->arc_announce_routine = arc_announce_routine;
    map->all_arcs = List__new("Map__new:List_New:all_arcs"); // <Tag>
    map->all_tags = List__new("Map__new:List_New:all_tags"); // <Tag>
    map->announce_object = announce_object;
    map->arcs_table = Table__create((Table_Equal_Routine)Arc__equal,
      (Table_Hash_Routine)Arc__hash, (Memory)0,
      "Map__new:Table__create:map_arcs_table"); // <Arc, Arc>
    map->changes_count = 0;
    map->file_name = map_file_name;
    map->is_changed = (Logical)0;
    map->is_saved = (Logical)1;
    map->image_log = (Logical)0;
    map->pending_arcs = List__new("Map__new:List__new:pending_arcs"); // <Tag>
    map->tag_announce_routine = tag_announce_routine;
    map->tag_heights =
      List__new("Map__new:List__new:tag_heights"); // <Tag_Height>
    map->tags_table = Table__create((Table_Equal_Routine)Unsigned__equal,
      (Table_Hash_Routine)Unsigned__hash, (Memory)0,
      "Map__new:Table__create:map_tags_table");
      // <Unsigned, Tag>
    map->temporary_arc = Arc__new("Map__new:Arc__New:temporary_arc");
    map->visit = 0;

    // Read in the contents of *map_heights_file_name* into *map*:
    Map__tag_heights_xml_read(map, tag_heights_file_name);

    // Restore *map* if appropriate:
    File in_file = File__open(map_file_name, "r");
    if (in_file != (File)0) {
        Map__restore(map, in_file);
	File__close(in_file);
    }
    return map;
}

/// @brief Releases storage associated with *map*.
/// @param map to release storage for.
///
/// *Map__free*() will release the storage associaed with *map*.

void Map__free(Map map) {
    // Save the map:
    Map__save(map);

    // Release all the *Arc*'s:
    List /* <Arc> */ all_arcs = map->all_arcs;
    Unsigned arcs_size = List__size(all_arcs);
    for (Unsigned index = 0; index < arcs_size; index++) {
	Arc arc = (Arc)List__fetch(all_arcs, index);
	Arc__free(arc);
    }
    List__free(all_arcs);
    Arc__free(map->temporary_arc);

    // Release all the *Tag*'s:
    List /* <Tap> */ all_tags = map->all_tags;
    Unsigned tags_size = List__size(all_tags);
    for (Unsigned index = 0; index < tags_size; index++) {
	Tag tag = (Tag)List__fetch(all_tags, index);
	Tag__free(tag);
    }
    List__free(all_tags);

    // Release all the *Tag_Height*'s:
    List /* <Tag_Height> */ tag_heights = map->tag_heights;
    Unsigned tag_heights_size = List__size(tag_heights);
    for (Unsigned index = 0; index < tag_heights_size; index++) {
	Tag_Height tag_height = (Tag_Height)List__fetch(tag_heights, index);
	Tag_Height__free(tag_height);
    }
    List__free(map->tag_heights);

    List__free(map->pending_arcs);
    Table__free(map->arcs_table);    
    Table__free(map->tags_table);
    Memory__free((Memory)map);
}

/// @brief Log image to disk if image logging is turned on.
/// @param map to use.
/// @param image to log.
///
/// *Map__image_log*() will log *image* to disk if image logging is turned on.

static int last_sequence_number = 0xffffffff;

void Map__image_log(Map map, CV_Image image, Unsigned sequence_number) {
    if (image != (CV_Image)0 && map->image_log &&
      sequence_number != last_sequence_number) {
	// Log the image here:
	String file_name = String__format("log%05d.tga", sequence_number);
	CV_Image__tga_write(image, file_name);
	last_sequence_number = sequence_number;
    }
}

/// @brief Restore the contents of *Map* from *in_file*.
/// @param map is the *Map* to restore into
/// @param in_file is the *File* to read from.
///
/// *Map__restore*() will read in an XML map file from *in_file* and
/// store it into *map*.

void Map__restore(Map map, File in_file) {
    // Read in Map XML tag '<Map Tags_Count="xx" Arcs_Count="xx">' :
    File__tag_match(in_file, "Map");
    Unsigned all_tags_size =
      (Unsigned)File__integer_attribute_read(in_file, "Tags_Count");
    Unsigned all_arcs_size =
      (Unsigned)File__integer_attribute_read(in_file, "Arcs_Count");
    File__string_match(in_file, ">\n");

    // Read in the *all_tags_size* *Tag* objects:
    for (Unsigned index = 0; index < all_tags_size; index++) {
	Tag tag = Tag__read(in_file, map);
    }

    // Read in the *all_arcs_size* *Arc* objects:
    for (Unsigned index = 0; index < all_arcs_size; index++) {
	Arc arc = Arc__read(in_file, map);
    }

    // Process the final Map XML tag "</MAP>":
    File__tag_match(in_file, "/Map");
    File__string_match(in_file, ">\n");

    // Do some final checks:
    assert (List__size(map->all_arcs) == all_arcs_size);
    assert (List__size(map->all_tags) == all_tags_size);
}

/// @brief Save *map* out to the file named *file_name*.
/// @param map to save out.
/// @param file_name to save *map* to.
///
/// *Map__save*() will save *map* to the *file_name* file in XML format.

void Map__save(Map map) {
    if (!map->is_saved) {
	File out_file = File__open(map->file_name, "w");
	assert (out_file != (File)0);
	Map__write(map, out_file);
	File__close(out_file);
	map->is_saved = (Logical)1;
    }
}

/// @brief Sort the contents of *map* to be in a consistent order.
/// @param map to reorder.
///
/// *Map__sort*() will reorder all of the tags and neihbgors in *map*
/// to be in a consitent order.

void Map__sort(Map map) {
    List__sort(map->all_tags, (List__Compare__Routine)Tag__compare);
    List__sort(map->all_arcs, (List__Compare__Routine)Arc__compare);
}

/// @brief Writes *map* out to a file called *svg_base_name*.svg.
/// @param map is the *Map* to write out.
/// @param svg_base_name is the base name of the .svg file to write out.
///
/// *Map__svg_write*() will write out *map* out *svg_base_name*.svg.

void Map__svg_write(Map map, const String svg_base_name, List locations) {
    // Figure out how many *Arc*'s and *Tag*'s we have:
    List all_arcs = map->all_arcs;
    List all_tags = map->all_tags;
    Unsigned all_tags_size = List__size(all_tags);
    Unsigned all_arcs_size = List__size(all_arcs);

    // Compute the *bounding_box*:
    Bounding_Box bounding_box = Bounding_Box__new();
    for (Unsigned index = 0; index < all_tags_size; index++) {
	Tag tag = (Tag)List__fetch(all_tags, index);
	Tag__bounding_box_update(tag, bounding_box);
    }

    // Open the Scalable Vector Graphics file:
    SVG svg = SVG__open(svg_base_name, 8.0, 10.5, 1.0, 1.0, "in");

    SVG__cartesian_scale(svg, 8.0, 10.5, bounding_box);

    // Draw the X/Y axes:
    String color = "cyan";
    SVG__line(svg,
      bounding_box->minimum_x, 0.0, bounding_box->maximum_x, 0.0, color);
    SVG__line(svg,
      0.0, bounding_box->minimum_y, 0.0, bounding_box->maximum_y, color);

    // Output each *tag in *all_tags*:
    for (Unsigned index = 0; index < all_tags_size; index++) {
	Tag tag = (Tag)List__fetch(all_tags, index);
	Tag__svg_write(tag, svg);
    }

    // Output each *tag in *all_tags*:
    for (Unsigned index = 0; index < all_arcs_size; index++) {
	Arc arc = (Arc)List__fetch(all_arcs, index);
	Arc__svg_write(arc, svg);
	// publish rviz marker here

    }

    Unsigned locations_size = List__size(locations);
    Double last_x = 0.0;
    Double last_y = 0.0;
    for (Unsigned index = 0; index < locations_size; index++) {
	Location location = (Location)List__fetch(locations, index);
        Double x = location->x;
	Double y = location->y;
	Double bearing = location->bearing;
	//File__format(stderr, "Location[%d]: id:%d x:%f y:%f bearing:%f\n",
	//  index, location->id, x, y, bearing * 180 / 3.1415926);

	// Draw a triangle that shows the bearing:
	Double k1 = 40.0;
	Double k2 = k1 / 2.0;
	Double angle = 3.14159 * 0.75;
	Double x0 = x + k1 * Double__cosine(bearing);
	Double y0 = y + k1 * Double__sine(bearing);
	Double x1 = x + k2 * Double__cosine(bearing + angle);
	Double y1 = y + k2 * Double__sine(bearing + angle);
	Double x2 = x + k2 * Double__cosine(bearing - angle);
	Double y2 = y + k2 * Double__sine(bearing - angle);
	SVG__line(svg, x0, y0, x1, y1, "black");
	SVG__line(svg, x1, y1, x2, y2, "black");
	SVG__line(svg, x2, y2, x0, y0, "black");

	// Draw a line that connects the centers of the triangles:
	if (index > 0) {
	    SVG__line(svg, last_x, last_y, x, y, "purple");
	}
	last_x = x;
	last_y = y;
    }

    // Close *svg*:
    SVG__close(svg);

    Bounding_Box__free(bounding_box);
}

/// @brief Causes an arc announce callback routine to be called.
/// @param map is the parent *Map* object.
/// @param tag is the *Tag* object that has just been changed.
/// @param visible is True if the tag is in the current field of view.
/// @param image is the current image being processed.
///
/// *Map__arc_announce*() will cause the arc announde call back routine
/// to be called for *arc*.

void Map__tag_announce(Map map,
  Tag tag, Logical visible, CV_Image image, Unsigned sequence_number) {
    map->tag_announce_routine(map->announce_object,
      tag->id, tag->x, tag->y, tag->z, tag->twist,
      tag->diagonal, tag->distance_per_pixel, visible, tag->hop_count);
    if (visible) {
	//Map__image_log(map, image, sequence_number);
    }
}

/// @brief Returns the distance per pixel for *id*.
/// @param map is the *Map* object that contains the distance per pixel table.
/// @param id is the *Tag* identifier to look up.
/// @returns the distance per pixel for *Tag* id.
///
/// *Map__distance_per_pixel*() will return the distance per pixel for
/// *Tag* *id*.  The distance can be in any consistent distance (e.g.
/// (millimeters, centimeters, meters, kilometers, inches, feet, miles,
/// light seconds, etc.)

Tag_Height Map__tag_height_lookup(Map map, Unsigned id) {
    Double distance_per_pixel = 0.0;
    List /* Tag_Height */ tag_heights = map->tag_heights;
    Tag_Height tag_height = (Tag_Height)0;
    assert (tag_heights != (List)0);
    Unsigned size = List__size(tag_heights);
    for (Unsigned index = 0; index < size; index++) {
	tag_height = (Tag_Height)List__fetch(tag_heights, index);
	if (tag_height->first_id <= id && id <= tag_height->last_id) {
	    distance_per_pixel = tag_height->distance_per_pixel;
	    break;
	}
	tag_height = (Tag_Height)0;
    }
    return tag_height;
}

/// @brief Reads the tag heights .xml file.
/// @param map to to store tag heights into.
/// @param tag_heights_file_name is the file to read from.
///
/// *Map__tag_heights_xml_read*() will read the *tag_heights_file_name* .xml
/// file and the the tag heights into *map*.

void Map__tag_heights_xml_read(Map map, String_Const tag_heights_file_name) {
    // Open *tag_height_file_name* for reading:
    File xml_in_file = File__open(tag_heights_file_name, "r");
    if (xml_in_file == (File)0) {
	File__format(stderr, "Could not open '%s'\n", tag_heights_file_name);
	assert(0);
    }

    // Read in Map XML tag '<Map_Tag_Heights Count="xx">' :
    File__tag_match(xml_in_file, "Map_Tag_Heights");
    Unsigned count =
      (Unsigned)File__integer_attribute_read(xml_in_file, "Count");
    File__string_match(xml_in_file, ">\n");

    // Read in the *count* *Tag_Height* objects into *tag_heights*:
    List tag_heights = map->tag_heights;
    assert (tag_heights != (List)0);
    for (Unsigned index = 0; index < count; index++) {
	Tag_Height tag_height = Tag_Height__xml_read(xml_in_file);
	List__append(tag_heights, (Memory)tag_height,
	  "Map__tag_heights_xml_read:List__append, tag_heights");
    }

    // Process the final Map XML tag "</Map_Tag_Heights>":
    File__tag_match(xml_in_file, "/Map_Tag_Heights");
    File__string_match(xml_in_file, ">\n");

    // Close out *xml_in_file*:
    File__close(xml_in_file);

    // Sort *tag_heights*:
    List__sort(tag_heights, (List__Compare__Routine)Tag_Height__compare);
}

/// @brief Return the *Tag* associated with *tag_id* from *map*.
/// @param map to use for lookup.
/// @param tap_id to lookup.
/// @returns *Tag* associated with *tag_id*.
///
/// *Map__tag_lookup*() will lookup and return the *Tag* associaed with
/// *tag_id* using *map.  If no previous instance of *tag_id* has been
/// encountered, a new *Tag* is created and add to the association in *map*.

Tag Map__tag_lookup(Map map, Unsigned tag_id) {
    Table tags_table /* <Unsigned, Tag> */ = map->tags_table;
    Memory memory_tag_id = Unsigned__to_memory(tag_id);
    Tag tag = (Tag)Table__lookup(tags_table, memory_tag_id);
    if (tag == (Tag)0) {
	tag = Tag__create(tag_id, map);
	Table__insert(tags_table, memory_tag_id, (Memory)tag);
	List__append(map->all_tags, tag,
	  "Map__tag_lookup:List__append:all_tags");
	map->changes_count += 1;
	map->is_changed = (Logical)1;
	map->is_saved = (Logical)0;
    }
    return tag;
}

/// @brief Writes *map* out to *out_file*.
/// @param map to write out.
/// @param out_file to write to.
///
/// *Map__write*() will write *map* to *out_file* in XML format.

void Map__write(Map map, File out_file) {
    // Figure out how many *Arc*'s and *Tag*'s we have:
    List all_arcs = map->all_arcs;
    List all_tags = map->all_tags;
    Unsigned all_tags_size = List__size(all_tags);
    Unsigned all_arcs_size = List__size(all_arcs);

    // Output <Map ...> tag:
    File__format(out_file, "<Map");
    File__format(out_file, " Tags_Count=\"%d\"", all_tags_size);
    File__format(out_file, " Arcs_Count=\"%d\"", all_arcs_size);
    File__format(out_file, ">\n");

    // Put the tags out in sorted order:
    Map__sort(map);

    // Output each *tag in *all_tags*:
    for (Unsigned index = 0; index < all_tags_size; index++) {
	Tag tag = (Tag)List__fetch(all_tags, index);
	Tag__write(tag, out_file);
    }

    // Output each *tag in *all_tags*:
    for (Unsigned index = 0; index < all_arcs_size; index++) {
	Arc arc = (Arc)List__fetch(all_arcs, index);
	Arc__write(arc, out_file);
    }

    // Output the closing </Map> tag:
    File__format(out_file, "</Map>\n");
}

/// @brief Updates the location of each *tag* in *map*.
/// @param map to update.
/// @param image is the current image.
/// @param sequence_number is the image sequence number.
///
/// *Map__update*() will update the location of all the *Tag*'s in *map*.

void Map__update(Map map, CV_Image image, Unsigned sequence_number) {
    if (map->is_changed) {
	// Increment *visit* to the next value to use for updating:
	Unsigned visit = map->visit + 1;
	map->visit = visit;

	// We want the tag with the lowest id number to be the origin.
	// Sort *tags* from lowest tag id to greatest:
	List /* <Tag> */ all_tags = map->all_tags;
	List__sort(all_tags, (List__Compare__Routine)Tag__compare);

	// The first tag in {tags} has the lowest id and is forced to be the
	// map origin:
	Tag origin_tag = (Tag)List__fetch(all_tags, 0);
	origin_tag->visit = visit;
	origin_tag->hop_count = 0;
	
	// The first step is to identify all of the *Arc*'s that make a
	// spanning tree of the *map* *Tags*'s.

	// Initializd *pending_arcs* with the *Arc*'s from *orgin_tag*:
	List /* <Arc> */ pending_arcs = map->pending_arcs;
	List__all_append(pending_arcs, origin_tag->arcs);

	// We always want to keep *pending_arcs* sorted from longest to
	// shortest at the end.  *Arc__distance_compare*() sorts longest first:
	List__sort(pending_arcs, (List__Compare__Routine)Arc__distance_compare);

	// We keep iterating across *pending_arcs* until it goes empty.
	// since we keep it sorted from longest to shortest (and we always
	// look at the end), we are building a spanning tree using the shortest
	// possible *Arc*'s:
	while (List__size(pending_arcs) != 0) {
	    // Pop the shortest *arc* off the end of *pending_arcs*:
	    Arc arc = (Arc)List__pop(pending_arcs);

	    // For debugging only:
	    //File__format(stderr, "----------\n");
	    //Unsigned size = List__size(pending_arcs);
	    //for (Unsigned index = 0; index < size; index++) {
	    //    Arc arc = (Arc)List__fetch(pending_arcs, index);
	    //    File__format(stderr,
	    //      "pending_arcs[%d]: Arc[%d,%d] dist=%f\n",
	    //      index, arc->origin->id, arc->target->id, arc->distance);
	    //}

	    // If we already visited *arc*, just ignore it:
	    if (arc->visit != visit) {
		// We have not visited this *arc* in this cycle, so now we
		// mark it as being *visit*'ed:
		arc->visit = visit;

		// Figure out if *origin* or *target* have been added to the
		// spanning tree yet:
		Tag from_tag = arc->from_tag;
		Tag to_tag = arc->to_tag;
		Logical from_is_new = (Logical)(from_tag->visit != visit);
		Logical to_is_new = (Logical)(to_tag->visit != visit);

		if (from_is_new || to_is_new) {
		    if (from_is_new) {
			// Add *to* to spanning tree:
			assert (!to_is_new);
			from_tag->hop_count = to_tag->hop_count + 1;
			List__all_append(pending_arcs, from_tag->arcs);
			from_tag->visit = visit;
			Tag__update_via_arc(from_tag,
			  arc, image, sequence_number);
		    } else {
			// Add *from* to spanning tree:
			assert (!from_is_new);
			to_tag->hop_count = from_tag->hop_count + 1;
			List__all_append(pending_arcs, to_tag->arcs);
			to_tag->visit = visit;
			Tag__update_via_arc(to_tag,
			  arc, image, sequence_number);
		    }

		    // Mark that *arc* is part of the spanning tree:
		    arc->in_tree = (Logical)1;

		    // Resort *pending_arcs* to that the shortest distance
		    // sorts to the end:
		    List__sort(pending_arcs,
		      (List__Compare__Routine)Arc__distance_compare);
		} else {
		    // *arc* connects across two nodes of spanning tree:
		    arc->in_tree = (Logical)0;
		}
	    }
	}

	// Mark that *map* is fully updated:
        map->is_changed = (Logical)0;
	map->is_saved = (Logical)0;
    }
}

