// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

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
#include "Map.h"
#include "Tag.h"
#include "Table.h"
#include "Unsigned.h"

// *Map* routines:

/// @brief Appends *arc* to *map*.
/// @param map to append to.
/// @param arc to append
///
/// *Map__arc_append*() will append *arc* to *map*.

void Map__arc_append(Map map, Arc arc) {
    List__append(map->all_arcs, arc);
    map->is_changed = (Logical)1;
}

Arc Map__arc_lookup(Map map, Tag from, Tag to) {
    if (from->id > to->id) {
	Tag temporary = from;
	from = to;
	to = temporary;
    }

    Arc temporary_arc = map->temporary_arc;
    temporary_arc->from = from;
    temporary_arc->to = to;
    Table /* <Arc, Arc> */ arcs_table = map->arcs_table;
    Arc arc = (Arc)Table__lookup(arcs_table, (Memory)temporary_arc);
    if (arc == (Arc)0) {
 	arc = Arc__new();
	arc->from = from;
	arc->to = to;
	Table__insert(arcs_table, (Memory)arc, (Memory)arc);
	Map__arc_append(map, arc);
	Tag__arc_append(from, arc);
	Tag__arc_append(to, arc);
    }
    return arc;
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
/// @returns a new *Map*.
///
/// *Map__new*() creates and returns an empty initialized *Map* object.

Map Map__new(void) {
    Map map = Memory__new(Map);
    map->all_arcs = List__new(); // <Tag>
    map->all_tags = List__new(); // <Tag>
    map->arcs_table = Table__create((Table_Equal_Routine)Arc__equal,
      (Table_Hash_Routine)Arc__hash, (Memory)0); // <Arc, Arc>
    map->is_changed = (Logical)0;
    map->pending_arcs = List__new(); // <Tag>
    map->tags_table = Table__create((Table_Equal_Routine)Unsigned__equal,
      (Table_Hash_Routine)Unsigned__hash, (Memory)0); // <Unsigned, Tag>
    map->temporary_arc = Arc__new();
    map->visit = 0;
    return map;
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
	List__append(map->all_tags, tag);
	map->is_changed = (Logical)1;
    }
    return tag;
}

/// @brief Read a *Map* from *in_file* and returns it.
/// @param in_file is the file to read from.
/// @returns the *Map* object.
///
/// *Map__read*() will read in an XML map file from *in_file* and return
/// the resulting *Map* object.

Map Map__read(File in_file) {
    // Create *map* and get *tags* list:
    Map map = Map__new();

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

    return map;
}

/// @brief Read in a *map* XML file and return it.
/// @param file_name is the file to read in.
/// @returns the resulting *Map* object.
///
/// *Map__restore*() will read in an map XML from *file_name* and return the
/// resulting *Map* object.

Map Map__restore(String file_name) {
    File in_file = File__open(file_name, "r");
    assert(in_file != (File)0);
    Map map = Map__read(in_file);
    File__close(in_file);
    return map;
}

/// @brief Save *map* out to the file named *file_name*.
/// @param map to save out.
/// @param file_name to save *map* to.
///
/// *Map__save*() will save *map* to the *file_name* file in XML format.

void Map__save(Map map, String file_name) {
    File out_file = File__open(file_name, "w");
    assert (out_file != (File)0);
    Map__write(map, out_file);
    File__close(out_file);
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

// This routine will update the map coordinates for each *tag*
// in *map*.  The tag with the lowest id number is used as the
// origin and bearing of zero.

void Map__update(Map map) {
    if (map->is_changed != 0) {
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
		Tag from = arc->from;
		Tag to = arc->to;
		Logical from_is_new = (Logical)(from->visit != visit);
		Logical to_is_new = (Logical)(to->visit != visit);

		if (from_is_new || to_is_new) {
		    if (from_is_new) {
			// Add *to* to spanning tree:
			assert (!to_is_new);
			from->hop_count = to->hop_count + 1;
			List__all_append(pending_arcs, from->arcs);
			from->visit = visit;
			Tag__update_via_arc(from, arc);
		    } else {
			// Add *from* to spanning tree:
			assert (!from_is_new);
			to->hop_count = from->hop_count + 1;
			List__all_append(pending_arcs, to->arcs);
			to->visit = visit;
			Tag__update_via_arc(to, arc);
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

//	// Now we can force a map update starting from {origin_tag}:
//	Tag__initialize(origin_tag, 0.0, 0.0, 0.0, visit_counter);
//
//	// Make sure all new tags get the update routine called on them:
//	List changed_tags = map->changed_tags;
//	List__sort(changed_tags, (List__Compare__Routine)Tag__compare);
//	List__unique(changed_tags, (List__Equal__Routine)Tag__equal);
//	Unsigned size = List__size(changed_tags);
//	for (Unsigned index = 0; index < size; index++) {
//	    Tag tag = (Tag)List__fetch(changed_tags, index);
//	    //Tag__updated(tag);
//	}
//	List__trim(changed_tags, 0);
//
//	// Make sure all new neighbors get the update routine called on them:
//	List changed_neighbors = map->changed_neighbors;
//	List__sort(changed_neighbors,
//	  (List__Compare__Routine)Neighbor__compare);
//	List__unique(changed_neighbors, (List__Equal__Routine)Neighbor__equal);
//	size = List__size(changed_neighbors);
//	for (Unsigned index = 0; index < size; index++) {
//	    Neighbor neighbor = (Neighbor)List__fetch(changed_neighbors, index);
//	    //Neighbor__updated(neighbor);
//	}
//	List__trim(changed_neighbors, 0);

	// Mark that *map* is fully updated:
        map->is_changed = (Logical)0;
    }
}

/// @brief Makes sure the *Arc* connecting *from* to *to* is up to date.
/// @brief map to use for *Arc* updating.
/// @brief from is the *Camera_Tag* to for one end of the *Arc*.
/// @brief to is the *Camera_Tag* to the other end of the *Arc*.
/// @brief image is the image that the *Camera_Tag*'s came from.
/// @returns the number of *Arc*'s updated (1 or 0).
///
/// *Map__arc_update*() will create or update the *Arc* in *map* associated
/// with *from* and *to*.  *image* used to determine the frame size.

Unsigned Map__arc_update(
  Map map, Camera_Tag camera_from, Camera_Tag camera_to, CV_Image image) {
    // Get the *width* and *height*:
    Integer rows = CV_Image__height_get(image);
    Integer columns = CV_Image__width_get(image);
    Double height = (Double)rows;
    Double width = (Double)columns;

    // Compute some constants:
    Double half_width = width / 2.0;
    Double half_height = height / 2.0;
    Double pi = 3.14159265358979323846264;

    // Extract some field values from *from* and *to*:
    Double from_center_x = camera_from->x;
    Double from_center_y = camera_from->y;
    Tag from_tag = camera_from->tag;
    Double to_center_x = camera_to->x;
    Double to_center_y = camera_to->y;
    Tag to_tag = camera_to->tag;

    // Compute the X and Y distance between the center of the image and the
    // center of *from* and *to*:
    Double fdx = half_width - from_center_x;
    Double fdy = half_height - from_center_y;
    Double tdx = half_width - to_center_x;
    Double tdy = half_height - to_center_y;

    // Compute diagonal pixel distance between image center and tag centers:
    Double from_distance = Double__square_root(fdx * fdx + fdy * fdy);
    Double to_distance = Double__square_root(tdx * tdx + tdy * tdy);

    // To minimize camera distortion, we want to use images where *from*
    // and *to* are about equidistant from the image center.  This
    // we want to minimum the absolute value of the distance difference:
    Double goodness = Double__absolute(from_distance - to_distance);

    // Find associated *Arc*:
    Arc arc = Map__arc_lookup(map, from_tag, to_tag);

    // Now see if the new {goodness_metric} is better than the previous one:
    Unsigned changed = 0;
    if (goodness < arc->goodness) {
	// We have a better *goodness* metric, record new values into *arc*:

	// Grab some additional values from *from* and *to*:

	// {neighbor_twist} is normalized angle change in fiducial orientation
	// from {from} to {to}.
	Double from_twist = from_tag->twist;
	Double to_twist = to_tag->twist;
	Double arc_twist = Double__angle_normalize(to_twist - from_twist);

	// Compute *inches_per_pixel*:
	//Double inches_across_frame = constants.inches_across_frame;
	Double distance_per_pixel = from_tag->distance_per_pixel;

	// Compute the distance between *origin* and *to*.
	Double dx = to_center_x - from_center_x;
	Double dy = to_center_y - from_center_y;
	Double distance_pixels = Double__square_root(dx * dx + dy * dy);
	Double distance = distance_pixels * distance_per_pixel;

	// {neighbor_angle} is the angle from bottom edge of {from}
	// to the center of {to}:
 	Double spin = Double__arc_tangent2(dy, dx);
	Double angle = Double__angle_normalize(spin - from_twist);

	Arc__update(arc, distance, angle, spin, goodness);

	changed = 1;
    }
    return changed;
}


