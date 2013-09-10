// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

/// @brief Global map of ceiling fiducial markers.
///
/// A *Map* consists of a list of *Tag* objects, where each Tag represents
/// the position and orientation of ceiling fiduical markers.  Each *Tag*
/// has zero one or more *Neighbor* objects that specify the distance and
/// orientation of *Tag* pairs.

typedef struct Map__Struct *Map_Doxygen_Fake_Out;

#include <assert.h>

#include "Integer.h"
#include "File.h"
#include "List.h"
#include "Map.h"
#include "Tag.h"
#include "Unsigned.h"

// *Map* routines:

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
    List tags1 = map1->tags;
    List tags2 = map2->tags;
    Unsigned tags1_size = List__size(tags1);
    Unsigned tags2_size = List__size(tags2);
    result = Unsigned__compare(tags1_size, tags2_size);
    if (result == 0) {
	for (Unsigned index = 0; index < tags1_size; index++) {
	    Tag tag1 = (Tag)List__fetch(tags1, index);
	    Tag tag2 = (Tag)List__fetch(tags2, index);
	    result = Tag__compare(tag1, tag2);
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
    map->tags = List__new();
    map->tags_table = Table__create((Table_Equal_Routine)Unsigned__equal,
      (Table_Hash_Routine)Unsigned__hash, (Memory)0);
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
    Table tags_table = map->tags_table;
    Tag tag = (Tag)Table__lookup(tags_table, (Memory)tag_id);
    if (tag == (Tag)0) {
	tag = Tag__create(tag_id);
	Table__insert(tags_table, (Memory)tag_id, (Memory)tag);
	List__append(map->tags, tag);
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
    List tags = map->tags;

    // Read in Map XML tag '<Map Tags_Count="xx">' :
    File__tag_match(in_file, "Map");
    Unsigned size =
      (Unsigned)File__integer_attribute_read(in_file, "Tags_Count");
    File__string_match(in_file, ">\n");

    // Read in the *size* *Tag* objects:
    for (Unsigned index = 0; index < size; index++) {
	Tag tag = Tag__read(in_file, map);
    }

    // Process the final Map XML tag "</MAP>":
    File__tag_match(in_file, "/Map");
    File__string_match(in_file, ">\n");
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
    List tags = map->tags;
    List__sort(tags, (List__Compare__Routine)Tag__compare);
    Unsigned size = List__size(tags);
    for (Unsigned index = 0; index < size; index++) {
	Tag tag = (Tag)List__fetch(tags, index);
	Tag__sort(tag);
    }
}

/// @brief Writes *map* out to *out_file*.
/// @param map to write out.
/// @param out_file to write to.
///
/// *Map__write*() will write *map* to *out_file* in XML format.

void Map__write(Map map, File out_file) {
    // Output <Map ...> tag:
    List tags = map->tags;
    Unsigned tags_size = List__size(tags);
    File__format(out_file, "<Map Tags_Count=\"%d\">\n", tags_size);

    // Put the tags out in sorted order:
    Map__sort(map);

    // Output each *tag in *tags*:
    for (Unsigned index = 0; index < tags_size; index++) {
	Tag tag = (Tag)List__fetch(tags, index);
	Tag__write(tag, out_file);
    }

    // Output the closing </Map> tag:
    File__format(out_file, "</Map>\n");
}

