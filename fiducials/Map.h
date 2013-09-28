// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(MAP_H_INCLUDED)
#define MAP_H_INCLUDED 1

/// @brief *Map* is the representation of a fiducial marker map.
typedef struct Map__Struct *Map;

#include "Arc.h"
#include "Camera_Tag.h"
#include "File.h"
#include "List.h"
#include "Map.h"
#include "Table.h"
#include "Tag.h"
#include "Unsigned.h"

/// @brief A *Map__Struct* represents the fiducial location map.
struct Map__Struct {
    /// @brief All of the *Arc*'s (i.e. measured intertag distances) in the map.
    List /* <Arc> */ all_arcs;

    /// @brief All of the tags (i.e. fiducials) in the map.
    List /* <Tag> */ all_tags;

    /// @brief An lookup *Arc* table.
    Table /* <Arc, Arc> */ arcs_table;

    /// @brief True if map has changed since last update.
    Logical is_changed;

    /// @brief List of pending *Arc*'s for map tree extraction.
    List /* <Arc> */ pending_arcs;

    /// @brief Table of all *tags* indexed by *Tag* *id*.
    Table /* <Unsigned, Tag>*/ tags_table;

    /// @brief a te
    Arc temporary_arc;

    /// @brief Increment *visit* each time a map update is propogated.
    Unsigned visit;
};

// *Map* routines:

extern void Map__arc_append(Map map, Arc arc);
extern Arc Map__arc_lookup(Map map, Tag from, Tag to);
extern Unsigned Map__arc_update(
  Map map, Camera_Tag camera_from, Camera_Tag camera_to, CV_Image image);
extern Integer Map__compare(Map map1, Map map2);
extern Map Map__new(void);
extern Map Map__read(File in_file);
extern Map Map__restore(String file_name);
extern void Map__save(Map map, String file_name);
extern void Map__sort(Map map);
extern void Map__svg_write(Map map, String svg_base_name);
extern Tag Map__tag_lookup(Map map, Unsigned tag_id);
extern void Map__update(Map map);
extern void Map__write(Map map, File out_file);

#endif // !defined(MAP_H_INCLUDED)
