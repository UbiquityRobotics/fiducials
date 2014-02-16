// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#if !defined(MAP_H_INCLUDED)
#define MAP_H_INCLUDED 1

#include "File.h"
#include "List.h"
#include "Location.h"
#include "Table.h"
#include "Unsigned.h"

/// @brief *Map* is the representation of a fiducial marker map.
typedef struct Map__Struct *Map;

#include "Arc.h"
#include "Tag.h"
#include "Camera_Tag.h"
#include "Fiducials.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief A *Map__Struct* represents the fiducial location map.
struct Map__Struct {
    /// @brief Routine to call to announce change to arc.
    Fiducials_Arc_Announce_Routine arc_announce_routine;

    /// @brief All of the *Arc*'s (i.e. measured intertag distances) in the map.
    List /* <Arc> */ all_arcs;

    /// @brief All of the tags (i.e. fiducials) in the map.
    List /* <Tag> */ all_tags;

    /// @brief Opaque object passed into announce routines.
    void *announce_object;

    /// @brief An lookup *Arc* table.
    Table /* <Arc, Arc> */ arcs_table;

    /// @brief Number of map changes:
    Unsigned changes_count;

    /// @brief Base name of map file name.
    String_Const file_base;

    /// @brief Directory/folder of map file.
    String_Const file_path;

    /// @brief True if images that change map need to be recorded.
    Logical image_log;

    /// @brief True if map has changed since last update.
    Logical is_changed;

    /// @brief True if changed map has been saved.
    Logical is_saved;

    /// @brief List of pending *Arc*'s for map tree extraction.
    List /* <Arc> */ pending_arcs;

    /// @brief Routine that is called each time a tag is changed.
    Fiducials_Tag_Announce_Routine tag_announce_routine;

    /// @brief List of all known tag heights:
    List /* <Tag_Height> */ tag_heights;

    /// @brief Table of all *tags* indexed by *Tag* *id*.
    Table /* <Unsigned, Tag>*/ tags_table;

    /// @brief a te
    Arc temporary_arc;

    /// @brief Increment *visit* each time a map update is propogated.
    Unsigned visit;
};

// *Map* routines:

extern void Map__arc_announce(
  Map map, Arc arc, CV_Image image, Unsigned sequence_number);
extern void Map__arc_append(Map map, Arc arc);
extern Arc Map__arc_lookup(Map map, Tag from, Tag to);
extern Unsigned Map__arc_update(Map map, Camera_Tag camera_from,
  Camera_Tag camera_to, CV_Image image, Unsigned sequence_number);
extern Integer Map__compare(Map map1, Map map2);
extern Map Map__create(String_Const map_path, String_Const map_base,
  void *announce_object, Fiducials_Arc_Announce_Routine arc_announce_routine,
  Fiducials_Tag_Announce_Routine tag_announce_routine,
  String_Const tag_heights_file_name, String from);
extern void Map__free(Map map);
extern Tag_Height Map__tag_height_lookup(Map map, Unsigned id);
extern void Map__image_log(Map map, CV_Image image, Unsigned sequence_number);
extern void Map__restore(Map map, File in_file);
extern void Map__save(Map map);
extern void Map__sort(Map map);
extern void Map__svg_write(
  Map map, const String svg_base_name, List /*<Location>*/ locations);
extern void Map__tag_heights_xml_read(
  Map map, String_Const tag_heights_file_name);
extern void Map__tag_announce(
  Map map, Tag tag, Logical visible, CV_Image image, Unsigned sequence_number);
extern Tag Map__tag_lookup(Map map, Unsigned tag_id);
extern void Map__update(Map map, CV_Image image, Unsigned sequence_number);
extern void Map__write(Map map, File out_file);

#ifdef __cplusplus
}
#endif
#endif // !defined(MAP_H_INCLUDED)
