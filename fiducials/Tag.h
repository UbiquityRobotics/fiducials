// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(TAG_H_INCLUDED)
#define TAG_H_INCLUDED 1

/// @brief *Tag* is a representation of a ceiling fiducial marker.
typedef struct Tag__Struct *Tag;

#include "Arc.h"
#include "Float.h"
#include "File.h"
#include "Integer.h"
#include "List.h"
#include "Map.h"
#include "Unsigned.h"

/// @brief A *Tag_Struct* represents the location and orientation of one 
/// ceiling fiducial tag.
struct Tag__Struct {
    /// @brief List *Arc*'s connected to this *Tag*.
    List /* <Arc>*/ arcs;

    ///@brief True if rest of *Tag* is initialized.
    Logical initialized;

    /// @brief The angle from the floor X axis to the tag bottom edge.
    Float floor_angle;

    /// @brief Absolute X floor coordinate.
    Float floor_x;

    /// @brief Absolute Y floor coordinate.
    Float floor_y;

    /// @brief Distance from origin in hops:
    Unsigned hop_count;

    /// @brief Tag identifier.
    Unsigned id;

    /// @brief Parent *Map* object.
    Map map;

    /// @brief Visit counter.
    Unsigned visit;
};

// *Tag* routines;

extern void Tag__arc_append(Tag tag, Arc arc);
extern Tag Tag__create(Unsigned id, Map map);
extern Integer Tag__compare(Tag tag1, Tag tag2);
extern Logical Tag__equal(Tag tag1, Tag tag2);
extern Unsigned Tag__hash(Tag tag);
extern void Tag__initialize(
  Tag tag, Float floor_angle, Float floor_x, Float floor_y, Unsigned visit);
extern void Tag__sort(Tag tag);
extern Tag Tag__read(File in_file, Map map);
extern void Tag__write(Tag tag, File out_file);

#endif // !defined(TAG_H_INCLUDED)
