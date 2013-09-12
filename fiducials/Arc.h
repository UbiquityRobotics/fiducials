// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(ARC_H_INCLUDED)
#define ARC_H_INCLUDED 1

typedef struct Arc__Struct *Arc;

#include "File.h"
#include "Float.h"
#include "Logical.h"
#include "Integer.h"
#include "Map.h"
#include "Tag.h"

/// @brief A *Arc_Struct* represents arc from an *origin* *Tag* to a
/// *target* *Tag*.
///
/// It is constrained so that the *origin* id is less than the *target* id.
struct Arc__Struct {
    /// @brief The distance between the origin and the target.
    Float distance;

    /// @brief Distance between camera center and line connecting both tags.
    Float goodness;

    /// @brief Set to true if this *Arc* is part of the map tree.
    Logical in_tree;

    /// @brief The origin *Tag* (has smaller id than *target*).
    Tag origin;

    /// @brief The target *Tag* (has larger id than *origin*).
    Tag target;

    /// @brief The angle in radians from the *origin* center parallel to the
    /// bottom edge to the line that connects the *origin* and *target* centers.
    Float target_angle;

    /// @brief The angle in radians from the origin bottom edge to the
    /// target bottom edge.
    Float target_twist;

    /// @brief The visit number for the arc.
    Unsigned visit;
};

// *Arc* routines:

extern Integer Arc__compare(Arc arc1, Arc arc2);
extern Integer Arc__distance_compare(Arc arc1, Arc arc2);
extern Logical Arc__equal(Arc arc1, Arc arc2);
extern Arc Arc__create(Tag origin, Tag target, Float distance,
  Float target_angle, Float target_twist, Float goodness);
extern Unsigned Arc__hash(Arc arc);
extern Arc Arc__read(File out_file, Map map);
extern void Arc__write(Arc arc, File out_file);

#endif // !defined(ARC_H_INCLUDED)

