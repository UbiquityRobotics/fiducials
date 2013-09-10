// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(NEIGHBOR_H_INCLUDED)
#define NEIGHBOR_H_INCLUDED 1

typedef struct Neighbor__Struct *Neighbor;

#include "File.h"
#include "Float.h"
#include "Integer.h"
#include "Map.h"
#include "Tag.h"

/// @brief A *Neighbor_Struct* represents arc from an *origin* *Tag* to a
/// *target* *Tag*.
struct Neighbor__Struct {
    /// @brief The distance between the origin and the target.
    Float distance;

    /// @brief Distance between camera center and line connecting both tags.
    Float goodness;

    /// @brief The origin *Tag*.
    Tag origin;

    /// @brief The target *Tag*.
    Tag target;

    /// @brief The angle in radians from the origin bottom edge to the
    /// target center.
    Float target_angle;

    /// @brief The angle in radians from the origin bottom edge to the
    /// target bottom edge.
    Float target_twist;
};

// *Neighbor* routines:

extern Integer Neighbor__compare(Neighbor neighbor1, Neighbor neighbor2);
extern Neighbor Neighbor__create(Tag origin, Tag target, Float distance,
  Float target_angle, Float target_twist, Float goodness);
extern Neighbor Neighbor__read(File out_file, Map map);
extern void Neighbor__write(Neighbor neighbor, File out_file);

#endif // !defined(NEIGHBOR_H_INCLUDED)

