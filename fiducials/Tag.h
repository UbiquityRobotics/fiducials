// Copyright (c) 20013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(TAG_H_INCLUDED)
#define TAG_H_INCLUDED 1

/// @brief *Tag* is a representation of a ceiling fiducial marker.
///
/// A *Tag* represents a 5-tuple:
///
///        (id, twist, x, y, arcs)
///
/// where:
///
/// * *id* is the tag identifier,
///
/// * *twist* is the amount the tag is twisted from the floor X axis,
///
/// * *x* is the absolute X floor coordinate of the center of *Tag*,
///
/// * *y* is the absolute Y floor coordinate of the center of *Tag*,
///
/// * *arcs* is a list of 0, 1, or more *Arc*'s that connect to other
///   *Tag*'s.
///
/// *twist* needs a little more discussion.  The bottom edge of the
/// fiducial establishes a coordinate system for the *Tag*.  The vector
/// from the lower left corner to the lower right corner is the Tag "X"
/// axis.  Here is some crude ASCII art:
///
///        UL-------UR
///        |         |
///        |    O----+------> "X axis"
///        |         |
///        LL-------LR
///
/// The four corners are labeled UL, UR, LL, and LR for Upper Left, Upper
/// Right, Lower Left, and Lower Right respectively.  O stands for Origin
/// and is located in the exact center of the fiducial.  The "X axis"
/// for the fiducial goes to the left starting from the origin (O) and
/// moving to the right.  The "X axis" is parallel the line the goes
/// through the points LL and LR.
///
/// Internally, *twist* is represented in radians.

typedef struct Tag__Struct *Tag;

#include "Arc.h"
#include "Double.h"
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

    /// @brief Fiducial tag diagnal distance in camera pixels.
    Double diagonal;

    /// @brief Distance per camera pixel.
    Double distance_per_pixel;

    ///@brief True if rest of *Tag* is initialized.
    Logical initialized;

    /// @brief Distance from origin in hops:
    Unsigned hop_count;

    /// @brief Tag identifier.
    Unsigned id;

    /// @brief Parent *Map* object.
    Map map;

    /// @brief The twist from the floor X axis to the tag bottom edge.
    Double twist;

    /// @brief Visit counter.
    Unsigned visit;

    /// @brief Absolute X floor coordinate.
    Double x;

    /// @brief Absolute Y floor coordinate.
    Double y;
};

// *Tag* routines;

extern void Tag__arc_append(Tag tag, Arc arc);
extern Tag Tag__create(Unsigned id, Map map);
extern Integer Tag__compare(Tag tag1, Tag tag2);
extern Logical Tag__equal(Tag tag1, Tag tag2);
extern Unsigned Tag__hash(Tag tag);
extern void Tag__initialize(
  Tag tag, Double angle, Double x, Double y, Unsigned visit);
extern void Tag__sort(Tag tag);
extern Tag Tag__read(File in_file, Map map);
extern void Tag__write(Tag tag, File out_file);
extern void Tag__update_via_arc(Tag tag, Arc arc);

#endif // !defined(TAG_H_INCLUDED)
