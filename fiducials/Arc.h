// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(ARC_H_INCLUDED)
#define ARC_H_INCLUDED 1

/// @brief *Arc* is just a pointer to *Arc__Struct*.

typedef struct Arc__Struct *Arc;

#include "File.h"
#include "Double.h"
#include "Logical.h"
#include "Integer.h"
#include "Map.h"
#include "Tag.h"

/// @brief An *Arc_Struct* represents arc from the *from* *Tag* to the
/// *to* *Tag*.
///
/// Ultimately an *Arc* specifies an ordered 5-tuple:
///
///        (from, to, distance, angle, twist)
///
/// where
///
/// * *from* is the from *Tag*,
///
/// * *to* is the to *Tag*,
///
/// * *distance* is the distance between the center of *from* to the
///    center of *to*.
///
/// * *angle* measures the angle from the *from* "X axis" to the distance line.
///
/// * *twist* is the change in twist between the *from* coordinate system
///   to the *to* coordinate system.
///
/// A conjugate *Arc* is one where the *from* and *to* tags have been swapped.
/// For an *Arc*:
///
///        (from, to, distance, angle, twist)
///
/// the conjugate *Arc* is:
///
///        (to, from, distance, 180 + angle - twist, -twist)
///
/// I wish I could draw some crude ASCII art to show this geometry,
/// but it would be illegible.  Please have fun drawing the geometry
/// to verify the formulas.
///
/// For fun, we can conjegate the same arc twice:
///
/// The original conjugate:
///
///        angle' = 180 + angle - twist
///        twist' = -twist
///
/// The double conjugate algebra follows:
///
///        angle'' = 180 + angle' - twist'
///        twist'' = -twist'
///
///        angle'' = 180 + (180 + angle - twist) - (-twist)
///        twist'' = -(-twist)
///
///        angle'' = 180 + 180 + angle - twist + twist
///        twist'' = twist
///
///        angle'' = 360 + angle
///        twist'' = twist
///
///        angle'' = angle
///        twist'' = twist
///
/// In order to avoid have a bunch *Arc* and conjugate pairs, we arbitrarily
/// constrain each *Arc* object such that the *from* identifier is less than
/// the *to* identifier.  The *Map* and *Arc* code can trivially compute the
/// conjugate if needed.
///
/// Note: The more I think about this data structure it should be changed to:
///
///        (from, to, distance, from_twist, to_twist)
///
/// where the twist angles are measured using the line between *from* and *to*
/// as the X axis starting from fiducial center in question.  Thus, the
/// for *from* starts at *from* and points directly at *to*.  Likewise,
/// the "X axis" for *to* starts at the center of *to* and points directly
/// at the centero of *from*.  Using these definitions, the complex conjegate
/// would be:
///
///        (to, from, distance, to_twist, from_twist)
///
/// This is pretty simple to understand.

struct Arc__Struct {
    /// @brief The angle in radians from the *origin* center parallel to the
    /// bottom edge to the line that connects the *origin* and *target* centers.
    Double angle;

    /// @brief The distance between the *from* and *to*.
    Double distance;

    /// @brief The from *Tag* (has smaller id than *to*).
    Tag from;

    /// @brief Distance between camera center and line connecting both tags.
    Double goodness;

    /// @brief Set to true if this *Arc* is part of the map tree.
    Logical in_tree;

    /// @brief The to *Tag* (has larger id than *from*).
    Tag to;

    /// @brief The releative change in tag twist angle from *from* coordinates
    /// to *to* coordinates.
    Double twist;

    /// @brief The visit number for the arc.
    Unsigned visit;
};

// *Arc* routines:

extern Integer Arc__compare(Arc arc1, Arc arc2);
extern Integer Arc__distance_compare(Arc arc1, Arc arc2);
extern Logical Arc__equal(Arc arc1, Arc arc2);
extern Arc Arc__create(Tag from, Tag to,
  Double distance, Double angle, Double twist, Double goodness);
extern Arc Arc__new(void);
extern Unsigned Arc__hash(Arc arc);
extern Arc Arc__read(File out_file, Map map);
extern void Arc__update(
  Arc arc, Double distance, Double angle, Double twist, Double goodness);
extern void Arc__write(Arc arc, File out_file);

#endif // !defined(ARC_H_INCLUDED)

