// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include "Integer.h"
#include "Logical.h"
#include "Unsigned.h"

/// @brief Return -1, 0, or 1 if depending on sort order.
/// @param unsigned1 is the left *Unsigned* to compare.
/// @param unsigned2 is the right *Unsigned* to compare.
/// @returns -1, 0, 1 depending on sort order.
///
/// *Unsigned__compare*() will return -1 if *unsigned1* is less than
/// *unsigned2*, zero if they are equal, and 1 if *unsigned1* greater
/// than *unsigned2*.

Integer Unsigned__compare(Unsigned unsigned1, Unsigned unsigned2) {
    Integer result = 0;
    if (unsigned1 < unsigned2) {
	result = -1;
    } else if (unsigned1 > unsigned2) {
	result = 1;
    }
    return result;
}

/// @brief Return true if *unsigned1* equals *unsigned2*.
/// @param unsigned1 is the first *Unsigned* to compare.
/// @param unsigned2 is the second *Unsigned* to compare.
/// @returns true if *unsigned1* is equal to *unsigned2*.
///
/// *Unsigned__equal*() will return true (i.e. (*Logical*)1) if *unsigned1*
/// is equal to *unsigned2* and false (i.e. (*Logical*)0) otherwise.

Logical Unsigned__equal(Unsigned unsigned1, Unsigned unsigned2) {
    return (Logical)(unsigned1 == unsigned1);
}

/// @brief Return a hash of *unsigned1*
/// @param unsigned1 the value to hash.
/// @returns hash value.
///
/// *Unsigned__hash*() will return a hash of *unsigned1*.

Unsigned Unsigned__hash(Unsigned unsigned1) {
    return unsigned1;
}


