// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include "Unsigned.hpp"

/// @brief Return -1, 0, or 1 if depending on sort order.
/// @param unsigned1 is the left *Unsigned* to compare.
/// @param unsigned2 is the right *Unsigned* to compare.
/// @returns -1, 0, 1 depending on sort order.
///
/// *Unsigned__compare*() will return -1 if *unsigned1* is less than
/// *unsigned2*, zero if they are equal, and 1 if *unsigned1* greater
/// than *unsigned2*.

int Unsigned__compare(Unsigned unsigned1, Unsigned unsigned2) {
    int result = 0;
    if (unsigned1 < unsigned2) {
	result = -1;
    } else if (unsigned1 > unsigned2) {
	result = 1;
    }
    return result;
}

