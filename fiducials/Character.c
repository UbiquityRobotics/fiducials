// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "Logical.h"
#include "Character.h"

/// @brief Returns true if *character* is a decimal digit.
/// @param character to test.
/// @returns true if *character8 is a decimal digit.
///
/// *Character__is_decimal_digit*() will retrue (*Logical*)1 if *character*
/// is a decimal digit and (*Logical*)0 otherise.

Logical Character__is_decimal_digit(Character character) {
    return (Logical)isdigit(character);
}

