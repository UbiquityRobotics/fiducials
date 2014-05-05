// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "Character.hpp"

/// @brief Returns true if *character* is a decimal digit.
/// @param character to test.
/// @returns true if *character8 is a decimal digit.
///
/// *Character__is_decimal_digit*() will retrue (*bool*)1 if *character*
/// is a decimal digit and (*bool*)0 otherise.

bool Character__is_decimal_digit(Character character) {
    return (bool)isdigit(character);
}

