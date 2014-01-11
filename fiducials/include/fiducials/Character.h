// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CHARACTER_H_INCLUDED)
#define CHARACTER_H_INCLUDED

#include <ctype.h>
#include "Logical.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief *Character* is a 32-bit signed character.
typedef int Character;

extern Logical Character__is_decimal_digit(Character character);

#ifdef __cplusplus
}
#endif
#endif // !defined(CHARACTER_H_INCLUDED)

