// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(UNSIGNED_H_INCLUDE)
#define UNSIGNED_H_INCLUDE 1

/// @brief *Unsigned* is a 32-bit unsigned integer.
typedef unsigned int Unsigned;

#include "Integer.h"
#include "Logical.h"

// *Unsigned* routines:

extern Integer Unsigned__compare(Unsigned unsigned1, Unsigned unsigned2);
extern Unsigned Unsigned__minimum(Unsigned unsigned1, Unsigned unsigned2);
extern Logical Unsigned__equal(Unsigned unsigned1, Unsigned unsigned2);
extern Unsigned Unsigned__hash(Unsigned unsigned1);

#endif // !defined(UNSIGNED_H_INCLUDE)
