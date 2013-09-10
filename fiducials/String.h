/// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(STRING_H_INCLUDED)
#define STRING_H_INCLUDED 1

/// @brief *String* is a null-terminated string.
typedef char *String;

#include "Unsigned.h"

// External declarations:

extern String String__allocate(Unsigned size);
extern String String__format(String format, ...);
extern void String__free(String string);
extern Unsigned String__size(String string);
extern Unsigned String__to_unsigned(String string);

#endif // !defined(STRING_H_INCLUDED)

