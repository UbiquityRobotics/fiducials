/// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(STRING_H_INCLUDED)
#define STRING_H_INCLUDED 1

#include "Unsigned.h"
#include "Logical.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief *String* is a null-terminated string.
typedef char *String;

// External declarations:

extern Logical String__equal(String string1, String string2);
extern String String__allocate(Unsigned size);
extern String String__format(String format, ...);
extern void String__free(String string);
extern Unsigned String__size(String string);
extern Unsigned String__to_unsigned(String string);

#ifdef __cplusplus
}
#endif
#endif // !defined(STRING_H_INCLUDED)

