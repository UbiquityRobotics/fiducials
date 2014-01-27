/// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#if !defined(STRING_H_INCLUDED)
#define STRING_H_INCLUDED 1

#include "Unsigned.h"
#include "Logical.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief *String* is a null-terminated string.
typedef char *String;
  typedef const char *String_Const;

// External declarations:

extern Logical String__equal(String_Const string1, String_Const string2);
extern String String__allocate(Unsigned size);
extern String String__format(String_Const format, ...);
extern void String__free(String_Const string);
extern Unsigned String__size(String_Const string);
extern Unsigned String__to_unsigned(String_Const string);

#ifdef __cplusplus
}
#endif
#endif // !defined(STRING_H_INCLUDED)

