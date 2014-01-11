// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FILE_H_INCLUDED)
#define FILE_H_INCLUDED 1

#include <stdio.h>
#include <stdarg.h>

#include "Character.h"
#include "Float.h"
#include "Integer.h"
#include "String.h"
#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief *FILE* is a file I/O object.
typedef FILE *File;

// External declarations:
extern Unsigned File__byte_read(File file);
extern void File__byte_write(File file, Unsigned byte);
extern Character File__character_read(File in_file);
extern void File__close(File file);
extern Float File__float_attribute_read(File in_file, String attribute_name);
extern void File__format(File file, const char * format, ...);
extern Integer File__integer_attribute_read(
  File in_file, String attribute_name);
extern Unsigned File__little_endian_short_read(File);
extern void File__little_endian_short_write(File, Unsigned xshort);
extern File File__open(String file_name, String flags);
extern void File__string_match(File in_file, String pattern);
extern void File__tag_match(File in_file, String tag_name);


#ifdef __cplusplus
}
#endif
#endif // !defined(FILE_H_INCLUDED)
