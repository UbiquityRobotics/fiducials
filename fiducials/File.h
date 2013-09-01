// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FILE_H_INCLUDED)
#define FILE_H_INCLUDED 1

#include <stdio.h>
#include <stdarg.h>

#include "String.h"

/// @brief *FILE* is a file I/O object.
typedef FILE *File;

// External declarations:
extern void File__close(File file);
extern void File__format(File file, String format, ...);
extern File File__open(String file_name, String flags);

#endif // !defined(FILE_H_INCLUDED)
