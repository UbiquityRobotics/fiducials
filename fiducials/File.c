// Copyright (c) 2013 by Wayne C. Gramlich all rights reserved.

#include <assert.h>

#include "File.h"
#include "String.h"

// *File* routines:

/// @brief Closes *file*.
/// @param file to close.
///
/// *File__close*() will close *file*.

void File__close(File file) {
    assert (fclose(file) == 0);
}

/// @brief will write *format* out to *file* with all patterns that
/// start with "%" replaced by formatted versions of its arguments.
/// @param file to output to.
/// @parma format is the formatting string.
///
/// *File__format*() will write *format* out to *file* with all patterns that
/// start with "%" replaced by formatted versions of its arguments.

void File__format(File file, String format, ...) {
    // Set up *variadic_arguments to start after *format*:
    va_list variadic_arguments;
    va_start(variadic_arguments, format);

    // Perform the format:
    Unsigned formatted_size = vfprintf(file, format, variadic_arguments);
}

/// @brief will open *file_name* using *flags* to specify read/write options.
/// @param file_name is the file name to open.
/// @param flags specify the read/write options.
///
/// *File__open*() will open *file_name* using *flags* to read/write options.
/// An open *File* object is returned or (*File)0 if the open failed.

File File__open(String file_name, String flags) {
    return fopen(file_name, flags);
}

