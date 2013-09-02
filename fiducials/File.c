// Copyright (c) 2013 by Wayne C. Gramlich all rights reserved.

#include <assert.h>

#include "File.h"
#include "Integer.h"
#include "String.h"
#include "Unsigned.h"

// *File* routines:

/// @brief Read a byte from *file*.
/// @param file to read from.
/// @returns byte read from *file*.
///
/// *File__byte_read*() will read a byte from *file* and return it.

Unsigned File__byte_read(File file) {
    Integer byte = fgetc(file);
    assert (byte >= 0);
    return (Unsigned)byte;
}

/// @brief Write *byte* ot *file*.
/// @param file to read from.
/// @param byte to write out.
///
/// *File__byte_write*() will write *byte* to *file*.

void File__byte_write(File file, Unsigned byte) {
    fputc(byte, file);
}

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

/// @brief Read a little endian short (16-bits) from *file*.
/// @param file to read from.
/// @returns 16-bit value from *file*.
///
/// *File__little_endian_short_read*() will read a 16-bit unsigned integer
/// from *file* and return it.

Unsigned File__little_endian_short_read(File file) {
    Integer low_byte = fgetc(file);
    assert (low_byte >= 0);
    Integer high_byte = fgetc(file);
    assert (high_byte >= 0);
    Unsigned result = ((Unsigned)high_byte << 8) | (Unsigned)low_byte;
    return result;
}

/// @brief Write 16-bit *xshort* to *file* in little endian format.
/// @param file to write.
/// @param xshort to write.
///
/// *File__little_endian_short_write*() will write write *xshort* to *file*
/// as a little endian 16-bit unsigned integer.

void File__little_endian_short_write(File file, Unsigned xshort) {
    fputc(xshort & 0xff, file);
    fputc((xshort >> 8) & 0xff, file);
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

