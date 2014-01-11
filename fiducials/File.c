// Copyright (c) 2013 by Wayne C. Gramlich all rights reserved.

#include <assert.h>

#include "Character.h"
#include "File.h"
#include "Float.h"
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

/// @brief Return the next character read from *in_file*.
/// @param in_file to read from.
/// @returns character read from *in_file*.
///
/// *File__character_read*() will read in and return the next character
/// from *in_file*.  (*Character*)(-1) is returned when an end of file
/// condition is encountered on *in_file*.

Character File__character_read(File in_file) {
    return (Character)fgetc(in_file);
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

void File__format(File file, const char * format, ...) {
    // Set up *variadic_arguments to start after *format*:
    va_list variadic_arguments;
    va_start(variadic_arguments, format);

    // Perform the format:
    Unsigned formatted_size = vfprintf(file, format, variadic_arguments);
}

/// @brief Reads in an XML attribute with a floating point value.
/// @param in_file is the input file to read from.
/// @param attribute name is the attribute name.
/// @returns the floating point value.
///
/// *File__float_attribute_read*() will read in a pattern that matches
/// ' ATTRIBUTE_NAME="VALUE"', where ATTRIBUTE_NAME matches *attribute_name*
/// and VALUE is an optionally signed floating point number.  This is used
/// for parsing "XML" file input.  "XML" is in quotes is because this is
/// really not a very robust XML parser.  An assertion failure occurs if
/// the input does not parse properly.

Float File__float_attribute_read(File in_file, String attribute_name) {
    File__string_match(in_file, " ");
    File__string_match(in_file, attribute_name);
    File__string_match(in_file, "=\"");
    Float fraction = (Float)1.0;
    Logical have_decimal_point = (Logical)0;
    Logical negative = (Logical)0;
    Float result = (Float)0.0;
    while (1) {
        Character character = File__character_read(in_file);
	if (Character__is_decimal_digit(character)) {
	    if (have_decimal_point) {
		fraction /= (Float)10.0;
		result += fraction * (Float)(character - '0');
	    } else {
		result = result * 10.0 + (Float)(character - '0');
	    }
	} else if (character == '"') {
	    break;
	} else if (character == '.') {
	    have_decimal_point;
	} else if (character == '-') {
	    negative = (Logical)1;
	} else {
	    assert(0);
	}
    }
    if (negative) {
	result = -result;
    }
    return result;
}

/// @brief Reads in an XML attribute with a integer value.
/// @param in_file is the input file to read from.
/// @param attribute name is the attribute name.
/// @returns the floating point value.
///
/// *File__integer_attribute_read*() will read in a pattern that matches
/// ' ATTRIBUTE_NAME="VALUE"', where ATTRIBUTE_NAME matches *attribute_name*
/// and VALUE is an optionally signed integer number.  This is used
/// for parsing "XML" file input.  "XML" is in quotes is because this is
/// really not a very robust XML parser.  An assertion failure occurs if
/// the input does not parse properly.

Integer File__integer_attribute_read(File in_file, String attribute_name) {
    File__string_match(in_file, " ");
    File__string_match(in_file, attribute_name);
    File__string_match(in_file, "=\"");
    Logical negative = (Logical)0;
    Integer result = 0;
    while (1) {
        Character character = File__character_read(in_file);
	if (Character__is_decimal_digit(character)) {
	    result = result * 10 + (character - '0');
	} else if (character == '"') {
	    break;
	} else if (character == '-') {
	    negative = (Logical)1;
	} else {
	    assert(0);
	}
    }
    if (negative) {
	result = -result;
    }
    return result;
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

/// @brief Exactly matches *pattern* read from *in_file*.
/// @param in_file to read from.
/// @param pattern to matach.
///
/// *File__string_match*() will read characters from *in_file* that must
/// exactly match *pattern*.  An assertion failure occurs if *pattern*
/// does not match exactly.

void File__string_match(File in_file, String pattern) {
    Unsigned size = String__size(pattern);
    for (Unsigned index = 0; index < size; index++) {
        Character character = File__character_read(in_file);
	assert(character == pattern[index]);
    }
}

/// @brief Matchs and "XML" start tag.
/// @param in_file is the file to read from.
/// @param tag_name is the name of the tat to match.
///
/// *File__tag_match*() parse "WHITESPACE<TAG" where WHTITESPACE  is zero
/// or more spaces and TAG matches *tag_name*.  An assertion failure occurs
/// the pattern does not parse properly.  This is not a very robust XML
/// parser.

void File__tag_match(File in_file, String tag_name) {
    while (1) {
        Character character = File__character_read(in_file);
	if (character == '<') {
	    break;
	} else if (character == ' ') {
	    // Do nothing:
	} else {
	    assert(0);
	}
    }
    File__string_match(in_file, tag_name);
}

