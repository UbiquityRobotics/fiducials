// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Character.h"
#include "Logical.h"
#include "String.h"
#include "Memory.h"
#include "Unsigned.h"

/// @brief Allocate and return for *size* characters.
/// @param size is the maximum string size excluding trailing null.
/// @returns allocated *String* object.
///
/// *String__allocate*() will return a string that can conatain
/// *size* characters excluding trailing null.

String String__allocate(Unsigned size) {
    return (String)Memory__allocate(size + 1, "String__allocate");
}

/// @brief Returns true if *string1* equals *string2*.
/// @param string1 is the first *String* to compare.
/// @param string2 is the second *String* to compare.
/// @returns true if *string1* is equal to *string2*.
///
/// *String__equal*() will return true if *string1* is equal to *string2*
/// and false otherwise.

Logical String__equal(String_Const string1, String_Const string2) {
    return (Logical)(strcmp(string1, string2) == 0);
}

/// @brief Return a formatted version of *format*.
/// @param format is the string to format.
/// @param ... are the addition arguments to be formatted.
/// @returns the formatted string.
///
/// *String__format*() will a formatted version of *format* using the
/// additional variadic arguements.

String String__format(String_Const format, ...) {
    // Set up *variadic_arguments to start after *format*:
    va_list variadic_arguments;
    va_start(variadic_arguments, format);

    // Compute *formatted_size*:
    char buffer[2];
    Unsigned formatted_size = vsnprintf(buffer, 0, format, variadic_arguments);
    // Allocated *formatted*:
    String formatted =
      (String)Memory__allocate(formatted_size + 1, "String__format");

    // Format *formatted*:
    va_start(variadic_arguments, format);
    (void)vsnprintf(formatted, formatted_size + 1, format, variadic_arguments);

    return formatted;
}

/// @brief will free memory assciated with *string*.
/// @param string to free.

void String__free(String_Const string) {
    Memory__free((Memory)string);
}

/// @brief Returns the size of *string*.
/// @param string to get size of.
/// @returns size of string.
///
/// *String__size*() will return the size size of *string*.

Unsigned String__size(String_Const string) {
    Unsigned size = 0;
    while (*string++ != '\0') {
	size += 1;
    }
    return size;
}


/// @brief Converts from decimal string into a number and return it.
/// @param string to convert.
///
/// *String__to_unsigned*() will convert from decimal string into a
/// number and return it.

Unsigned String__to_unsigned(String_Const string) {
    Unsigned result  = 0;
    while (1) {
	Character character = *string++;
	if (Character__is_decimal_digit(character)) {
	    result = result * 10 + (character - '0');
	} else {
	    break;
	}
    }
    return result;
}
