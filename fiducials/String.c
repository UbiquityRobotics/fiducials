// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>

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
    return (String)Memory__allocate(size + 1);
}

/// @brief Return a formatted version of *format*.
/// @param format is the string to format.
/// @param ... are the addition arguments to be formatted.
/// @returns the formatted string.
///
/// *String__format*() will a formatted version of *format* using the
/// additional variadic arguements.

String String__format(String format, ...) {
    // Set up *variadic_arguments to start after *format*:
    va_list variadic_arguments;
    va_start(variadic_arguments, format);

    // Compute *formatted_size*:
    char buffer[2];
    Unsigned formatted_size = vsnprintf(buffer, 0, format, variadic_arguments);
    // Allocated *formatted*:
    String formatted = (String)Memory__allocate(formatted_size + 1);

    // Format *formatted*:
    (void)vsnprintf(formatted, formatted_size, format, variadic_arguments);

    return formatted;
}

/// @brief will free memory assciated with *string*.
/// @param string to free.

void String__free(String string) {
    Memory__free((Memory)string);
}

