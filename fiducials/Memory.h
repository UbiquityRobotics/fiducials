// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(MEMORY_H_INCLUDED)
#define MEMORY_H_INCLUDED 1

/// @brief Allocate a *Type* object from the heap.
///
/// The type must be a pointer type where the underlying type defintion
/// is available to the compiler.
///
/// This allocates an object of type *Type* by doing "Memory_new(Type)".
/// This macro when expanded works as follows:
///
///        Type zilch = (Type)0;
///        UInteger type_size = sizeof(*zilch);
///        Type type = (Type)Memory_allocate(type_size);
///
/// "sizeof(*((Type)0))" does not generate any code.  The compiler
/// evaluates it to get the number of bytes associated with "Type":

#define Memory__new(Type) ((Type)Memory__allocate(sizeof(*((Type)0))))

/// @brief *Memory* is a pointer to memory.
typedef void *Memory;

#include "Unsigned.h"

// Extern declarations:

extern Memory Memory__allocate(Unsigned bytes);
extern void Memory__free(Memory memory);


#endif // !defined(MEMORY_H_INCLUDED)

