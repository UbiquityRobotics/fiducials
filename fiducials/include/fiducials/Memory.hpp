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
///        Uint type_size = sizeof(*zilch);
///        Type type = (Type)Memory_allocate(type_size);
///
/// "sizeof(*((Type)0))" does not generate any code.  The compiler
/// evaluates it to get the number of bytes associated with "Type":

#include "String.hpp"

#define Memory__new(Type, from) \
  ((Type)Memory__allocate(sizeof(*((Type)0)), from))

/// @brief *Memory* is a pointer to memory.
typedef void *Memory;

// Extern declarations:

#if defined(MEMORY_LEAK_CHECK)
    extern void Memory__leak_check(Memory memory);
    extern void Memory__leak_found(Memory memory);
#endif // defined(MEMORY_LEAK_CHECK)

extern Memory Memory__allocate(unsigned int bytes, String_Const from);
extern void Memory__free(Memory memory);
extern Memory Memory__reallocate(Memory memory, unsigned int new_size,
    String_Const from);
extern Memory unsigned__to_memory(unsigned int unsigned1);

#endif // !defined(MEMORY_H_INCLUDED)

