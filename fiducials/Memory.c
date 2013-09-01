// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdlib.h>
#include "Unsigned.h"
#include "Memory.h"

/// @brief Allocates *bytes* of memory and returns a pointer to it.
/// @param bytes is the number of bytes to allocate.
/// @returns a pointer to the allocated memory chunk.
///
/// *Memory__allocate*() will allocated and return a pointer to a chunk
/// of *bytes* memory.

Memory Memory__allocate(Unsigned bytes) {
    Memory memory = (Memory)malloc(bytes);
    assert (memory != (Memory)0);
    return memory;
}

/// @brief Releases the storage associated with *memory*.
/// @param memory to release.
///
/// *Memory__free*() will release the storage associated with *memory*.

void Memory__free(Memory memory) {
    free(memory);
}
