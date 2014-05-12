// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdlib.h>
#include "File.hpp"
#include "Memory.hpp"

#if defined(MEMORY_LEAK_CHECK)
    static File Memory__allocate_file = (File)0;
    static File Memory__free_file = (File)0;
    static Memory Memory__leak = (Memory)0;
#endif // defined(MEMORY_LEAK_CHECK)

/// @brief Allocates *bytes* of memory and returns a pointer to it.
/// @param bytes is the number of bytes to allocate.
/// @param from is a debugging string.
/// @returns a pointer to the allocated memory chunk.
///
/// *Memory__allocate*() will allocated and return a pointer to a chunk
/// of *bytes* memory.

Memory Memory__allocate(unsigned int bytes, String_Const from) {
    Memory memory = (Memory)malloc(bytes);
    assert (memory != (Memory)0);
    #if defined(MEMORY_LEAK_CHECK)
	// Make sure that the logging files are open:
	if (Memory__allocate_file == (File)0) {
	    Memory__allocate_file = File__open("/tmp/memory_allocate.log","w");
	    Memory__free_file = File__open("/tmp/memory_free.log","w");
	    assert (Memory__allocate_file != (File)0);
	    assert (Memory__free_file != (File)0);
	}
	File__format(Memory__allocate_file, "0x%08x %s\n", memory, from);
	File__flush(Memory__allocate_file);

	// Now check for a memory leak match:
	if (memory == Memory__leak) {
	    Memory__leak_found(memory);
	}
    #endif // defined(MEMORY_LEAK_CHECK)
    return memory;
}

#if defined(MEMORY_LEAK_CHECK)
    void Memory__leak_check(Memory memory) {
	Memory__leak = memory;
    }

    void Memory__leak_found(Memory memory) {
	// Plant break point here:
	memory = memory;
    }
#endif // defined(MEMORY_LEAK_CHECK)

/// @brief Releases the storage associated with *memory*.
/// @param memory to release.
///
/// *Memory__free*() will release the storage associated with *memory*.

void Memory__free(Memory memory) {
    #if defined(MEMORY_LEAK_CHECK)
	File__format(Memory__free_file, "0x%08x\n", memory);
	File__flush(Memory__free_file);
    #else
	free(memory);
    #endif // defined(MEMORY_LEAK_CHECK)
}
