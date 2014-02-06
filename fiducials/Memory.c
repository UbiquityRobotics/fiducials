// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <stdlib.h>
#include "File.h"
#include "Memory.h"
#include "Unsigned.h"

#if defined(MEMORY_LEAK_CHECK)
    static File Memory__allocate_file = (File)0;
    static File Memory__free_file = (File)0;
    static Memory Memory__leak = (Memory)0;
#endif // defined(MEMORY_LEAK_CHECK)

/// @brief Allocates *bytes* of memory and returns a pointer to it.
/// @param bytes is the number of bytes to allocate.
/// @returns a pointer to the allocated memory chunk.
///
/// *Memory__allocate*() will allocated and return a pointer to a chunk
/// of *bytes* memory.

Memory Memory__allocate(Unsigned bytes, String from) {
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

/// @brief Expands/contracts *memory* to be *new_size* bytes.
/// @param memory to expand or contract.
/// @param new_size is the new size of the memory segement.
/// @returns *memory* or a copy of *memory*.
///
/// *Memory__reallocate*() will either *resize* *memory* to be *new_bytes*
/// bytes in size, or allocate a new chunk of memory that is *new_bytes* in
/// size.  If the later case, the previous contents of memory is copied over
/// before releasing the original storage.

Memory Memory__reallocate(Memory memory, Unsigned new_size, String from) {
    #if defined(MEMORY_LEAK_CHECK)
	Memory new_memory = (Memory)malloc(new_size);
	assert(new_memory != (Memory)0);

	// This is a tad ugly.  We do not know the old size of *memory*,
	// but we need to copy the contents of *memory* into *new_memory*.
	// The solution is to copy *new_size* bytes from *memory* to
	// completely fill up *new_memory*.  This may run off the end
	// of *memory*, but it is unlikely that we cause a memory fault.
	String new_string = (String)new_memory;
	String old_string = (String)memory;
	for (Unsigned index = 0; index < new_size; index++) {
	    new_string[index] = old_string[index];
	}

	// New record the reallocation as an allocate and a free:
	assert(Memory__allocate_file != (File)0);
	assert(Memory__free_file != (File)0);
	File__format(Memory__allocate_file,
	  "0x%08x %s_0x%08x\n", new_memory, from, memory);
	File__flush(Memory__allocate_file);
	File__format(Memory__free_file, "0x%08x\n", memory);
	File__flush(Memory__free_file);

	// Now check for a memory leak match:
	if (new_memory == Memory__leak) {
	    Memory__leak_found(new_memory);
	}
    #else
	Memory new_memory = realloc(memory, new_size);
	assert (new_memory != (Memory)0);
    #endif // defined(MEMORY_LEAK_CHECK)
    return new_memory;
}

/// @brief Return *unsigned1* as a *Memory* pointer.
/// @param unsigned1 is the value to be treated as *Memory*
/// @returns *unsigned1* as a *Memory* pointer.
///
/// *Unsigned__to_memory*() returns *unsigned1* as a *Memory* pointer.

Memory Unsigned__to_memory(Unsigned unsigned1) {
    union {
        Unsigned unsigned1;
        Memory memory;
    } convert;
    convert.unsigned1 = unsigned1;
    return convert.memory;
}
