// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include "List.h"
#include "Memory.h"
#include "Unsigned.h"

List List__new(void) {
    List list = Memory__new(List);
    list->items = (Memory *)Memory__allocate(sizeof(Memory));
    list->size = 0;
    list->limit = 1;
    return list;
}

void List__append(List list, Memory item) {
    Memory *items = list->items;
    Unsigned size = list->size;
    Unsigned limit = list->limit;
    if (size >= limit) {
	limit <<= 1;
	items =
	  (Memory *)Memory__reallocate((Memory)items, limit * sizeof(Memory));
	list->limit = limit;
	list->items = items;
    }
    items[size] = item;
    list->size = size + 1;
}

Memory List__fetch(List list, Unsigned index) {
    assert(index < list->size);
    return list->items[index];
}

Unsigned List__size(List list) {
    return list->size;
}

/// @brief Sort *list* using *compare_routine* to compare elements.
/// @param list to sort.
/// @param compare_routine is a comparison routine that is called each
///        time to values in *list* need to be compared.
///
/// This routine will sort the contents of *list* using *compare_routine*
/// to compare values in *list*.  This algorithm takes O(N log N) time
/// to execute.

void List__sort(List list, List__Compare__Routine compare_routine) {
    Unsigned size = list->size;
    Unsigned power = 0;
    Unsigned temp = 1;
    while (temp < size) {
	power++;
	temp <<= 1;
    }

    // We need a temporary area, so stash them on the end of *list*:
    Unsigned index = 0;
    while (index < size) {
	Memory mem = list->items[index];
	(void)List__append(list, mem);
	index++;
    }

    // Figure out where to start sorting:
    Unsigned mem1 = 0;
    Unsigned mem2 = 0;
    if ((power & 1) == 1) {
	mem1 = size;
	mem2 = 0;
    } else {
	mem1 = 0;
	mem2 = size;
    }

    // Now sort pairs, then quads, then octettes, etc.:
    Unsigned step1 = 1;
    Unsigned step2 = 2;
    index = power;
    while (index != 0) {
	Unsigned index2 = mem2;
	Unsigned index1a = mem1;
	Unsigned index1b = mem1 + step1;
	Unsigned offset = 0;
	while (offset < size) {
	    Unsigned count1a = step1;
	    Unsigned count1b = step1;
	    temp = size - offset;
	    if (temp < step1) {
		count1a = temp;
		count1b = 0;
	    } else {
		temp = temp - step1;
		if (temp < step1) {
		    count1b = temp;
		}
	    }

	    while (count1a != 0 && count1b != 0) {
		Memory mem1a = list->items[index1a];
		Memory mem1b = list->items[index1b];
		if (compare_routine(mem1a, mem1b) < 0) {
		    list->items[index2] = mem1a;
		    index1a++;
		    count1a--;
		} else {
		    list->items[index2] = mem1b;
		    index1b++;
		    count1b--;
		}
		index2++;
	    }

	    while (count1a != 0) {
		list->items[index2] = list->items[index1a];
		index1a++;
		count1a--;
		index2++;
	    } while (count1b != 0) {
		list->items[index2] = list->items[index1b];
		index1b++;
		count1b--;
		index2++;
	    }
	    index1a += step1;
	    index1b += step1;
	    offset += step2;
	}
	Unsigned mem_temp = mem1;
	mem1 = mem2;
	mem2 = mem_temp;
	step1 <<= 1;
	step2 <<= 1;
	index--;
    }

    list->size = size;
 
    // Verify that we are properly sorted:
    for (index = 1; index < size; index++) {
	Memory mem_before = list->items[index - 1];
	Memory mem_after = list->items[index];
	assert (compare_routine(mem_before, mem_after) <= 0);
    }
}


