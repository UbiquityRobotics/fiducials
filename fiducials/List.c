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
