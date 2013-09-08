// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(LIST_H_INCLUDED)
#define LIST_H_INCLUDED 1

typedef struct List__Struct *List;

#include "Memory.h"
#include "Unsigned.h"

struct List__Struct {
    Memory *items;
    Unsigned limit;
    Unsigned size;
};

// *List* routines:

extern List List__new(void);
extern void List__append(List list, Memory item);
extern Memory List__fetch(List list, Unsigned index);
extern Unsigned List__size(List list);

#endif // !defined(LIST_H_INCLUDED)
