// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(LIST_H_INCLUDED)
#define LIST_H_INCLUDED 1

typedef struct List__Struct *List;

#include "Memory.h"
#include "Unsigned.h"

/// @brief A procedure variable signature for comparing two *Memory* objects.
typedef Integer (*List__Compare__Routine)(Memory, Memory);

struct List__Struct {
    Memory *items;
    Unsigned limit;
    Unsigned size;
};

// *List* routines:

extern List List__new(void);
extern void List__sort(List list1, List__Compare__Routine compare_routine);
extern void List__append(List list, Memory item);
extern Memory List__fetch(List list, Unsigned index);
extern Unsigned List__size(List list);

#endif // !defined(LIST_H_INCLUDED)
