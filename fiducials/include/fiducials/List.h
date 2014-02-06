// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#if !defined(LIST_H_INCLUDED)
#define LIST_H_INCLUDED 1

#include "Memory.h"
#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief *List* is a pointer to a list object.
typedef struct List__Struct *List;

/// @brief A procedure variable signature for comparing two *Memory* objects.
typedef Integer (*List__Compare__Routine)(Memory, Memory);

/// @brief A procedure variable signature for testing two *Memory* objects
/// for equality.
typedef Logical (*List__Equal__Routine)(Memory, Memory);

/// @brief *List__Struct* is the structure for a list of items.
struct List__Struct {
    /// @brief The items of data.  Always at least 1 item is available.
    Memory *items;

    /// @brief The maximum number of items before a resize is required.
    Unsigned limit;

    /// @brief the current number items in the list.
    Unsigned size;
};

// *List* routines:

extern void List__all_append(List to_list, List from_list);
extern void List__append(List list, Memory item, String from);
extern Memory List__fetch(List list, Unsigned index);
extern void List__free(List list);
extern List List__new(String from);
extern Memory List__pop(List list);
extern Unsigned List__size(List list);
extern void List__sort(List list1, List__Compare__Routine compare_routine);
extern void List__trim(List list, Unsigned new_size);
extern void List__unique(List list, List__Equal__Routine equal_routine);

#ifdef __cplusplus
}
#endif
#endif // !defined(LIST_H_INCLUDED)
