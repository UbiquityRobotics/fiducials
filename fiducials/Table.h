// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#ifndef TABLE_H_INCLUDED
#define TABLE_H_INCLUDED 1

typedef struct Table_Struct *Table;
typedef struct Table_List_Struct *Table_List;
typedef struct Table_Triple_Struct *Table_Triple;

#include "File.h"
#include "Integer.h"
#include "Logical.h"
#include "Memory.h"
#include "Unsigned.h"

typedef Logical (*Table_Equal_Routine)(Memory, Memory);
typedef Integer (*Table_Hash_Routine)(Memory);
typedef void (*Table_Key_Show_Routine)(Memory, File);
typedef void (*Table_Value_Show_Routine)(Memory, File);

struct Table_Struct
{
    Table_List *table_lists;
    Unsigned table_lists_size;
    Memory empty_value;
    Table_Equal_Routine equal_routine;
    Table_Hash_Routine hash_routine;
    Unsigned size;
    Unsigned threshold;
    Table_Key_Show_Routine key_show_routine;
    Table_Value_Show_Routine value_show_routine;
};

/// @brief *Table__Triple_Struct* contains a key/value binding.
struct Table_Triple_Struct
{
    Unsigned hash;
    Memory key;
    Memory value;
};

/// @brief *Table__List__Struct* is a resizable list of *Table__Triples*.
struct Table_List_Struct
{
    Unsigned available;
    Unsigned size;
    struct Table_Triple_Struct table_triples[0];
};


// *Table* routines:

extern Table Table__create(Table_Equal_Routine equal_routine,
  Table_Hash_Routine hash_routine, Memory empty_value);
extern void Table__free(Table table);
extern Logical Table__has_key(Table table, Memory key);
extern void Table__insert(Table table, Memory key, Memory value);
extern Memory Table__key_lookup(Table table, Memory key);
extern Memory Table__lookup(Table table, Memory key);
extern void Table__replace(Table table, Memory key, Memory value);
extern void Table__resize(Table table);
extern void Table__show(Table table, File file);
extern void Table__show_enable(Table table, 
  Table_Key_Show_Routine key_show_routine,
  Table_Value_Show_Routine value_show_routine);

// *Table__List* routine:

extern Table_List Table_List__append(
  Table_List table_list, Unsigned hash, Memory key, Memory value);
extern void Table_List__delete(Table_List table_list, Unsigned index);
extern Table_Triple Table_List__fetch(Table_List table_list, Unsigned index);
extern void Table_List__free(Table_List table_list);
extern Table_List Table_List__new(void);
extern Table_List Table_List__resize(Table_List table_list);
extern Table_Triple Table_List__search(
  Table_List table_list, Unsigned hash, Memory key, Table_Equal_Routine equal_routine);
extern void Table_List__show(Table_List table_list, File file,
  Table_Key_Show_Routine key_show_routine,
  Table_Value_Show_Routine value_show_routine);

#endif // TABLE_H_INCLUDED
