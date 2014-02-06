// Copyright (c) 2013 by Hasbro, Inc.  All rights reserved.

#include <assert.h>

#include "Integer.h"
#include "Logical.h"
#include "Memory.h"
#include "String.h"
#include "Table.h"
#include "Unsigned.h"

// *Table* routines:

/// @brief Returns a newly created table for string key/binding
///        associatons.
/// @param equal_routine is a procedure variable that is used to determine
///        if two keys are equal.
/// @param hash_routine is a procedure variable that is used to compute a
///        32-bit hash of a key.
/// @param empty_value is a value that is returned on lookup failure.
///
/// *Table__create*() will create and return an empty table for
/// storing key/value bindings.  *equal_routine*() is used to test key equality.
/// *hash_routine* is used to compute a 32-bit hash value of key.
/// *empty_value* is returned when a lookup fails.

Table Table__create(Table_Equal_Routine equal_routine,
  Table_Hash_Routine hash_routine, Memory empty_value, String from)
{
    // Allocate and initialize the *table_lists* object:
    Unsigned table_lists_size = 8;
    Table_List *table_lists = (Table_List *)Memory__allocate(
    table_lists_size * sizeof(Table_List), from);
    for (Unsigned index = 0; index < table_lists_size; index++)
    {
	table_lists[index] = Table_List__new();
    }

    // Build the *table* object:
    Table table = Memory__new(Table, from);
    table->table_lists = table_lists;
    table->table_lists_size = table_lists_size;
    table->empty_value = empty_value;
    table->equal_routine = equal_routine;
    table->hash_routine = hash_routine;
    table->size = 0;
    table->threshold = 6;
    return table;
}

/// @brief Frees the storage associated with *table*.
/// @param table is the *Table* object to free.
///
/// *Table__free*() will free the storage associated with *table*.  The
/// various keys and values that are in *table* are not freed.

void Table__free(Table table)
{
    Table_List *table_lists = table->table_lists;
    Unsigned size = table->table_lists_size;
    for (Unsigned index = 0; index < size; index++)
    {
	Table_List table_list = table_lists[index];
	Table_List__free(table_list);
    }
    Memory__free((Memory)table);
}

/// @brief Returns true if *key* is in *table*.
/// @param table is the table to search *key* for.
/// @param key is the key to look for in *table*.
/// @returns true (1) for success and false (0) otherwise.
///
/// *Table__has_key*() returns true if *key* is in *table* and false
/// otherwise.

Logical Table__has_key(Table table, Memory key)
{
    // Search for the binding:
    Unsigned hash = table->hash_routine(key);
    Unsigned mask = table->table_lists_size - 1;
    Table_List table_list = table->table_lists[hash & mask];
    Table_Triple table_triple =
      Table_List__search(table_list, hash, key, table->equal_routine);

    // Return result:
    return table_triple != (Table_Triple)0;
}

/// @brief Inserts a *key*-*value* binding into *table*.
/// @param table is the table to insert the *key*-*value* binding into.
/// @param key is the key portion of the key/value binding.
/// @param value is the value portion of the key/value binding.
///
/// *Table__insert*() inserts *key*-*value* as a binding into *table*.
/// An assertion failure occurs if *table* already has *key* in it.

void Table__insert(Table table, Memory key, Memory value)
{
    // See whether we need to resize.
    Unsigned size = table->size;
    if (size >= table->threshold)
    {
	Table__resize(table);
    }
  
    // Look for the *key*-*value* binding:
    Unsigned hash = table->hash_routine(key);
    Unsigned mask = table->table_lists_size - 1;
    Table_List *table_list_slot = &table->table_lists[hash & mask];
    Table_List table_list = *table_list_slot;
    Table_Triple table_triple =
      Table_List__search(table_list, hash, key, table->equal_routine);

    // Ensure that we did not find the binding:
    assert (table_triple == (Table_Triple)0);

    // Append the appropriate triple values to *table_list*:
    *table_list_slot = Table_List__append(table_list, hash, key, value);

    // Increment the size:
    table->size = size + 1;
}

/// @brief Return the matching key stored in *table* that matches *key*.
/// @param table is the table to search for *key* in.
/// @param key is the key value to match in *table*.
/// @returns the key from *table* that matches *key* on success and and null
///        empty otherwise.
///
/// *Table__key_lookup*() will return the key stored in *table* that matches
/// *key*.  If *key* is matched*, null is returned.

Memory Table__key_lookup(Table table, Memory key)
{
    // Search for the *table_triple* associated with *key*:
    Unsigned hash = table->hash_routine(key);
    Unsigned mask = table->table_lists_size - 1;
    Table_List table_list = table->table_lists[hash & mask];
    Table_Triple table_triple =
      Table_List__search(table_list, hash, key, table->equal_routine);

    // Return null on failure and the the actual key on success:
    key = (Memory)0;
    if (table_triple != (Table_Triple)0)
    {
	key = table_triple->key;
    }
    return key;
}

/// @brief Return the value associated *key* in *table*.
/// @param table is the table to search for *key* in.
/// @param key is the key value to search for in *table*.
/// @returns the value associted with *key* on success and an empty value
///          otherwise.
///
/// *Table__lookup*() will return the value associated with *key* in
/// *table*. If *key* is not in *table*, the empty value that was provided when
/// *table* was created by *Table__create*().

Memory Table__lookup(Table table, Memory key)
{
    // Search for the *table_triple* associated with *key*:
    Unsigned hash = table->hash_routine(key);
    Unsigned mask = table->table_lists_size - 1;
    Table_List table_list = table->table_lists[hash & mask];
    Table_Triple table_triple =
      Table_List__search(table_list, hash, key, table->equal_routine);

    // Return *empty_value* on failure and the the actual value on success:
    Memory value = table->empty_value;
    if (table_triple != (Table_Triple)0)
    {
	value = table_triple->value;
    }
    return value;
}

/// @brief Replace the *value* associated with *key* in *table*.
/// @param table is the table to replace the *key*-*value* binding in.
/// @param key is the key to search for in *table*.
/// @param value is the new value to bind to *key*.
///
/// *Table__replace*() will replace the value asociated with *key* in
/// *table* with *value*.  This routine triggers an assertion failure if *key*
/// is not in *table*.

void Table__replace(Table table, Memory key, Memory value)
{
    /// Search for the *table_triple* associated with *key*:
    Unsigned hash = table->hash_routine(key);
    Unsigned mask = table->table_lists_size - 1;
    Table_List table_list = table->table_lists[hash & mask];
    Table_Triple table_triple =
      Table_List__search(table_list, hash, key, table->equal_routine);

    // Make sure that we found a matching *key*:
    assert (table_triple != (Table_Triple)0);

    // Replace the *value*:
    table_triple->value = value;
}

/// @brief Double the number of slots in *table*.
/// @param table is the table to resize.
///
/// *Table__resize*() will double the number of slots available in *table*.
/// Hash table performance is kept up by keeping the table only about
/// 75% full.

void Table__resize(Table table)
{
    // Double the size of *table_lists*:
    Unsigned table_lists_size = table->table_lists_size;
    Unsigned new_table_lists_size = table_lists_size << 1;

    // Update the threshold for about 75% full:
    table->table_lists_size = new_table_lists_size;
    table->threshold = new_table_lists_size * 3 / 4;

    // Make sure there is enough storage for the new *table_lists_size* slots:
    Table_List *table_lists =
      (Table_List*)Memory__reallocate((Memory)table->table_lists,
      sizeof(Table_List) * new_table_lists_size, "Table__resize");

    // Initialize the *table_list_size* slots added to the end of *table_lists*:
    for (Unsigned index = 0; index < table_lists_size; index++)
    {
	table_lists[table_lists_size + index] = Table_List__new();
    }
    table->table_lists = table_lists;

    // Now rehash the all the table_lists:
    Unsigned mask = new_table_lists_size - 1;
    for (Unsigned table_lists_index = 0;
      table_lists_index < table_lists_size; table_lists_index++)
    {
 	// The *old_table_list* has some triples that need to be moved
	// over to *new_table_list*:
	Table_List old_table_list = table_lists[table_lists_index];
	Table_List *new_table_list_slot =
	  &table_lists[table_lists_index + table_lists_size];
	Table_List new_table_list = *new_table_list_slot;

	// Carefully move the appropriate triples over:
	Unsigned size = old_table_list->size;
	Unsigned index = 0;
	while (index < size)
	{
	    Table_Triple table_triple =
	      Table_List__fetch(old_table_list, index);
	    Unsigned hash = table_triple->hash;
	    if ((hash & mask) == table_lists_index)
	    {
		// This triple is fine on *old_dist_list*:
		index += 1;
	    } else
	    {
		// This triple belongs on *new_dist_list*:
		new_table_list = Table_List__append(new_table_list,
		  hash, table_triple->key, table_triple->value);
		Table_List__delete(old_table_list, index);

		// Notice that size is decreased by one, since the previous
		// delete operation overwrote the contents at {index}:
		size -= 1;
	    }
	}

	// Restore *new_table_list* back into *table_lists*:
	*new_table_list_slot = new_table_list;
    }
}

/// @brief Output the contents of *table* to *file*.
/// @param table is the table to show.
/// @param file is the file to output to.
///
/// *Table__show*() will output the contents of *table* to *file* using
/// the key/value show routines that were supplied to *table* by calling
/// *Table__show_enable*().

void Table__show(Table table, File file)
{
    // Grab some values from *table*:
    Unsigned size = table->size;
    Unsigned table_lists_size = table->table_lists_size;
    Table_Key_Show_Routine key_show_routine = table->key_show_routine;
    Table_Value_Show_Routine value_show_routine = table->value_show_routine;
    assert(key_show_routine != (Table_Key_Show_Routine)0);
    assert(value_show_routine != (Table_Value_Show_Routine)0);

    // Print out a header line:
    File__format(file, "Table: size=%d threshold=%d table_lists_size=%d\n",
      size, table->threshold, table_lists_size);

    // Iterate through all of the *table_lists*:
    Table_List *table_lists = table->table_lists;
    for (Unsigned index = 0; index < table_lists_size; index++)
    {
        // Output each *table_list*:
	Table_List table_list = table_lists[index];
	File__format(file, "[%d]:", index);
	Table_List__show(table_list,
	  file, key_show_routine, value_show_routine);
    }
    File__format(file, "\n");
}

/// @brief Enable *Table__show* by provide by storing both a key and value
///        show routine into *table*.
/// @param table is the table to associate the show routines with.
/// @param key_show_routine is a procedure that is used to output the
///        contents of a key to a file.
/// @param value_show_routine is a procedure that is used to output the
///        contents of a value to a file.
///
/// *Table__show_enable*() will set both the key and value show routines
/// for use by the *Table__show*() routine.  These two routine are stored
/// into *table*.

void Table__show_enable(
  Table table, Table_Key_Show_Routine key_show_routine,
  Table_Value_Show_Routine value_show_routine)
{
    table->key_show_routine = key_show_routine;
    table->value_show_routine = value_show_routine;
}

// *Table__List* routines:

/// @brief Append a (*hash*, *key*, *value*) triple to the end fo *table_list*.
/// @param table_list is *Table__List* object to append to.
/// @param hash is the has value to append.
/// @param key is the key to append.
/// @param value is the value to append.
/// @returns the potentially new location for *table_list*.
///
/// *Table__list_append*() will append the (*hash*, *key*, *value*)
/// triple to the end of *table_list*.  If additional space is needed for
/// *Table__List,  it may get relocated to a new location.  The
/// location that contains the newly appended triple is always returned.

Table_List Table_List__append(Table_List table_list,
  Unsigned hash, Memory key, Memory value)
{
    // See whether we need to resize:
    Unsigned size = table_list->size;
    if (table_list->size >= table_list->available)
    {
        table_list = Table_List__resize(table_list);
    }

    // Load up *table_triple*:
    Table_Triple table_triple = &table_list->table_triples[size];
    table_triple->hash = hash;
    table_triple->key = key;
    table_triple->value = value;

    // Bump the size and return:
    table_list->size = size + 1;
    return table_list;
}

/// @brief Delete the hash-key-value triple at *index* in *table_list*.
/// @param table_list to delete triple from.
/// @param index the index into *table-list* to delete.
///
/// *Table__List__delete*() will delete the *index*'th triple from
/// *table_list*.  This is done by copying the last triple into the
/// *index*'th location.

void Table_List__delete(Table_List table_list, Unsigned index)
{
    // Make sure *index* and *size are in bounds:
    Unsigned size = table_list->size;
    assert (index < size);

    // Grab the *old_triple* (to be deleted) and the *end_triple*:
    Table_Triple old_triple = Table_List__fetch(table_list, index);
    Table_Triple end_triple = Table_List__fetch(table_list, size - 1);

    // Copy contents of *end_triple* on top of *old_triple*:
    old_triple->hash = end_triple->hash;
    old_triple->key = end_triple->key;
    old_triple->value = end_triple->value;

    // Reduce the size by one:
    table_list->size = size - 1;
}

/// @brief Fetch the *index*'th *Table__Triple* from *table_list*.
/// @param table_list to fetch from.
/// @param index is the index to select from *table_list*.
/// @returns the *index*'th table list.
///
/// *Table__List__fetch*() will return the *index*'th *Table__Triple*
/// from  *table_list*.  An assertion failure occurs if *index* is out of bounds.

Table_Triple Table_List__fetch(
  Table_List table_list, Unsigned index)
{
    assert(index < table_list->size);
    Table_Triple table_triple = &table_list->table_triples[index];
    return table_triple;
}

/// @brief Free the storage associated with *table_list*.
/// @param table_list object to free
///
/// *Table__List__free*() will free the storage associatd with *table_list*.
/// Any keys and values in *table_list* are not freed.

void Table_List__free(Table_List table_list)
{
    Memory__free((Memory)table_list);
}

/// @brief Create and return a new empty *Table__List* object.
/// @returns new empty *Table__List* object.
///
/// *Table__List__new*() will crate and return a new empty
/// *Table__List* object.

Table_List Table_List__new(void)
{
    Table_List table_list = Memory__new(Table_List, "Table_List__new");
    table_list->size = 0;
    table_list->available = 0;
    return table_list;
}

/// @brief Doubles the number of available slots in *table_list*.
/// @returns new location of resized *table_list*.
///
/// *Table__List__resize*() will double the number of available slots in
/// *table_list*.  As a side effect of doubling the slots, *table_list*
/// may be relocated to a new location.  The relocated *table_list*
/// is returned.

Table_List Table_List__resize(Table_List table_list)
{
    // Figure out the number of *available* slots neede:
    Unsigned available = table_list->available << 1;
    if (available == 0)
    {
	// We always want at least one slot:
	available = 1;
    }

    // Make sure *table* list has enough slots:
    table_list = (Table_List)Memory__reallocate((Memory)table_list,
      sizeof(struct Table_List_Struct) +
      sizeof(struct Table_Triple_Struct) * available, "Table_List__resize");

    // Update *available* and return:
    table_list->available = available;
    return table_list;
}

/// @brief Return *Table__Triple* object that corresponds to *key* in
///               *table_list*.
/// @param table_list is the *Table__List* object to search.
/// @param hash is the hash value to match.
/// @param key is the key to match.
/// @param equal_routine is a routine that is used to compare two keys
///        for equality.
///
/// *Table__List__search*() returns the *Table__Tripple* associated
/// with *key* in *table_list*.  *hash* must be the correct 32-bit hash for
/// *key*.  *equal_routine* must be a procedure that compares two keys for
/// equality.  If *key* is not found, null is returned.

Table_Triple Table_List__search(Table_List table_list,
  Unsigned hash, Memory key, Table_Equal_Routine equal_routine)
{
    // Search through the triples:
    Unsigned size = table_list->size;
    Table_Triple table_triples = &table_list->table_triples[0];
    for (Unsigned index = 0; index < size; index++)
    {
	Table_Triple table_triple = table_triples + index;
	if (table_triple->hash == hash && equal_routine(table_triple->key, key))
	{
	    // We found a match; return it:
	    return table_triple;
	}
    }

    // No match found; return null:
    return (Table_Triple)0;
}

/// @brief Output table_lis to to file:
/// @param table_list to output.
/// @param file to output to.
/// @param key_show_routine is a procedure that outputs a key to a file.
/// @param value_show_routine is a procedure that outputs a value to a file.
///
/// *Table__List__show*() will output *table_list* to *file*.
/// *key_show_routine*() and *value_show_routine*() are two routines
/// for outputing a key and a value respectively.

void Table_List__show(Table_List table_list,
  File file, Table_Key_Show_Routine key_show_routine,
  Table_Value_Show_Routine value_show_routine)
{
    // Output the header line:
    Unsigned size = table_list->size;
    File__format(file, "%d(of %d) 0x%08x:",
      size, table_list->available, (Memory)table_list);

    // Output each triple:
    for (Unsigned index = 0; index < size; index++)
    {
	Table_Triple table_triple =
	  Table_List__fetch(table_list, index);
	File__format(file, " (0x%08x, ", table_triple->hash);
	key_show_routine(table_triple->key, file);
	File__format(file, ", ");
	value_show_routine(table_triple->value, file);
	File__format(file, ")");
    }
    File__format(file, "\n");
}
