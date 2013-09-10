// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "File.h"
#include "Float.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "Map.h"
#include "Neighbor.h"
#include "String.h"
#include "Tag.h"
#include "Unsigned.h"

extern void Map__build(Map map);


Integer main(Unsigned arguments_size, String arguments[]) {
    Map map1 = Map__new();
    Tag tag5 = Map__tag_lookup(map1, 5);
    Tag tag4 = Map__tag_lookup(map1, 4);
    Tag tag3 = Map__tag_lookup(map1, 3);
    Tag tag2 = Map__tag_lookup(map1, 2);
    Tag tag1 = Map__tag_lookup(map1, 1);

    String xml_file_name = "Map_Test.xml";
    Map__save(map1, xml_file_name);
    Map map2 = Map__restore(xml_file_name);

    assert (Map__compare(map1, map2) == 0);

    return 0;
}

void Map__build(Map map) {
}
