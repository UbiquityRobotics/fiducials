// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Arc.h"
#include "File.h"
#include "Float.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "Map.h"
#include "String.h"
#include "Tag.h"
#include "Unsigned.h"

extern void Map__build(Map map);


Integer main(Unsigned arguments_size, String arguments[]) {
    Map map1 = Map__new();
    Unsigned visit = map1->visit;

    Tag tag1 = Map__tag_lookup(map1, 1);
    Tag__initialize(tag1, 0.0, 0.0, 0.0, visit);
    Tag tag2 = Map__tag_lookup(map1, 2);
    Tag__initialize(tag2, 0.0, 10.0, 0.0, visit);
    Tag tag3 = Map__tag_lookup(map1, 3);
    Tag__initialize(tag3, 0.0, 10.0, 10.0, visit);
    Tag tag4 = Map__tag_lookup(map1, 4);
    Tag__initialize(tag4, 0.0, 0.0, 10.0, visit);
    Tag tag5 = Map__tag_lookup(map1, 5);
    Tag__initialize(tag5, 0.0, 5.0, 5.0, visit);


    Float square_root_200 = Float__square_root(10.0 * 10.0 + 10.0 * 10.0);
    Float square_root_50 = Float__square_root(5.0 * 5.0 + 5.0 * 5.0);

    // Sides:
    Arc Arc1_2 = Arc__create(tag1, tag2, 10.0, 0.0, 0.0, 0.0);
    Arc Arc1_4 = Arc__create(tag1, tag4, 10.0, 0.0, 0.0, 0.0);
    Arc Arc2_3 = Arc__create(tag2, tag3, 10.0, 0.0, 0.0, 0.0);
    Arc Arc3_4 = Arc__create(tag3, tag4, 10.0, 0.0, 0.0, 0.0);

    // Arcs to center:
    Arc Arc1_5 = Arc__create(tag1, tag5, square_root_50, 0.0, 0.0, 0.0);
    Arc Arc2_5 = Arc__create(tag2, tag5, square_root_50, 0.0, 0.0, 0.0);
    Arc Arc3_5 = Arc__create(tag3, tag5, square_root_50, 0.0, 0.0, 0.0);
    Arc Arc4_5 = Arc__create(tag4, tag5, square_root_50, 0.0, 0.0, 0.0);

    // Large diagonals:
    Arc Arc1_3 = Arc__create(tag1, tag3, square_root_200, 0.0, 0.0, 0.0);
    Arc Arc2_4 = Arc__create(tag2, tag4, square_root_200, 0.0, 0.0, 0.0);

    Map__update(map1);

    String xml_file_name = "Map_Test.xml";
    Map__save(map1, xml_file_name);
    Map map2 = Map__restore(xml_file_name);

    assert (Map__compare(map1, map2) == 0);

    return 0;
}

void Map__build(Map map) {
}
