// Copyright (c) by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Arc.h"
#include "File.h"
#include "Double.h"
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

    Double pi = 3.14159265358979323846264;
    Double degrees_to_radians = pi / 180.0;

    // Some angles (in radians) to use:
    Double angle0   =   0.0 * degrees_to_radians;
    Double angle10  =  10.0 * degrees_to_radians;
    Double angle20  =  20.0 * degrees_to_radians;
    Double angle30  =  30.0 * degrees_to_radians;
    Double angle40  =  40.0 * degrees_to_radians;
    Double angle45  =  45.0 * degrees_to_radians;
    Double angle90  =  90.0 * degrees_to_radians;
    Double angle135 = 135.0 * degrees_to_radians;
    Double angle180 = 180.0 * degrees_to_radians;

    Double square_root_200 = Double__square_root(10.0 * 10.0 + 10.0 * 10.0);
    Double square_root_50 = Double__square_root(5.0 * 5.0 + 5.0 * 5.0);

    // The test map looks as follows:
    //
    //  3---2
    //  |\ /|
    //  | 4 |
    //  |/ \|
    //  0---1

    Tag tag0 = Map__tag_lookup(map1, 0);
    Tag__initialize(tag0, 0.0, 0.0, 0.0, visit);
    Tag tag1 = Map__tag_lookup(map1, 1);
    Tag__initialize(tag1, 0.0, 0.0, 0.0, visit);
    Tag tag2 = Map__tag_lookup(map1, 2);
    Tag__initialize(tag2, 0.0, 0.0, 0.0, visit);
    Tag tag3 = Map__tag_lookup(map1, 3);
    Tag__initialize(tag3, 0.0, 0.0, 0.0, visit);
    Tag tag4 = Map__tag_lookup(map1, 4);
    Tag__initialize(tag4, 0.0, 0.0, 0.0, visit);

    // Sides:
    Arc__create(tag0, tag1, 10.0, angle0   -     0.0,  angle10, 0.0);
    Arc__create(tag0, tag3, 10.0, angle90  -     0.0,  angle30, 0.0);
    Arc__create(tag1, tag2, 10.0, angle90  - angle10,  angle10, 0.0);
    Arc__create(tag2, tag3, 10.0, angle180 - angle20,  angle10, 0.0);

    // Arcs to center:
    Arc__create(tag0, tag4, square_root_50,   angle45 - angle0,  angle40, 0.0);
    Arc__create(tag1, tag4, square_root_50,  angle135 - angle10, angle30, 0.0);
    Arc__create(tag2, tag4, square_root_50, -angle135 - angle20, angle20, 0.0);
    Arc__create(tag3, tag4, square_root_50,  -angle45 - angle30, angle10, 0.0);

    // Large diagonals:
    Arc__create(tag0, tag2, square_root_200,  angle45 -     0.0, angle20, 0.0);
    Arc__create(tag1, tag3, square_root_200, angle135 - angle10, angle20, 0.0);

    Map__update(map1);

    String xml_file_name = "Map_Test.xml";
    Map__save(map1, xml_file_name);
    Map map2 = Map__restore(xml_file_name);

    assert (Map__compare(map1, map2) == 0);

    return 0;
}

void Map__build(Map map) {
}
