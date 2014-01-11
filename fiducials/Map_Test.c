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


int main(int arguments_size, char * arguments[]) {
    Map map1 = Map__new((void *)0, Map__tag_announce);
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

    Unsigned tag_size = 1.0;
    Tag tag0 = Map__tag_lookup(map1, 0);
    Tag__initialize(tag0, 0.0, 0.0, 0.0, tag_size, visit);
    Tag tag1 = Map__tag_lookup(map1, 1);
    Tag__initialize(tag1, 0.0, 0.0, 0.0, tag_size, visit);
    Tag tag2 = Map__tag_lookup(map1, 2);
    Tag__initialize(tag2, 0.0, 0.0, 0.0, tag_size, visit);
    Tag tag3 = Map__tag_lookup(map1, 3);
    Tag__initialize(tag3, 0.0, 0.0, 0.0, tag_size, visit);
    Tag tag4 = Map__tag_lookup(map1, 4);
    Tag__initialize(tag4, 0.0, 0.0, 0.0, tag_size, visit);

    // The tags are twisted by the tag id x 10 degerees.  Thus, tag
    // 0 has no twist, tag 1 is 10 degerees, ..., and tag 4 is 40 degrees.

    // Sides:
    Double d = 10.0;
    Arc__create(tag0, 0.0 + 0.0,           d, tag1, -angle180 + angle10, 0.0);
    Arc__create(tag1, -angle90 + angle10,  d, tag2,  angle90  + angle20, 0.0);
    Arc__create(tag0, -angle90,            d, tag3,  angle90  + angle30, 0.0);
    Arc__create(tag2, -angle180 + angle20, d, tag3,  0.0      + angle30, 0.0);

    // Arcs to center:
    d = square_root_50;
    Arc__create(tag0, -angle45,            d, tag4,  angle135 + angle40, 0.0);
    Arc__create(tag1, -angle135 + angle10, d, tag4,  angle135 + angle40, 0.0);
    Arc__create(tag2, -angle45  + angle20, d, tag4,  -angle45 + angle40, 0.0);
    Arc__create(tag3,  angle45  + angle30, d, tag4, -angle135 + angle40, 0.0);

    // Large diagonals:
    d = square_root_200;
    //Arc__create(tag0, ??, square_root_200, tag2, ??, 0.0);
    //Arc__create(tag1, ??, square_root_200, tag3, ??, 0.0);

    Map__update(map1);

    String xml_file_name = "Map_Test.xml";
    Map__save(map1, xml_file_name);
    Map map2 = Map__restore(xml_file_name);

    assert (Map__compare(map1, map2) == 0);

    List /*<Location>*/ locations = List__new();
    Map__svg_write(map1, "Map_Test", locations);

    return 0;
}

void Map__build(Map map) {
}
