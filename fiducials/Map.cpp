// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

/// @brief Global map of ceiling fiducial markers.
///
/// A *Map* consists of a list of *Tag* objects, where each Tag represents
/// the position and orientation of ceiling fiduical markers.  Each *Tag*
/// has zero one or more *Neighbor* objects that specify the distance and
/// orientation of *Tag* pairs.

typedef struct Map__Struct *Map_Doxygen_Fake_Out;

#include <assert.h>
#include <angles/angles.h>

#include "Arc.hpp"
#include "CV.hpp"
#include "Camera_Tag.hpp"
#include "File.hpp"
#include "Location.hpp"
#include "Map.hpp"
#include "Tag.hpp"

// *Map* routines:

/// @brief Causes an arc announce callback routine to be called.
/// @param map is the parent *Map* object.
/// @param arc is the *Arc* object that has just been changed.
/// @param image is the image associated with the arc.
/// @param sequence_number is the sequence number associated with announce.
///
/// *Map__arc_announce*() will cause the arc announde call back routine
/// to be called for *arc*.

void Map__arc_announce(Map map,
  Arc arc, CV_Image image, unsigned int sequence_number) {
   Tag from_tag = arc->from_tag;
   Tag to_tag = arc->to_tag;
   map->arc_announce_routine(map->announce_object,
     from_tag->id, from_tag->x, from_tag->y, from_tag->z,
     to_tag->id, to_tag->x, to_tag->y, to_tag->z,
     arc->goodness, arc->in_tree);
   Map__image_log(map, image, sequence_number);
}

/// @brief Appends *arc* to *map*.
/// @param map to append to.
/// @param arc to append
///
/// *Map__arc_append*() will append *arc* to *map*.

void Map__arc_append(Map map, Arc arc) {
    map->all_arcs.push_back(arc);
    map->changes_count += 1;
    map->is_changed = (bool)1;
    map->is_saved = (bool)0;
}

/// @brief Returns the *Arc* that contains *from_tag* and *to_tag*.
/// @param map that has the *Arc* table.
/// @param from_tag is the from *Tag*.
/// @param to_tag is the to *Tag*.
/// @returns the corresponding *Arc* object.
///
/// *Map__arc_lookup*() will return the *Arc* that contains *from_tag*
/// and *to_tag*.  If no such *Arc* exists yet, it is created.

Arc Map__arc_lookup(Map map, Tag from_tag, Tag to_tag) {
    // Make sure that *from_tag* has the lower id:
    if (from_tag->id > to_tag->id) {
        Tag temporary_tag = from_tag;
        from_tag = to_tag;
        to_tag = temporary_tag;
    }

    // See whether or not an *Arc* with these two tags preexists:
    std::pair<unsigned int, unsigned int> id(from_tag->id, to_tag->id);
    Arc arc;
    if( map->arcs_.count(id) == 0 ) {
        // No preexisting *Arc*; create one:
        arc = Arc__create(from_tag, 0.0, 0.0, to_tag, 0.0, 123456789.0);
        map->arcs_[id] = arc;
    } else {
        arc = map->arcs_[id];
    }
    return arc;
}

/// @brief Makes sure the *Arc* connecting *from* to *to* is up to date.
/// @param map to use for *Arc* updating.
/// @param camera_from is the *Camera_Tag* to for one end of the *Arc*.
/// @param camera_to is the *Camera_Tag* to the other end of the *Arc*.
/// @param image is the image that the *Camera_Tag*'s came from.
/// @param sequence_number is the image sequence number.
/// @returns the number of *Arc*'s updated (1 or 0).
///
/// *Map__arc_update*() will create or update the *Arc* in *map* associated
/// with *from* and *to*.  *image* used to determine the frame size.

unsigned int Map__arc_update(Map map, CameraTag *camera_from, CameraTag *camera_to,
  CV_Image image, unsigned int sequence_number) {
    // Get the *width* and *height*:
    int rows = CV_Image__height_get(image);
    int columns = CV_Image__width_get(image);
    double height = (double)rows;
    double width = (double)columns;

    // Compute some constants:
    double half_width = width / 2.0;
    double half_height = height / 2.0;
    double pi = 3.14159265358979323846264;
    double r2d = 180.0 / pi;

    // Extract some field values from *camera_from*:
    Tag from_tag = camera_from->tag;
    double camera_from_twist = camera_from->twist;
    double camera_from_x = camera_from->x;
    double camera_from_y = camera_from->y;

    // Extract some values from *from_tag*:
    Tag to_tag = camera_to->tag;
    double camera_to_twist = camera_to->twist;
    double camera_to_x = camera_to->x;
    double camera_to_y = camera_to->y;

    // Find associated *Arc* that contains *from_tag* and *to_tag*:
    Arc arc = Map__arc_lookup(map, from_tag, to_tag);

    // Compute the polar distance (in pixels) and angle from the camera
    // center to the *from_tag* center:
    double camera_from_dx = camera_from->x - half_width;
    double camera_from_dy = camera_from->y - half_height;
    double camera_from_polar_distance = hypot(camera_from_dx, camera_from_dy);
    double camera_from_polar_angle = atan2(camera_from_dy, camera_from_dx);

    // Compute the polar_distance (in pixels) and angle from the camera
    // center to the *to_tag* center:
    double camera_to_dx = camera_to_x - half_width;
    double camera_to_dy = camera_to_y - half_height;
    double camera_to_polar_distance = hypot(camera_to_dx, camera_to_dy);
    double camera_to_polar_angle = atan2(camera_to_dy, camera_to_dx);

    // To minimize camera distortion effects, we want to use images where
    // *from* and *to* are about equidistant from the image center.  Thus,
    // we want to minimum the absolute value of the distance difference:
    double goodness = abs(camera_from_polar_distance - camera_to_polar_distance);

    // Now see if the new *goodness* is better than the previous one:
    //File__format(stderr,
    //  "goodness=%.4f arc_goodness=%.4f\n", goodness, arc->goodness);
    unsigned int changed = 0;
    if (goodness < arc->goodness) {
        // We have a better *goodness* metric, compute the new values to
        // load into *arc*:

        // Get two *distance_from_pixel* values which may not be
        // the same because the fiducials are at different heights:
        double from_distance_per_pixel = 
          from_tag->world_diagonal / from_tag->diagonal;
        double to_distance_per_pixel = 
          to_tag->world_diagonal / to_tag->diagonal;

        // Now compute floor to/from X/Y's that coorrespond to the (X,Y)
        // projection of each tag center onto the floor as if the camera
        // is located at the floor origin:
        double from_floor_x = from_distance_per_pixel *
          camera_from_polar_distance * cos(camera_from_polar_angle);
        double from_floor_y = from_distance_per_pixel *
          camera_from_polar_distance * sin(camera_from_polar_angle);
        double to_floor_x = to_distance_per_pixel *
          camera_to_polar_distance * cos(camera_to_polar_angle);
        double to_floor_y = to_distance_per_pixel *
          camera_to_polar_distance * sin(camera_to_polar_angle);

        // Now we can compute the floor distance between the two two
        // projected points:
        double floor_dx = from_floor_x - to_floor_x;
        double floor_dy = from_floor_y - to_floor_y;
        double floor_distance = hypot(floor_dx, floor_dy);

        // Compute *angle* to line segment connecting both tags:
        double camera_dx = camera_to_x - camera_from_x;
        double camera_dy = camera_to_y - camera_from_y;
         double arc_angle = atan2(camera_dy, camera_dx);
        double from_twist =
          angles::normalize_angle(camera_from_twist - arc_angle);
        double to_twist =
          angles::normalize_angle(camera_to_twist + pi - arc_angle);

        // OLD: Compute the distance between *origin* and *to*:
        //double distance_per_pixel = from_tag->distance_per_pixel;
        //double camera_distance =
        //  double__square_root(camera_dx * camera_dx + camera_dy * camera_dy);
        //double old_floor_distance = camera_distance * distance_per_pixel;
        //File__format(stderr, "floor_distance=%.2f old_floor_distance=%.2f\n",
        //  floor_distance, old_floor_distance);

        //File__format(stderr,
        //  "Map__arc_update: camera_from_twist=%.2f camera_to_twist=%.2f\n",
         //  camera_from_twist * r2d, camera_to_twist * r2d);
        //File__format(stderr,
        //  "Map__arc_update: arc_angle=%.2f from_twist=%.2f to_twist=%.2f\n",
        //  arc_angle * r2d, from_twist * r2d, to_twist * r2d);

        // Finally, upate *arc*:
        Arc__update(arc, from_twist, floor_distance, to_twist, goodness);
        map->changes_count += 1;
        map->is_changed = (bool)1;
        map->is_saved = (bool)0;

        // Let interested parties know that *arc* has been updated:
        Map__arc_announce(map, arc, image, sequence_number);

        changed = 1;
    }
    return changed;
}

/// @brief Returns -1, 0, 1 depending upon the sort order of *map1* and *map2*.
/// @param map1 is the first *Map* to compare.
/// @param map2 is the second *Map* to compare.
/// @returns sort order.
///
/// *Map__compare*() will compare *map1* to *map2* and return -1 if *map1*
/// sorts before *map2*, 0 if they are equal, and -1 if *map1* would sort
/// after *map2*.  Realistically, this routine is only used for testing
/// equality.

bool Map__equals(Map map1, Map map2) {
    // First make sure all of the *Tag*'s match up:
    unsigned int all_tags1_size = map1->all_tags.size();
    unsigned int all_tags2_size = map2->all_tags.size();
    if (all_tags1_size == all_tags2_size) {
        // Visit each *Tag*:
        for (unsigned int index = 0; index < all_tags1_size; index++) {
            Tag tag1 = map1->all_tags[index];
            Tag tag2 = map2->all_tags[index];
            if (!Tag__equal(tag1, tag2)) {
              return false;
            }
        }
    } else {
        return false;
    }

    // Second make sure all of the *Arc*'s match up:
    unsigned int all_arcs1_size = map1->all_arcs.size();
    unsigned int all_arcs2_size = map2->all_arcs.size();
    if (all_arcs1_size == all_arcs2_size) {
        // Visit each *Arc*:
        for (unsigned int index = 0; index < all_arcs1_size; index++) {
            Arc arc1 = map1->all_arcs[index];
            Arc arc2 = map2->all_arcs[index];
            if( !Arc__equal(arc1, arc2)) {
              return false;
            }
        }
    } else {
      return false;
    }
    return true;
}

/// @brief Returns a new *Map*.
/// @param file_path is the directory/folder that the map fileis stored in.
/// @param file_base is the base name of the map file.
/// @param announce_object is an opaque object that is passed into announce
///        routines.
/// @param arc_announce_routine is the arc callback routine.
/// @param tag_announce_routine is the tag callback routine.
/// @param tag_heights_file_name is the tag ceiling heights .xml file.
/// @param from is used for memory leak checking.
/// @returns a new *Map*.
///
/// *Map__create*() creates and returns an empty initialized *Map* object.

Map Map__create(String_Const file_path, String_Const file_base,
  void *announce_object, Fiducials_Arc_Announce_Routine arc_announce_routine,
  Fiducials_Tag_Announce_Routine tag_announce_routine,
  String_Const tag_heights_file_name, String_Const from) {
    // Create and fill in *map*:
    Map map = Memory__new(Map, from);
    map->arc_announce_routine = arc_announce_routine;
    map->announce_object = announce_object;
    map->changes_count = 0;
    map->file_base = file_base;
    map->file_path = file_path;
    map->is_changed = (bool)0;
    map->is_saved = (bool)1;
    map->image_log = (bool)0;
    map->tag_announce_routine = tag_announce_routine;
    map->temporary_arc = Arc__new("Map__new:Arc__New:temporary_arc");
    map->visit = 0;

    // Read in the contents of *map_heights_file_name* into *map*:
    Map__tag_heights_xml_read(map, tag_heights_file_name);

    // Restore *map* from "*map_path*/*map_base*{0,1}.xml".
    // We try to read "...1.xml" first, followed by "...0.xml":
    String full_map_file_name =
      String__format("%s/%s1.xml", file_path, file_base);
    File in_file = File__open(full_map_file_name, "r");
    if (in_file == (File)0) {
        // We failed to open "...1.xml"; now try "...0.xml":
        String__free(full_map_file_name);
        String full_map_file_name =
          String__format("%s/%s0.xml", file_path, file_base);
        in_file = File__open(full_map_file_name, "r");
        if (in_file != (File)0) {
            // We opened "...0.xml", read it in:
            Map__restore(map, in_file);
            File__close(in_file);
        }
    } else {
        // We opened "...1.xml", read it in:
        printf("Reading %s\n", full_map_file_name);
        Map__restore(map, in_file);
        File__close(in_file);
    }
    return map;
}

/// @brief Releases storage associated with *map*.
/// @param map to release storage for.
///
/// *Map__free*() will release the storage associaed with *map*.

void Map__free(Map map) {
    // Save the map:
    Map__save(map);

    // Release all the *Arc*'s:
    unsigned int arcs_size = map->all_arcs.size();
    for (unsigned int index = 0; index < arcs_size; index++) {
        Arc arc = map->all_arcs[index];
        Arc__free(arc);
    }
    Arc__free(map->temporary_arc);

    // Release all the *Tag*'s:
    unsigned int tags_size = map->all_tags.size();
    for (unsigned int index = 0; index < tags_size; index++) {
        Tag tag = map->all_tags[index];
        Tag__free(tag);
    }

    // Release all the *Tag_Height*'s:
    unsigned int tag_heights_size = map->tag_heights.size();
    for (unsigned int index = 0; index < tag_heights_size; index++) {
        Tag_Height tag_height = map->tag_heights[index];
        Tag_Height__free(tag_height);
    }

    Memory__free((Memory)map);
}

/// @brief Log image to disk if image logging is turned on.
/// @param map to use.
/// @param image to log.
///
/// *Map__image_log*() will log *image* to disk if image logging is turned on.

static int last_sequence_number = 0xffffffff;

void Map__image_log(Map map, CV_Image image, unsigned int sequence_number) {
    if (image != (CV_Image)0 && map->image_log &&
      sequence_number != last_sequence_number) {
        // Log the image here:
        String file_name = String__format("log%05d.tga", sequence_number);
        CV_Image__tga_write(image, file_name);
        last_sequence_number = sequence_number;
    }
}

/// @brief Restore the contents of *Map* from *in_file*.
/// @param map is the *Map* to restore into
/// @param in_file is the *File* to read from.
///
/// *Map__restore*() will read in an XML map file from *in_file* and
/// store it into *map*.

void Map__restore(Map map, File in_file) {
    // Read in Map XML tag '<Map Tags_Count="xx" Arcs_Count="xx">' :
    File__tag_match(in_file, "Map");
    unsigned int all_tags_size =
      (unsigned int)File__integer_attribute_read(in_file, "Tags_Count");
    unsigned int all_arcs_size =
      (unsigned int)File__integer_attribute_read(in_file, "Arcs_Count");
    File__string_match(in_file, ">\n");

    // Read in the *all_tags_size* *Tag* objects:
    for (unsigned int index = 0; index < all_tags_size; index++) {
        Tag tag = Tag__read(in_file, map);

        fprintf(stderr, "announce %d\n", tag->id);
        map->tag_announce_routine(map->announce_object,
        tag->id, tag->x, tag->y, tag->z, tag->twist,
        tag->diagonal, tag->world_diagonal/tag->diagonal,
        0, tag->hop_count);
    }

    // Read in the *all_arcs_size* *Arc* objects:
    for (unsigned int index = 0; index < all_arcs_size; index++) {
        Arc arc = Arc__read(in_file, map);
    }

    // Process the final Map XML tag "</MAP>":
    File__tag_match(in_file, "/Map");
    File__string_match(in_file, ">\n");

    // Do some final checks:
    assert (map->all_arcs.size() == all_arcs_size);
    assert (map->all_tags.size() == all_tags_size);
}

/// @brief Save *map* out to the file named *file_name*.
/// @param map to save out.
///
/// *Map__save*() will save *map* to the *file_name* file in XML format.

void Map__save(Map map) {
      File__format(stderr, "**********Map__save************\n");
      if (!map->is_saved) {
        String full_map_file_name =
          String__format("%s/%s1.xml", map->file_path, map->file_base);
        File out_file = File__open(full_map_file_name, "w");
        assert (out_file != (File)0);
        String__free(full_map_file_name);
        Map__write(map, out_file);
        File__close(out_file);
        map->is_saved = (bool)1;
    }
}

/// @brief Sort the contents of *map* to be in a consistent order.
/// @param map to reorder.
///
/// *Map__sort*() will reorder all of the tags and neihbgors in *map*
/// to be in a consitent order.

void Map__sort(Map map) {
    std::sort(map->all_tags.begin(), map->all_tags.end(), Tag__less);
    std::sort(map->all_arcs.begin(), map->all_arcs.end(), Arc__less);
}

/// @brief Writes *map* out to a file called *svg_base_name*.svg.
/// @param map is the *Map* to write out.
/// @param svg_base_name is the base name of the .svg file to write out.
/// @param locations is the list of locations that the robot path took.
///
/// *Map__svg_write*() will write out *map* out *svg_base_name*.svg.

void Map__svg_write(Map map, const String svg_base_name, 
    std::vector<Location*> &locations) {
    // Figure out how many *Arc*'s and *Tag*'s we have:
    unsigned int all_tags_size = map->all_tags.size();
    unsigned int all_arcs_size = map->all_arcs.size();

    // Compute the *bounding_box*:
    BoundingBox * bounding_box = new BoundingBox();
    for (unsigned int index = 0; index < all_tags_size; index++) {
        Tag tag = map->all_tags[index];
        Tag__bounding_box_update(tag, bounding_box);
    }

    // Open the Scalable Vector Graphics file:
    SVG svg = SVG__open(svg_base_name, 8.0, 10.5, 1.0, 1.0, "in");

    SVG__cartesian_scale(svg, 8.0, 10.5, bounding_box);

    // Draw the X/Y axes:
    String_Const color = "cyan";
    SVG__line(svg,
      bounding_box->min_x(), 0.0, bounding_box->max_x(), 0.0, color);
    SVG__line(svg,
      0.0, bounding_box->min_y(), 0.0, bounding_box->max_y(), color);

    // Output each *tag in *all_tags*:
    double world_diagonal = 0.1;
    for (unsigned int index = 0; index < all_tags_size; index++) {
        Tag tag = map->all_tags[index];
        world_diagonal = tag->world_diagonal;
        Tag__svg_write(tag, svg);
    }

    // Output each *tag in *all_tags*:
    for (unsigned int index = 0; index < all_arcs_size; index++) {
        Arc arc = map->all_arcs[index];
        Arc__svg_write(arc, svg);
        // publish rviz marker here

    }

    unsigned int locations_size = locations.size();
    double last_x = 0.0;
    double last_y = 0.0;
    for (unsigned int index = 0; index < locations_size; index++) {
        Location * location = locations[index];
        double x = location->x;
        double y = location->y;
        double bearing = location->bearing;
        //File__format(stderr, "Location[%d]: id:%d x:%f y:%f bearing:%f\n",
        //  index, location->id, x, y, bearing * 180 / 3.1415926);

        // Draw a triangle that shows the bearing:
        double k1 = world_diagonal / 2.0;
        double k2 = k1 / 2.0;
        double angle = 3.14159 * 0.75;
        double x0 = x + k1 * cos(bearing);
        double y0 = y + k1 * sin(bearing);
        double x1 = x + k2 * cos(bearing + angle);
        double y1 = y + k2 * sin(bearing + angle);
        double x2 = x + k2 * cos(bearing - angle);
        double y2 = y + k2 * sin(bearing - angle);
        SVG__line(svg, x0, y0, x1, y1, "black");
        SVG__line(svg, x1, y1, x2, y2, "black");
        SVG__line(svg, x2, y2, x0, y0, "black");

        // Draw a line that connects the centers of the triangles:
        if (index > 0) {
            SVG__line(svg, last_x, last_y, x, y, "purple");
        }
        last_x = x;
        last_y = y;
    }

    // Close *svg*:
    SVG__close(svg);

    delete bounding_box;
}

/// @brief Causes an arc announce callback routine to be called.
/// @param map is the parent *Map* object.
/// @param tag is the *Tag* object that has just been changed.
/// @param visible is True if the tag is in the current field of view.
/// @param image is the current image being processed.
/// @param sequence_number is the sequence number.
///
/// *Map__arc_announce*() will cause the arc announde call back routine
/// to be called for *arc*.

void Map__tag_announce(Map map,
  Tag tag, bool visible, CV_Image image, unsigned int sequence_number) {
    map->tag_announce_routine(map->announce_object,
      tag->id, tag->x, tag->y, tag->z, tag->twist,
      tag->diagonal, tag->world_diagonal/tag->diagonal,
      visible, tag->hop_count);
    if (visible) {
        //Map__image_log(map, image, sequence_number);
    }
}

/// @brief Returns the distance per pixel for *id*.
/// @param map is the *Map* object that contains the distance per pixel table.
/// @param id is the *Tag* identifier to look up.
/// @returns the distance per pixel for *Tag* id.
///
/// *Map__distance_per_pixel*() will return the distance per pixel for
/// *Tag* *id*.  The distance can be in any consistent distance (e.g.
/// (millimeters, centimeters, meters, kilometers, inches, feet, miles,
/// light seconds, etc.)

Tag_Height Map__tag_height_lookup(Map map, unsigned int id) {
    //double distance_per_pixel = 0.0;
    Tag_Height tag_height = (Tag_Height)0;
    unsigned int size = map->tag_heights.size();
    for (unsigned int index = 0; index < size; index++) {
        tag_height = map->tag_heights[index];
        if (tag_height->first_id <= id && id <= tag_height->last_id) {
            //distance_per_pixel = tag_height->distance_per_pixel;
          
            break;
        }
        tag_height = (Tag_Height)0;
    }
    return tag_height;
}

/// @brief Reads the tag heights .xml file.
/// @param map to to store tag heights into.
/// @param tag_heights_file_name is the file to read from.
///
/// *Map__tag_heights_xml_read*() will read the *tag_heights_file_name* .xml
/// file and the the tag heights into *map*.

void Map__tag_heights_xml_read(Map map, String_Const tag_heights_file_name) {
    // Open *tag_height_file_name* for reading:
    File xml_in_file = File__open(tag_heights_file_name, "r");
    if (xml_in_file == (File)0) {
        File__format(stderr, "Could not open '%s'\n", tag_heights_file_name);
        assert(0);
    }

    // Read in Map XML tag '<Map_Tag_Heights Count="xx">' :
    File__tag_match(xml_in_file, "Map_Tag_Heights");
    unsigned int count =
      (unsigned int)File__integer_attribute_read(xml_in_file, "Count");
    File__string_match(xml_in_file, ">\n");

    // Read in the *count* *Tag_Height* objects into *tag_heights*:
    for (unsigned int index = 0; index < count; index++) {
        Tag_Height tag_height = Tag_Height__xml_read(xml_in_file);
        map->tag_heights.push_back(tag_height);
    }

    // Process the final Map XML tag "</Map_Tag_Heights>":
    File__tag_match(xml_in_file, "/Map_Tag_Heights");
    File__string_match(xml_in_file, ">\n");

    // Close out *xml_in_file*:
    File__close(xml_in_file);

    // Sort *tag_heights*:
    std::sort(map->tag_heights.begin(), map->tag_heights.end(),
        Tag_Height__less);
}

/// @brief Return the *Tag* associated with *tag_id* from *map*.
/// @param map to use for lookup.
/// @param tag_id to lookup.
/// @returns *Tag* associated with *tag_id*.
///
/// *Map__tag_lookup*() will lookup and return the *Tag* associaed with
/// *tag_id* using *map.  If no previous instance of *tag_id* has been
/// encountered, a new *Tag* is created and add to the association in *map*.

Tag Map__tag_lookup(Map map, unsigned int tag_id) {
    if( map->tags_.count(tag_id) == 0 ) {
        Tag tag = Tag__create(tag_id, map);
        map->tags_[tag_id] = tag;
        map->all_tags.push_back(tag);
        map->changes_count += 1;
        map->is_changed = (bool)1;
        map->is_saved = (bool)0;
    }
    return map->tags_[tag_id];
}

/// @brief Writes *map* out to *out_file*.
/// @param map to write out.
/// @param out_file to write to.
///
/// *Map__write*() will write *map* to *out_file* in XML format.

void Map__write(Map map, File out_file) {
    // Figure out how many *Arc*'s and *Tag*'s we have:
    unsigned int all_tags_size = map->all_tags.size();
    unsigned int all_arcs_size = map->all_arcs.size();

    // Output <Map ...> tag:
    File__format(out_file, "<Map");
    File__format(out_file, " Tags_Count=\"%d\"", all_tags_size);
    File__format(out_file, " Arcs_Count=\"%d\"", all_arcs_size);
    File__format(out_file, ">\n");

    // Put the tags out in sorted order:
    Map__sort(map);

    // Output each *tag in *all_tags*:
    for (unsigned int index = 0; index < all_tags_size; index++) {
        Tag tag = map->all_tags[index];
        Tag__write(tag, out_file);
    }

    // Output each *tag in *all_tags*:
    for (unsigned int index = 0; index < all_arcs_size; index++) {
        Arc arc = map->all_arcs[index];
        Arc__write(arc, out_file);
    }

    // Output the closing </Map> tag:
    File__format(out_file, "</Map>\n");
}

/// @brief Updates the location of each *tag* in *map*.
/// @param map to update.
/// @param image is the current image.
/// @param sequence_number is the image sequence number.
///
/// *Map__update*() will update the location of all the *Tag*'s in *map*.

void Map__update(Map map, CV_Image image, unsigned int sequence_number) {
    if (map->is_changed) {
        // Increment *visit* to the next value to use for updating:
        unsigned int visit = map->visit + 1;
        map->visit = visit;

        // We want the tag with the lowest id number to be the origin.
        // Sort *tags* from lowest tag id to greatest:
        std::sort(map->all_tags.begin(), map->all_tags.end(), Tag__less);

        // The first tag in {tags} has the lowest id and is forced to be the
        // map origin:
        Tag origin_tag = map->all_tags[0];
        origin_tag->visit = visit;
        origin_tag->hop_count = 0;
        
        // The first step is to identify all of the *Arc*'s that make a
        // spanning tree of the *map* *Tags*'s.

        // Initializd *pending_arcs* with the *Arc*'s from *orgin_tag*:
        map->pending_arcs.insert(map->pending_arcs.end(),
            origin_tag->arcs_.begin(), origin_tag->arcs_.end());

        // We always want to keep *pending_arcs* sorted from longest to
        // shortest at the end.  *Arc__distance_compare*() sorts longest first:
        std::sort(map->pending_arcs.begin(), map->pending_arcs.end(),
            Arc__distance_less);

        // We keep iterating across *pending_arcs* until it goes empty.
        // since we keep it sorted from longest to shortest (and we always
        // look at the end), we are building a spanning tree using the shortest
        // possible *Arc*'s:
        while (map->pending_arcs.size() != 0) {
            // Pop the shortest *arc* off the end of *pending_arcs*:
            Arc arc = map->pending_arcs.back();
            map->pending_arcs.pop_back();

            // For debugging only:
            //File__format(stderr, "----------\n");
            //unsigned int size = List__size(pending_arcs);
            //for (unsigned int index = 0; index < size; index++) {
            //    Arc arc = (Arc)List__fetch(pending_arcs, index);
            //    File__format(stderr,
            //      "pending_arcs[%d]: Arc[%d,%d] dist=%f\n",
            //      index, arc->origin->id, arc->target->id, arc->distance);
            //}

            // If we already visited *arc*, just ignore it:
            if (arc->visit != visit) {
                // We have not visited this *arc* in this cycle, so now we
                // mark it as being *visit*'ed:
                arc->visit = visit;

                // Figure out if *origin* or *target* have been added to the
                // spanning tree yet:
                Tag from_tag = arc->from_tag;
                Tag to_tag = arc->to_tag;
                bool from_is_new = (bool)(from_tag->visit != visit);
                bool to_is_new = (bool)(to_tag->visit != visit);

                if (from_is_new || to_is_new) {
                    if (from_is_new) {
                        // Add *to* to spanning tree:
                        assert (!to_is_new);
                        from_tag->hop_count = to_tag->hop_count + 1;
                        map->pending_arcs.insert(map->pending_arcs.end(),
                            from_tag->arcs_.begin(), from_tag->arcs_.end());
                        from_tag->visit = visit;
                        Tag__update_via_arc(from_tag,
                          arc, image, sequence_number);
                    } else {
                        // Add *from* to spanning tree:
                        assert (!from_is_new);
                        to_tag->hop_count = from_tag->hop_count + 1;
                        map->pending_arcs.insert(map->pending_arcs.end(),
                            to_tag->arcs_.begin(), to_tag->arcs_.end());
                        to_tag->visit = visit;
                        Tag__update_via_arc(to_tag,
                          arc, image, sequence_number);
                    }

                    // Mark that *arc* is part of the spanning tree:
                    arc->in_tree = (bool)1;

                    // Resort *pending_arcs* to that the shortest distance
                    // sorts to the end:
                    std::sort(map->pending_arcs.begin(),
                        map->pending_arcs.end(), Arc__distance_less);
                } else {
                    // *arc* connects across two nodes of spanning tree:
                    arc->in_tree = (bool)0;
                }
            }
        }

        if (map->is_changed) {
            Map__save(map);
        }

        // Mark that *map* is fully updated:
        map->is_changed = (bool)0;
        map->is_saved = (bool)0;
    }
}

