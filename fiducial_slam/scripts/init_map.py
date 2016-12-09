#!/usr/bin/python

"""
Initialize a map for use by fiducial_slam by adding one ceiling fiducial at the orgin,
rotates so that will be in the co-ordiante system of the floor.
"""

import sys, os

if __name__ == "__main__":
    argc = len(sys.argv)
    if argc < 2 or argc > 3:
        print "Usage:  %s fiducial_id [map_file]" % sys.argv[0]
        sys.exit(1)

    fid = int(sys.argv[1])
    if argc == 3:
        map_file = sys.argv[2]
    else: 
        map_file = os.environ['HOME'] + "/.ros/slam/map.txt"

    dir=os.path.dirname(map_file)
    if dir and not os.path.exists(dir):
        os.makedirs(dir)
    if os.path.exists(map_file):
        print "File %s already exists, remove or rename it first" % map_file
        sys.exit(1)

    file = open(map_file, "w")
    file.write("%d 0.0 0.0 0.0 180.0 0 180.0 0 1\n" % fid)
    file.close()
    print "Map file %s created with fiducial %d" % (map_file, fid)
