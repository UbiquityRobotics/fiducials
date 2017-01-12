#!/usr/bin/python

"""
Move origin of fiducial co-ordinate system
"""

import numpy, sys, os
from fiducial_slam.map import Map

if __name__ == "__main__":
    argc = len(sys.argv)
    if argc != 4 and argc != 5:
        print "Usage: %s x y z [file]" % sys.argv[0]
        sys.exit(1)
    offset = numpy.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    if argc == 5:
        filename = sys.argv[4]
    else:
        filename = "~/.ros/slam/map.txt"
    filename = os.path.expanduser(filename)
    map = Map(filename)
    fids = map.keys()
    for fid in fids:
        f = map[fid]
        f.position += offset
    map.save()
