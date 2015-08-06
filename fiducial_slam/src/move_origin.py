#!/usr/bin/python

"""
Move origin of fiducial co-ordinate system
"""

import numpy, sys
from fiducial_slam import FiducialSlam

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print "Usage: %s x y z" % sys.argv[0]
        sys.exit(1)
    offset = numpy.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    node = FiducialSlam()
    fids = node.fiducials.keys()
    for fid in fids:
        f = node.fiducials[fid]
        f.position += offset
    node.close()
