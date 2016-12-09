#!/usr/bin/python

import os, sys

"""
Generate a PDF file containaing one or more fiducial marker for printing
"""

def checkCmd(cmd, package):
    rc = os.system("which inkscape > /dev/null")
    if rc != 0:
        print """This utility requires %s. It can be installed by typing:
    sudo apt install %s""" % (cmd, package)
        sys.exit(1)
     
if __name__ == "__main__":
    checkCmd("inkscape", "inkscape")
    checkCmd("pdfunite", "poppler-utils")
    if len(sys.argv) != 4:
        print "Usage: %s startId endId pdfFile" % sys.argv[0]
        sys.exit(1)
    outfile = sys.argv[3]
    tags = range(int(sys.argv[1]), int(sys.argv[2])+1)
    pdfs = map(lambda i: "tag%d.pdf" % i, tags)

    for i in tags:
        print " Tag %d\r" % i,
        sys.stdout.flush()
        os.system("rosrun fiducial_lib Tags %d" % i)
        os.system("inkscape --without-gui --export-pdf=tag%d.pdf tag%d.svg" % (i, i))
        os.remove("tag%d.svg" % i)
    print "Combining into %s" % outfile
    os.system("pdfunite %s %s" % (" ".join(pdfs), outfile))
    for f in pdfs:
        os.remove(f)
