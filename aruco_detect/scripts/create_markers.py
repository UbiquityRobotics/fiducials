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
     
def genSvg(file, id, dicno):
    f = open(file, "w")
    f.write("""<svg width="208.0mm" height="240.0mm"
 version="1.1"
 xmlns:xlink="http://www.w3.org/1999/xlink"
 xmlns="http://www.w3.org/2000/svg">

  <line x1="5.0mm" y1="5.0mm" x2="7.0mm" y2="5.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="5.0mm" x2="197.0mm" y2="5.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="21.0mm" x2="7.0mm" y2="21.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="21.0mm" x2="5.0mm" y2="23.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="21.0mm" x2="195.0mm" y2="21.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="21.0mm" x2="197.0mm" y2="23.0mm" style="stroke:black"/>
  
  <image x="31.0mm" y="47.0mm" width="140.0mm" height="140.0mm" xlink:href="marker%d.png" />

  <rect x="31.0mm" y="47.0mm" width="140.0mm" height="4.0mm" style="stroke:black; fill:black"/>
  <rect x="31.0mm" y="183.0mm" width="140.0mm" height="4.0mm" style="stroke:black; fill:black"/>
  <rect x="31.0mm" y="47.0mm" width="4.0mm" height="140.0mm" style="stroke:black; fill:black"/>
  <rect x="167.0mm" y="47.0mm" width="4.0mm" height="140.0mm" style="stroke:black; fill:black"/>

  <line x1="5.0mm" y1="213.0mm" x2="7.0mm" y2="213.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="213.0mm" x2="5.0mm" y2="211.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="213.0mm" x2="197.0mm" y2="213.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="213.0mm" x2="197.0mm" y2="211.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="229.0mm" x2="7.0mm" y2="229.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="229.0mm" x2="197.0mm" y2="229.0mm" style="stroke:black"/>

  <text x="90.0mm" y="220.0mm" style="font-family:ariel; font-size:24">%d D%d</text>

</svg>
""" % (id, id, dicno))
    f.close()

if __name__ == "__main__":
    checkCmd("inkscape", "inkscape")
    checkCmd("pdfunite", "poppler-utils")

    dicno = 7
    argc = len(sys.argv)
    if argc != 4 and argc != 5:
        print "Usage: %s startId endId pdfFile [dictionary]" % sys.argv[0]
        sys.exit(1)
    outfile = sys.argv[3]
    if argc == 5:
        dicno = int(sys.argv[4])
    markers = range(int(sys.argv[1]), int(sys.argv[2])+1)
    pdfs = map(lambda i: "marker%d.pdf" % i, markers)

    for i in markers:
        print " Marker %d\r" % i,
        sys.stdout.flush()
        os.system("rosrun aruco_detect create_marker --id=%d --ms=2000 --d=%d marker%d.png" % (i, dicno, i))
        genSvg("marker%d.svg" % i, i, 7)
        os.system("inkscape --without-gui --export-pdf=marker%d.pdf marker%d.svg" % (i, i))
        os.remove("marker%d.svg" % i)
        os.remove("marker%d.png" % i)
    print "Combining into %s" % outfile
    os.system("pdfunite %s %s" % (" ".join(pdfs), outfile))
    for f in pdfs:
        os.remove(f)
