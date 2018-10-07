#!/usr/bin/python

import os, sys
import em

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
    f.write(em.expand("""<svg width="@(paper_width)mm" height="@(paper_height)mm"
 version="1.1"
 xmlns:xlink="http://www.w3.org/1999/xlink"
 xmlns="http://www.w3.org/2000/svg">

  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width + fid_len)/2 - 4)mm" y="@((paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>

  <image x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="marker@(id).png" />


  @{cut = max(fid_len/10 * 1.4, 10)}

  @{corner_x = (paper_width - fid_len)/2 - cut}
  @{corner_y = (paper_height - fid_len)/2 - cut}
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x + 2)mm" y2="@(corner_y)mm" style="stroke:black"/>
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x)mm" y2="@(corner_y + 2)mm" style="stroke:black"/>
  
  @{corner_x = (paper_width + fid_len)/2 + cut}
  @{corner_y = (paper_height - fid_len)/2 - cut}
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x - 2)mm" y2="@(corner_y)mm" style="stroke:black"/>
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x)mm" y2="@(corner_y + 2)mm" style="stroke:black"/>

  <text x="@(paper_width/2)mm" y="@(corner_y - 1)mm" text-anchor="middle" style="font-family:ariel; font-size:8;">
      This line should be exactly @(fid_len/10)cm long. 
  </text>
  <line x1="@(paper_width/2 - fid_len/2)mm" y1="@(corner_y)mm" x2="@(paper_width/2 + fid_len/2)mm" y2="@(corner_y)mm" style="stroke:black"/>
  <line x1="@(corner_x)mm" y1="@(paper_height/2 - fid_len/2)mm" x2="@(corner_x)mm" y2="@(paper_height/2 + fid_len/2)mm" style="stroke:black"/>

  @{corner_x = (paper_width - fid_len)/2 - cut}
  @{corner_y = (paper_height + fid_len)/2 + cut}
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x + 2)mm" y2="@(corner_y)mm" style="stroke:black"/>
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x)mm" y2="@(corner_y - 2)mm" style="stroke:black"/>

  <line x1="@(corner_x)mm" y1="@(paper_height/2 - fid_len/2)mm" x2="@(corner_x)mm" y2="@(paper_height/2 + fid_len/2)mm" style="stroke:black"/>
  <line x1="@(paper_width/2 - fid_len/2)mm" y1="@(corner_y)mm" x2="@(paper_width/2 + fid_len/2)mm" y2="@(corner_y)mm" style="stroke:black"/>

  @{corner_x = (paper_width + fid_len)/2 + cut}
  @{corner_y = (paper_height + fid_len)/2 + cut}
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x - 2)mm" y2="@(corner_y)mm" style="stroke:black"/>
  <line x1="@(corner_x)mm" y1="@(corner_y)mm" x2="@(corner_x)mm" y2="@(corner_y - 2)mm" style="stroke:black"/>

  <text x="@(paper_width/2)mm" y="@((paper_height + fid_len)/2 + 30)mm" text-anchor="middle" style="font-family:ariel; font-size:24;">@(id) D@(dicno)</text>

</svg>
""", {"id": id, "dicno": dicno, "paper_width": 215.9, "paper_height": 279.4, "fid_len": 140.0}))
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
        os.system("inkscape --without-gui --export-area-page --export-pdf=marker%d.pdf marker%d.svg" % (i, i))
        os.remove("marker%d.svg" % i)
        os.remove("marker%d.png" % i)
    print "Combining into %s" % outfile
    os.system("pdfunite %s %s" % (" ".join(pdfs), outfile))
    for f in pdfs:
        os.remove(f)

    print '\033[91m' + """After printing, please make sure thatthe long lines around the marker are
EXACTLY 14.0 cm long. This is required for accurate position estimation.""" + '\033[0m'

