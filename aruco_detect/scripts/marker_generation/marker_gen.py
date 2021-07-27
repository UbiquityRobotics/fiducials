"""
Internal functions for generating a fiducial marker PDF.

This has to be in a separate module from the main entry point in order to work with joblib,
which we use to generate markers in parallel.
"""

import os, sys, argparse
import subprocess
import importlib.machinery

# Hack if a user has 'em' instead of 'empy' installed with pip
# If the module em doesn't have the expand function use the full path
em = None
for path in sys.path:
    filename = os.path.join(path, 'em.py')
    if os.path.exists(filename):
         em_loader = importlib.machinery.SourceFileLoader('em', filename)
         em = em_loader.load_module()
         if "expand" in dir(em):
             break
# For-else: else is called if loop doesn't break 
else:
    print("ERROR: could not find module em, please sudo apt install python3-empy")
    exit(2)

import cv2
import cv2.aruco as aruco
import cairosvg

def genSvg(id, dicno, paper_size):
    return em.expand("""<svg width="@(paper_width)mm" height="@(paper_height)mm"
 version="1.1"
 xmlns:xlink="http://www.w3.org/1999/xlink"
 xmlns="http://www.w3.org/2000/svg">

  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
  <rect x="@((paper_width + fid_len)/2 - 4)mm" y="@((paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>

  <image x="@((paper_width - fid_len)/2)mm" y="@((paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="/tmp/marker@(id).png" />


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
""", {"id": id, "dicno": dicno, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": 140.0})

def genMarker(i, dicno, paper_size):
    print(" Marker %d\r" % i)
    sys.stdout.flush()
    aruco_dict = aruco.Dictionary_get(dicno)
    img = aruco.drawMarker(aruco_dict, i, int(2000))
    cv2.imwrite("/tmp/marker%d.png" % i, img)
    svg = genSvg(i, dicno, paper_size)
    cairosvg.svg2pdf(bytestring=svg, write_to='/tmp/marker%d.pdf' % i)
    # Old slower method using subprocess for SVG to PDF conversion
    # cairo = subprocess.Popen(('cairosvg', '-f', 'pdf', '-o', '/tmp/marker%d.pdf' % i, '/dev/stdin'), stdin=subprocess.PIPE)
    # cairo.communicate(input=bytes(svg, 'utf-8'))
    os.remove("/tmp/marker%d.png" % i)
