#!/usr/bin/python3

"""
Generate a PDF file containaing one or more fiducial markers for printing
"""

import os, sys, argparse
import subprocess
from marker_generation import genMarker

def checkCmd(cmd, package):
    rc = os.system("which %s > /dev/null" % cmd)
    if rc != 0:
        print("""This utility requires %s. It can be installed by typing:
    sudo apt install %s""" % (cmd, package))
        sys.exit(1)
     
if __name__ == "__main__":
    checkCmd("pdfunite", "poppler-utils")
    checkCmd("cairosvg", "cairosvg python3-cairosvg")


    parser = argparse.ArgumentParser(description='Generate Aruco Markers.')
    parser.add_argument('startId', type=int,
                        help='start of marker range to generate')
    parser.add_argument('endId', type=int,
                        help='end of marker range to generate')
    parser.add_argument('pdfFile', type=str,
                        help='file to store markers in')
    parser.add_argument('dictionary', type=int, default='7', nargs='?',
                        help='dictionary to generate from')
    parser.add_argument('--paper-size', dest='paper_size', action='store',
                        default='letter', help='paper size to use (letter or a4)')
    args = parser.parse_args()

    outfile = args.pdfFile
    dicno = args.dictionary

    markers = range(args.startId, args.endId + 1)
    pdfs = map(lambda i: "/tmp/marker%d.pdf" % i, markers)

    if args.paper_size == 'letter':
        paper_size  = (215.9, 279.4)
    elif args.paper_size == 'a4':
        paper_size  = (210, 297)

    try:
        # For a parallel version
        from joblib import Parallel, delayed
        Parallel(n_jobs=-1)(delayed(genMarker)(i, dicno, paper_size) for i in markers)
    except ImportError:
        # Fallback to serial version
        for i in markers:
            genMarker(i, dicno, paper_size)

    print("Combining into %s" % outfile)
    os.system("pdfunite %s %s" % (" ".join(pdfs), outfile))
    for f in pdfs:
        os.remove(f)

    print('\033[91m' + """After printing, please make sure that the long lines around the marker are 
EXACTLY 14.0cm long. This is required for accurate position estimation.""" + '\033[0m')

