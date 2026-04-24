
# Simultaneous Localization and Mapping Using Fiducial Markers
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

## Overview

### Ceiling fiducials

This package implements a system that uses ceiling mounted
fiducial markers (think QR Codes) to allow a robot to identify
its location and orientation.  It does this by constructing
a map of the ceiling fiducials.  The position of one fiducial
needs to be specified, then a map of the fiducials is built 
up from observing pairs of markers in the same image. 
Once the map has been constructed, the robot can identify
its location by locating itself relative to one or more 
ceiling fiducials.

### Floor fiducials

You can put the fiducials also on the floor and direct the camera downwards. This is used in EZ-Map and Conveyorbot as additional source of localization data.

## TODO

- Extend the STag marker configuration so each tag can optionally store a known world-frame pose (`x`, `y`, `z`) for future global robot pose estimation.
