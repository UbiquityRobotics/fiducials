# stag_detect

## Overview

This package consist of files for generating and launching STag marker related software.

This node finds STag markers in images stream and estimates 3d transforms from the camera to the fiducials.
It is based on the [`STag`](https://github.com/usrl-uofsc/stag_ros), which documents also how well they perform according to ArUco.
We modified STag markers detection node for our needs and will not work with the codebase from original repository.

#### Library HD

You can generate STag markers from any library you prefer. 
Make sure you set the corresponding `libraryHD` parameter in launch file.
We performed most of the test with library HD11, which suited our needs.

## Packs

Generally each pack should correspond to a standalone route, but you can generate yourself packs according to your preference.
Take note that each marker has a unique id, while a number is unique only to the pack.
