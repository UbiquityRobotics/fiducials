#!/usr/bin/env python

# This little program will extract some images from a bag file
# and write them out in .pnm format.  The usage is:
#
#    bag_images_extract.py bag_file_name [images_directory]

import rosbag
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

# Perform command line argument processing first:
arguments = sys.argv
del arguments[0]
arguments_size = len(arguments)
if arguments_size == 0:
    print("Usage: bag_images_extract.py bag_file_name [images_directory]")
else:

    # Default directory is "."
    directory = "."
    if arguments_size >= 2:
	directory = arguments[1]

    # Open the bag file:
    bag_file_name = arguments[0]
    bag = rosbag.Bag(bag_file_name)
    bridge = CvBridge()

    # Sweep through messages that match *topic_name*:
    topic_name = "/fiducials_localization/interesting_images"
    index = 0
    previous_time = None
    for topic, msg, t in bag.read_messages(topic_name):
	# Extract *cv_image* in RGB8 mode:
	index += 1
	try:
	    cv_image = bridge.imgmsg_to_cv(msg, desired_encoding="rgb8")
	except CvBridgeError, e:
	    print e

	# Save the image
	cv.SaveImage("{0}/Image{1:03d}.pnm".format(directory, index), cv_image)

	# Print out time changes:
	if previous_time == None:
	    previous_time = t
	dt = t - previous_time
	previous_time = t
	print("[{0}]:{1}".format(index, dt))

