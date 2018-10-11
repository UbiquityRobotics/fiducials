#!/usr/bin/python

"""
Fit a plane to the fiducials in the map as a quantitative test of the
map quality.  Assumes they are all on a ceiling
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import math
import sys
import argparse
from standard_fit import standard_fit, distance, projection

def closest_angle(old, new):
    angle = new
    dif = angle - old
    if dif > 180:
       dif -= 360
    elif dif < -180:
       dif += 360
    if abs(dif) > 90:
        angle += 180
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle

# parse args
map_file = os.environ["HOME"] + "/.ros/slam/map.txt"
parser = argparse.ArgumentParser()
parser.add_argument("--map_file", help="Name of map file ", default=map_file, type=str)
parser.add_argument("--adjust", help="Adjust points to fit plane", action="store_true")
args = parser.parse_args()

# read data from map
ids = []
points = []
roll = []
pitch = []
other = []

adjust = args.adjust

with open(args.map_file) as file:
   lines = file.readlines()
   for line in lines:
       parts = line.split()
       ids.append(int(parts[0]))
       points.append([float(parts[1]), float(parts[2]), float(parts[3])])
       roll.append(float(parts[4]))
       pitch.append(float(parts[5]))
       other.append(" ".join(parts[6:]))

# plot raw data
plt.figure()

points = np.array(points)
ax = plt.subplot(111, projection='3d')
xs = points[:,0]
ys = points[:,1]
zs = points[:,2]

ax.scatter(xs, ys, zs, color='b')

C, N = standard_fit(points)
print("Plane normal: %s" % N)

# plot points projected onto plane
fixed_points = projection(points, C, N)
xs = fixed_points[:,0]
ys = fixed_points[:,1]
zs = fixed_points[:,2]
ax.scatter(xs, ys, zs, color='r')

errors = distance(points, C, N)
residual = np.linalg.norm(errors)

slopex = math.atan2(N[0], N[2]) * 180.0 / math.pi
slopey = math.atan2(N[1], N[2]) * 180.0 / math.pi
print("slope: %f deg in X %f deg in Y" % (slopex, slopey))

print("residual: %f" % residual)

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
zlim = ax.get_zlim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
D = -C.dot(N)
Z = (-N[0] * X - N[1] * Y - D) / N[2]
ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()


if adjust:
    print("Saving adjusted map")
    os.rename(map_file, map_file + ".bak")
    with open(map_file, 'w') as file:
        for i in range(len(errors)):
            file.write("%d %f %f %f %f %f %s\n" % (ids[i], xs[i], ys[i], zs[i],
                                                   closest_angle(roll[i], slopex),
                                                   closest_angle(pitch[i], slopey),
                                                   other[i])) 
