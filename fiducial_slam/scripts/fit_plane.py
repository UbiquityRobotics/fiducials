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

# read data from map
ids = []
xs = []
ys = []
zs = []

with open(os.environ["HOME"] + "/.ros/slam/map.txt") as file:
   lines = file.readlines()
   for line in lines:
       parts = line.split()
       ids.append(int(parts[0]))
       xs.append(float(parts[1]))
       ys.append(float(parts[2]))
       zs.append(float(parts[3]))

# plot raw data
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.scatter(xs, ys, zs, color='b')

# do fit
tmp_A = []
tmp_b = []
for i in range(len(xs)):
    tmp_A.append([xs[i], ys[i], 1])
    tmp_b.append(zs[i])
b = np.matrix(tmp_b).T
A = np.matrix(tmp_A)
fit = (A.T * A).I * A.T * b
errors = b - A * fit
residual = np.linalg.norm(errors)

print("Plane: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
slopex = math.atan(fit[0]) * 180.0 / math.pi
slopey = math.atan(fit[1]) * 180.0 / math.pi
print("slope: %f deg in X %f deg in Y" % (slopex, slopey))

print("errors:")
for i in range(len(errors)):
   print("%4d  %f" % (ids[i], errors[i]))

print("residual: %f" % residual)

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

