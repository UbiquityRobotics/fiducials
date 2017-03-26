#!/usr/bin/python

from fiducial_slam import updateLinear

u1 = 2
u2 = 2
sigma1 = 0.1
sigma2 = 0.1

print updateLinear(u1, sigma1, u2, sigma2)
