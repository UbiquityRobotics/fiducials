#!/usr/bin/python

from fiducial_slam import updateLinear

u1 = 2
u2 = 2
sigma1 = 0.1
sigma2 = 0.1

var1 = sigma1**2.0
var2 = sigma2**2.0
 
print updateLinear(u1, var1, u2, var2)
