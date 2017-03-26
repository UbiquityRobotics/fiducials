
import os, math
import numpy

from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler

"""
Create directory if not already there
"""
def mkdirnotex(filename):
    dir=os.path.dirname(filename)
    print "Directory", dir
    if dir and not os.path.exists(dir):
        os.makedirs(dir)

"""
Radians to degrees
"""
def rad2deg(rad):
    return rad / math.pi * 180.0

"""
Degrees to radians
"""
def deg2rad(deg):
    return float(deg) * math.pi / 180.0


"""
Weighted average of linear quantities
"""
def updateLinear(mean1, sigma1, mean2, sigma2):
    var1 = sigma1**2.0
    var2 = sigma2**2.0
    newMean = (mean1 * var2 + mean2 * var1) / (var1 + var2)
    #newVar = 1.0 / (1.0/var1 + 1.0/var2)
    # =((2*PI())^0.5)*C3*D3*EXP((((((C2-E2)^2))/(2*C3^2))+(((D2-E2)^2)/(2*(D3^2)))))
    d1 = dist = numpy.linalg.norm(mean1 - newMean)
    d2 = dist = numpy.linalg.norm(mean2 - newMean)
    """
    newVar = ((2.0*math.pi)**0.5) * var1 * var2 * math.exp(((((mean1-newMean)**2.0) / (2.0*var1)) +
        (((mean2-newMean)**2.0) / (2.0*var2))))
    """
    newVar = ((2.0*math.pi)**0.5) * var1 * var2 * math.exp((((d1**2.0) / (2.0*var1)) +
        ((d2**2.0) / (2.0*var2))))
    return [newMean, newVar]


"""
Weighted average of quaternions
"""
def updateAngular(quat1, var1, quat2, var2):
    newMean = quaternion_slerp(quat1, quat2, var1/(var1+var2))
    newVar = 1.0 / (1.0/var1 + 1.0/var2)
    return [newMean, newVar]

