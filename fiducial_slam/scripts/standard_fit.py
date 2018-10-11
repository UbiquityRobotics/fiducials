#!/usr/bin/env nosetests -vs
# coding=utf-8
"""
Linear algebra standard fitting module

(C) 2013 hashnote.net, Alisue
"""
__author__  = 'Alisue (lambdalisue@hashnote.net)'
__version__ = '0.1.0'
__date__    = '2013-10-28'
__all__ = ['standard_fit', 'projection', 'distance', 'function']

import numpy as np

def standard_fit(X):
    """
    Find (n - 1) dimensional standard (e.g. line in 2 dimension, plane in 3
    dimension, hyperplane in n dimension) via solving Singular Value
    Decomposition.

    The idea was explained in the following references

    - http://www.caves.org/section/commelect/DUSI/openmag/pdf/SphereFitting.pdf
    - http://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
    - http://www.ime.unicamp.br/~marianar/MI602/material%20extra/svd-regression-analysis.pdf
    - http://www.ling.ohio-state.edu/~kbaker/pubs/Singular_Value_Decomposition_Tutorial.pdf

    Example:
        >>> XY = [[0, 1], [3, 3]]
        >>> XY = np.array(XY)
        >>> C, N = standard_fit(XY)
        >>> C
        array([ 1.5,  2. ])
        >>> N
        array([-0.5547002 ,  0.83205029])

    Args:
        X: n x m dimensional matrix which n indicate the number of the dimension
            and m indicate the number of points

    Returns:
        [C, N] where C is a centroid vector and N is a normal vector
    """
    # Find the average of points (centroid) along the columns
    C = np.average(X, axis=0)
    # Create CX vector (centroid to point) matrix
    CX = X - C
    # Singular value decomposition
    U, S, V = np.linalg.svd(CX)
    # The last row of V matrix indicate the eigenvectors of
    # smallest eigenvalues (singular values).
    N = V[-1]
    return C, N

def projection(x, C, N):
    """
    Create orthogonal projection matrix of x on the plane

    Args:
        x: n x m dimensional matrix
        C: n dimensional vector whicn indicate the centroid of the standard
        N: n dimensional vector which indicate the normal vector of the standard

    Returns:
        n x m dimensional matrix which indicate the orthogonal projection points
        on the plane
    """
    rows, cols = x.shape
    NN = np.tile(N, (rows, 1))
    D = distance(x, C, N)
    DD = np.tile(D, (cols, 1)).T
    return x - DD * NN

def distance(x, C, N):
    """
    Calculate an orthogonal distance between the points and the standard

    Args:
        x: n x m dimensional matrix
        C: n dimensional vector whicn indicate the centroid of the standard
        N: n dimensional vector which indicate the normal vector of the standard

    Returns:
        m dimensional vector which indicate the orthogonal disntace. the value
        will be negative if the points beside opposite side of the normal vector
    """
    return np.dot(x-C, N)

def function(x, C, N):
    """
    Calculate an orthogonal projection of the points on the standard

    Args:
        x: (n-1) x m dimensional matrix
        C: n dimensional vector whicn indicate the centroid of the standard
        N: n dimensional vector which indicate the normal vector of the standard

    Returns:
        m dimensional vector which indicate the last attribute value of
        orthogonal projection
    """
    Ck = C[0:-1]    # centroid for known parameters
    Nk = N[0:-1]    # normal for known parmeters
    Cu = C[-1]      # centroid for unknown parameter
    Nu = N[-1]      # normal for unknown parameter
    return np.dot(x-Ck, Nk) * -1.0 / Nu + Cu

#===============================================================================
#
# Unittest
#
#===============================================================================
if __name__ == '__main__':
    import doctest; doctest.testmod()
