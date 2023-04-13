# cython: language_level=3
import numpy as np

def faster_for_no_ti(points):
    plen = len(points)
    points_np = np.zeros( plen, \
    dtype={ 
        "names": ( "x", "y", "z", "intensity", "t"), 
        "formats": ( "f4", "f4", "f4", "f4", "u4")} )

    for i in range(plen):
        p = points[i]
        points_np[i] = (p.x, p.y, p.z, 0, 0)

    return points_np

def faster_for_no_t(points):
    plen = len(points)
    points_np = np.zeros( plen, \
    dtype={ 
        "names": ( "x", "y", "z", "intensity", "t"), 
        "formats": ( "f4", "f4", "f4", "f4", "u4")} )

    for i in range(plen):
        p = points[i]
        points_np[i] = (p.x, p.y, p.z, p.intensity, 0)

    return points_np

def faster_for(points):
    plen = len(points)
    points_np = np.zeros( plen, \
    dtype={ 
        "names": ( "x", "y", "z", "intensity", "t"), 
        "formats": ( "f4", "f4", "f4", "f4", "u4")} )

    for i in range(plen):
        p = points[i]
        points_np[i] = (p.x, p.y, p.z, p.intensity, p.timestamp)


    return points_np