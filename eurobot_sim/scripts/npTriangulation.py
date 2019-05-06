from core_functions import *
import numpy as np
import scipy.optimize


def find_position_triangulation(beacons, landmarks, init_point):
    dist_from_beacon = np.linalg.norm(beacons[np.newaxis, :, :] -
                                      (cvt_local2global(landmarks, init_point))[:, np.newaxis, :], axis=2)
    ind_closest_beacon = np.argmin(dist_from_beacon, axis=1)
    closest_beacon = beacons[ind_closest_beacon]

    def fun(X):
        points = cvt_local2global(landmarks, X)
        r = np.sum((closest_beacon - points)**2, axis=1)**0.5
        return r

    res = scipy.optimize.least_squares(fun, init_point)
    return np.array(res.x)
