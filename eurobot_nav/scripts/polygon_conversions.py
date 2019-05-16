#!/usr/bin/env python
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import cascaded_union
import numpy as np


def list_from_polygon(polygon):
    coords = polygon.exterior.coords.xy
    x = coords[0]
    y = coords[1]
    polygon = []
    for i in range(len(x) - 1):
        polygon.append([x[i], y[i]])
    return polygon


def list_from_polygon_array(polygons):
    polygon = []
    for i in polygons:
        print i
        print np.array(list_from_polygon(i))
        polygon.append(list_from_polygon(i))
    return polygon


def polygon_list_from_numpy(polygon_list):
    polygons = []

    for i in polygon_list:
        polygons.append([((x[0], x[1])) for x in i])
        print polygons
        polygons_shapely = []
    for polygon in polygons:
        polygons_shapely.append(Polygon(polygon))
    return polygons_shapely
