#!/usr/bin/env python
from shapely.geometry import Polygon, MultiPolygon, LineString
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


def get_map(world, polygons):
    world_map = polygon_list_from_numpy([world])[0]
    polygons = polygon_list_from_numpy(polygons)
    obstacle = []
    for i in polygons:
        world_line = LineString(world.exterior.coords)
        if i.intersects(world_line):
            world_map = world_map.symmetric_difference(i).difference(i)
        else:
            obstacle.append(i)
    return world_map, obstacle
