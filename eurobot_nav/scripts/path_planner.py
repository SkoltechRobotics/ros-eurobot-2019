#!/usr/bin/env python
import visilibity as vg
import numpy as np
from polygon_conversions import list_from_polygon, list_from_polygon_array, polygon_list_from_numpy, get_map

class PathPlanning(object):
    def __init__(self):
        self.robot_radius = 0.0
        self.world = np.array([[self.robot_radius, self.robot_radius], [3. - self.robot_radius, self.robot_radius],
                  [3. - self.robot_radius, 2. - self.robot_radius], [2.55 + self.robot_radius, 2. - self.robot_radius],
                 [2.55 + self.robot_radius, 1.543 - self.robot_radius],
                               [0.45 - self.robot_radius, 1.543 - self.robot_radius],
                 [0.45 - self.robot_radius, 2. - self.robot_radius], [self.robot_radius, 2. - self.robot_radius]])
        self.chaos_zone_purple = np.array([[1. - self.robot_radius, 1 - self.robot_radius], [1. - self.robot_radius, 1. + self.robot_radius], [1. + self.robot_radius, 1. + self.robot_radius], [1. + self.robot_radius, 1. - self.robot_radius]])
        self.chaos_zone_yellow = np.array([[2. - self.robot_radius, 1 - self.robot_radius], [2. - self.robot_radius, 1. + self.robot_radius], [2. + self.robot_radius, 1. + self.robot_radius], [2. + self.robot_radius, 1. - self.robot_radius]])
        self.epsilon = 0.00001

    @staticmethod
    def points2vis_graph(points):
        points_list = []
        for point in points:
            print point
            points_list.append([vg.Point(x[0], x[1]) for x in point])
        g = vg.Visibility_Graph()
        polygon_list = []
        for i in points_list:
            print vg.Polygon(i).is_in_standard_form()
            polygon_list.append(vg.Polygon(i))
        print polygon_list
        g = vg.Environment(polygon_list)
        print ("ENV CREATED")
        return g

    @staticmethod
    def vg_path2points(path):
        path_ = []
        for i in path.path():
            path_.append([i.x(), i.y(), 0])
        return np.array(path_)

    @staticmethod
    def point2vg(point):
        return vg.Point(point[0], point[1])

    def create_path(self, start_point, goal_point, polygon):
        vg_start_point = self.point2vg(start_point)
        vg_goal_point = self.point2vg(goal_point)
        print ("CREATING ENV")
        print (polygon)
        polygons = [self.world.copy()]
        if polygon != []:
            polygons.append(np.array(polygon))
        print (polygons)
        visual_graph = self.points2vis_graph(polygons)
        print ("VS GRAPH")
        print (visual_graph.is_valid())
        vg_start_point.snap_to_boundary_of(visual_graph, self.epsilon)
        vg_start_point.snap_to_vertices_of(visual_graph, self.epsilon)
        print ("CREATE PATH")
        path = visual_graph.shortest_path(vg_start_point, vg_goal_point)
        print path
        points = self.vg_path2points(path)
        print points
        points[:-1, 2] = start_point[0]
        points[-1, 2] = goal_point[2]
        return points
