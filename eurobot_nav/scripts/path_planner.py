#!/usr/bin/env python
import visilibity as vg
import numpy as np
from polygon_conversions import list_from_polygon, list_from_polygon_array, polygon_list_from_numpy, get_map


class PathPlanning(object):
    def __init__(self, robot_radius):
        self.robot_radius = robot_radius
#         self.world = np.array([[self.robot_radius, self.robot_radius], [3. - self.robot_radius, self.robot_radius],
#                   [3. - self.robot_radius, 2. - self.robot_radius], [2.55 + self.robot_radius, 2. - self.robot_radius],
#                  [2.55 + self.robot_radius, 1.543 - self.robot_radius],
#                                [0.45 - self.robot_radius, 1.543 - self.robot_radius],
#                  [0.45 - self.robot_radius, 2. - self.robot_radius], [self.robot_radius, 2. - self.robot_radius]])
        self.chaos_zone_purple = np.array([[1. - self.robot_radius, 1 - self.robot_radius], [1. - self.robot_radius, 1. + self.robot_radius], [1. + self.robot_radius, 1. + self.robot_radius], [1. + self.robot_radius, 1. - self.robot_radius]])
        self.chaos_zone_yellow = np.array([[2. - self.robot_radius, 1 - self.robot_radius], [2. - self.robot_radius, 1. + self.robot_radius], [2. + self.robot_radius, 1. + self.robot_radius], [2. + self.robot_radius, 1. - self.robot_radius]])
        self.epsilon = 0.0001
        self.is_flipped = False
        self.world = np.array([[self.robot_radius, self.robot_radius], [0.5 - self.robot_radius, self.robot_radius],
                               [0.5 - self.robot_radius, 0.07 + self.robot_radius],
                               [2.5 + self.robot_radius, 0.07 + self.robot_radius],
                               [2.5 + self.robot_radius, self.robot_radius],
                               [3. - self.robot_radius, self.robot_radius],
                               [3. - self.robot_radius, 2. - self.robot_radius],
                               [2.55 + self.robot_radius, 2. - self.robot_radius],
                               [2.55 + self.robot_radius/2, 1.543 - self.robot_radius/2],
                               [1.6 - self.robot_radius/2, 1.543 - self.robot_radius/2],
                               [1.6 - self.robot_radius/2, 1.343 - self.robot_radius/2],
                               [1.4 - self.robot_radius/2, 1.343 - self.robot_radius/2],
                               [1.4 - self.robot_radius/2, 1.543 - self.robot_radius/2],
                               [0.45 - self.robot_radius, 1.543 - self.robot_radius],
                               [0.45 - self.robot_radius, 2. - self.robot_radius],
                               [self.robot_radius, 2. - self.robot_radius],
                               [self.robot_radius, self.robot_radius]])

    @staticmethod
    def points2vis_graph(points):
        points_list = []
        for point in points:
            print point
            points_list.append([vg.Point(x[0], x[1]) for x in point])
        polygon_list = []
        for ind, polygon in enumerate(points_list):
            print polygon
            print "INPUT POLYGON"
            input_polygon = vg.Polygon(polygon)
            print input_polygon
            area = input_polygon.area()
            print area
            if area <= 0:
                if ind == 0:
                    polygon = polygon[::-1]
                    input_polygon = vg.Polygon(polygon)
            elif ind != 0:
                polygon = polygon[::-1]
                input_polygon = vg.Polygon(polygon)
            # input_polygon = vg.Polygon(polygon)
            polygon_list.append(input_polygon)
        print ("CREATING ENV")
        g = vg.Environment(polygon_list)
        print ("CREATED")
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
        p = []
        world, obstacle = get_map(self.world.copy(), polygon)
        p.append(world)
        if len(obstacle) != 0:
            p.append(polygon)
        visual_graph = self.points2vis_graph(p)
        print visual_graph.is_valid()
        vg_start_point = self.point2vg(start_point)
        vg_goal_point = self.point2vg(goal_point)
        vg_start_point.snap_to_boundary_of(visual_graph, self.epsilon)
        vg_start_point.snap_to_vertices_of(visual_graph, self.epsilon)
        shortest_path = visual_graph.shortest_path(vg_start_point, vg_goal_point, self.epsilon)
        return self.vg_path2points(shortest_path)
