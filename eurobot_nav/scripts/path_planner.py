#!/usr/bin/env python
import numpy as np
import pyvisgraph as vg


class PathPlanning(object):
    def __init__(self):
        self.robot_radius = 0.15
        self.world_border_bottom = np.array([[0.1, 0.], [0.1, self.robot_radius], [2.9, self.robot_radius], [2.9, 0.]])
        self.world_border_left = np.array([[0., 0.], [0., 2.], [self.robot_radius, 2.], [self.robot_radius, 0.]])
        self.world_border_top = np.array([[0.1, 2], [3., 2], [3., 2. - self.robot_radius],
                                          [0.1, 2. - self.robot_radius]])
        self.world_border_right = np.array([[3., 0.], [3., 2.], [3. - self.robot_radius, 2.],
                                            [3. - self.robot_radius, 0.]])
        self.scales = np.array([[0.45 - self.robot_radius, 2], [0.45 - self.robot_radius, 1.543 - self.robot_radius],
                                [2.55 + self.robot_radius, 1.543 - self.robot_radius],
                                [2.55 + self.robot_radius, 2], [0.45 - self.robot_radius, 2]])

    @staticmethod
    def points2vis_graph(points):
        points_list = []
        g = vg.VisGraph()

        for point in points:
            points_list.append([vg.Point(x[0], x[1]) for x in point])
        g.build(points_list)
        return g

    @staticmethod
    def vg2points(vg_points):
        return np.array([(x.x, x.y, 0.) for x in vg_points])

    @staticmethod
    def point2vg(point):
        return vg.Point(point[0], point[1])

    def create_path(self, start_point, goal_point, polygons):
        vg_start_point = self.point2vg(start_point)
        vg_goal_point = self.point2vg(goal_point)
        polygons.append(self.world_border_bottom)
        polygons.append(self.world_border_top)
        polygons.append(self.world_border_right)
        polygons.append(self.world_border_left)
        polygons.append(self.scales)
        print polygons
        visual_graph = self.points2vis_graph(polygons)
        path = visual_graph.shortest_path(vg_start_point, vg_goal_point)
        points = self.vg2points(path)
        points[:-1, 2] = start_point[0]
        points[-1, 2] = goal_point[2]
        return points
