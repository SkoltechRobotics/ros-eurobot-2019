#!/usr/bin/env python
# coding: utf-8
import numpy as np
from core_functions import calculate_distance
from core_functions import batch_calculate_distance
from core_functions import wrap_angle
from core_functions import wrap_back
from core_functions import cvt_local2global
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def get_color(puck):
    """
    red (1, 0, 0)
    green (0, 1, 0)
    blue (0, 0, 1)
    :param puck: (x, y, id, 0, 0, 1)
    :return:
    """
    # pucks_colors = {
    #     (1, 0, 0): "REDIUM",
    #     (0, 1, 0): "GREENIUM",
    #     (0, 0, 1): "BLUNIUM"
    # }
    # color_val = pucks_colors.get(color_key)

    color_val = None
    color_key = puck[3:]
    # print color_key
    if all(color_key == np.array([1, 0, 0])):
        color_val = "REDIUM"
    elif all(color_key == np.array([0, 1, 0])):
        color_val = "GREENIUM"
    elif all(color_key == np.array([0, 0, 1])):
        color_val = "BLUNIUM"
    elif all(color_key == np.array([0, 0, 0])):
        color_val = "UNDEFINED"
    return color_val


def calc_inner_angles(coords):
    is_line = polygon_area(coords)  # returns area
    if is_line == 0:
        raise AttributeError('pucks lie on a straigt line, so no inner angles')

    else:
        hull_indexes = calculate_convex_hull(coords)
        hull = np.array([coords[i] for i in hull_indexes])  # using returned indexes to extract hull's coords

        next_ = np.roll(hull, -1, axis=0)
        prev_ = np.roll(hull, 1, axis=0)

        prev_dist = batch_calculate_distance(hull[:, :2], prev_[:, :2])
        next_dist = batch_calculate_distance(hull[:, :2], next_[:, :2])

        prev_a = np.arctan2(prev_dist[:, 1], prev_dist[:, 0])
        next_a = np.arctan2(next_dist[:, 1], next_dist[:, 0])
        angles = wrap_angle(prev_a - next_a)

        coords_angles = list(zip(hull, angles))
        coords_angles.sort(key=lambda t: t[1])
        coords_sorted = np.array([v[0] for v in coords_angles])
        angles_sorted = np.array([v[1] for v in coords_angles])

        return coords_sorted, angles_sorted


def polygon_area(hull):
    x = np.array([h[0] for h in hull])
    y = np.array([h[1] for h in hull])  # because slicing doesn't work for some reason

    x_ = x - np.mean(x)  # coordinate shift
    y_ = y - np.mean(y)
    correction = x_[-1] * y_[0] - y_[-1] * x_[0]
    main_area = np.dot(x_[:-1], y_[1:]) - np.dot(y_[:-1], x_[1:])
    return 0.5*np.abs(main_area + correction)


def rotate(a, b, c):
    return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])


def calculate_convex_hull(coords):
    """
    jarvis march
    input: [(x0, y0), (x1, y1), ...]
    :return: convex hull, INDEXES of coordinates in input list, not coords themselves
    """
    n = len(coords)
    p = range(n)
    # start point
    for i in range(1, n):
        if coords[p[i]][0] < coords[p[0]][0]:
            p[i], p[0] = p[0], p[i]
    h = [p[0]]
    del p[0]
    p.append(h[0])
    while True:
        right = 0
        for i in range(1, len(p)):
            if rotate(coords[h[-1]], coords[p[right]], coords[p[i]]) < 0:
                right = i
        if p[right] == h[0]:
            break
        else:
            h.append(p[right])
            del p[right]

    # hull = [coords[i] for i in h]  # this was used to extract coords from input list using calculated indexes
    return h


def find_indexes_of_outer_points_on_line(coords):
    """
    finds indexes of minimum and maximum coords, for example there are 4 points in a raw and we'll get outers [1, 2]
    checkk if line is horizontal (look at gamma of any point provided)
    if line of 4 points is horizontal than just take two points with max and min Y-coord,
    else choose points with max and min X-coord
    :param coords: [[x1, y1], [x2, y2], ...]
    :return: indexes [1, 2, 0, 3]
    """

    indexes = []
    is_line_horizontal = [True if coords[0][0] == coords[-1][0] else False]
    if is_line_horizontal:
        srt = np.argsort(coords[:, 1], axis=0)
    else:
        srt = np.argsort(coords[:, 0], axis=0)  # argsort returns indexes

    srt = list(srt)
    indexes.append(srt.pop(0))
    indexes.append(srt.pop(-1))
    while len(srt) > 0:
        indexes.append(srt.pop(0))

    return indexes


def calculate_pucks_configuration(robot_coords, known_chaos_pucks, critical_angle):

    known_chaos_pucks = sort_wrt_robot(robot_coords, known_chaos_pucks)  # [(0.95, 1.1, 3, 0, 0, 1), ...]

    if len(known_chaos_pucks) >= 3:
        is_hull_safe_to_approach, coords_sorted_by_angle = sort_by_inner_angle_and_check_if_safe(robot_coords, known_chaos_pucks, critical_angle)

        if is_hull_safe_to_approach:  # only sharp angles
            print("hull is SAFE to approach, sorted wrt robot")

        if not is_hull_safe_to_approach:
            known_chaos_pucks = coords_sorted_by_angle  # calc vert-angle, sort by angle, return vertices (sorted)
            print("hull is not safe to approach, sorted by angle")

    # when we finally sorted them, chec if one of them is blue. If so, roll it
    if len(known_chaos_pucks) > 1 and all(known_chaos_pucks[0][3:6] == [0, 0, 1]):
        known_chaos_pucks = np.roll(known_chaos_pucks, -1, axis=0)  # so blue becomes last one to collect
        print("blue rolled")
    return known_chaos_pucks


def calculate_landings(robot_coords, coordinates, approach_vec, scale_factor, approach_dist):
    coords = coordinates[:, :2]
    if len(coords) == 1:
        landings = calculate_closest_landing_to_point(robot_coords, coords, approach_vec)  # Should be [(x, y, theta), ...]
    else:
        landings = unleash_power_of_geometry(coords, scale_factor, approach_dist)
    return landings


def unleash_power_of_geometry(coords, scale_factor, approach_dist):
    """
    Calculates offset a hull,
    get line between orig hull and offset
    search intersection point between orbit around centre of each puck and line
    There are many safe landing coords, not just on outer bissectrisa, but for now only the intersection

    Solve system of two equations:
    x**2 + y**2 = r**2, where r is an approch distance to puck for robot
    y = tg(gamma) * x, using calculated slope to get equation of the line
    Orbit inersects line in two points, so we get two landings

    for two pucks there are 4 candidates, for three - 6 candidates
    we calculate to keep only those that lie between puck and it's outer offset

    :return: list of landing coordinates for all angles [[x_l, y_l, gamma], ...]
    """

    landings = []

    mc = np.mean(coords, axis=0)
    offset = list(coords)  # a miserable attempt to copy a list
    offset -= mc  # Normalize the polygon by subtracting the center values from every point.
    offset *= scale_factor
    offset += mc

    for orig, ofs in zip(coords, offset):
        dist, _ = calculate_distance(ofs, orig)
        gamma = np.arctan2(dist[1], dist[0])  # and here we calculate slope of outer bis
        gamma = wrap_back(gamma)
        x_candidate1 = np.sqrt(approach_dist**2 / (1 + np.tan(gamma)**2))
        x_candidate2 = - np.sqrt(approach_dist**2 / (1 + np.tan(gamma)**2))

        y_candidate1 = np.tan(gamma) * x_candidate1
        y_candidate2 = np.tan(gamma) * x_candidate2

        candidate1 = np.array([x_candidate1 + orig[0], y_candidate1 + orig[1], gamma])
        candidate2 = np.array([x_candidate2 + orig[0], y_candidate2 + orig[1], gamma])

        xmin = min(orig[0], ofs[0])
        xmax = max(orig[0], ofs[0])
        ymin = min(orig[1], ofs[1])
        ymax = max(orig[1], ofs[1])

        if all([candidate1[0] >= xmin, candidate1[0] <= xmax, candidate1[1] >= ymin, candidate1[1] <= ymax]):
            landings.append(candidate1)
        elif all([candidate2[0] >= xmin, candidate2[0] <= xmax, candidate2[1] >= ymin, candidate2[1] <= ymax]):
            landings.append(candidate2)
        else:
            print ("ORIG=", orig)
            print ("OFS=", ofs)
            print ("candidate1=", candidate1)
            print ("candidate2=", candidate2)
            print ("SOMETHING WRONG AT LANDING CALCULATIONS")

    landings = np.array(landings)

    return landings


def sort_wrt_robot(robot_coords, coords):
    """
    To make operation more quick there is no need to choose the sharpest angle,
    instead better choose the closest one from angles that are marked as safe.
    TODO add id to every landing so it corresponds to color of puck
    :param coords: now [[x1, y1], ...]. Should be [[0, 0.95, 1.1, 0, 0, 1], ...]
    :robot_coords
    :return: format: [[3, 3, 1.57], [1, 1, 3.14], [2, 2, 4, 71], [0, 0, 6.28]]
    """
    coords = np.array(coords)

    if len(coords) == 1:
        return coords
    else:

        distances_from_robot_to_landings = []
        for coordinate in coords:
            dist, _ = calculate_distance(robot_coords, coordinate)  # return deltaX and deltaY coords
            dist_norm = np.linalg.norm(dist)
            distances_from_robot_to_landings.append(dist_norm)
        a = zip(coords, distances_from_robot_to_landings)
        a.sort(key=lambda t: t[1])
        sorted_coords = np.array([v[0] for v in a])
        return sorted_coords


def calculate_closest_landing_to_point(robot_coords, point, approach_vec):
    """
    calculates closest landing to point wrt to current robot position
    :param point: [x,y]
    :return: [xl,yl,thetal]
    """
    point = point[0][:2]  # added [:2] because we don't give a shit about color

    dist, _ = calculate_distance(robot_coords, point)  # return deltaX and deltaY coords
    gamma = np.arctan2(dist[1], dist[0])
    point = np.hstack((point, gamma))
    landing = cvt_local2global(approach_vec, point)
    landing = landing[np.newaxis, :]
    return landing


def sort_by_inner_angle_and_check_if_safe(robot_coords, coords, critical_angle):

    """
    here we first find inner angles
    than sort them with sharpest first
    than compares them with trheshold and returns True False
    :param coords: [(0.95, 1.1, 3, 0, 0, 1), ...]
    :return: # Should be [(0.95, 1.1, 3, 0, 0, 1), ...]
    """
    is_hull_safe_to_approach = False
    try:
        coords_sorted, angles_sorted = calc_inner_angles(coords)
        coords_sorted[:2] = sort_wrt_robot(robot_coords, coords_sorted[:2])  # trick with sorting two sharpest
        if any(angles_sorted > critical_angle):
            is_hull_safe_to_approach = False
        elif all(angles_sorted < critical_angle):
            is_hull_safe_to_approach = True

    except AttributeError:
        print("pucks lie in straigt line, so no angles")
        indexes_outer = find_indexes_of_outer_points_on_line(coords)
        coords_sorted = np.array([coords[i] for i in indexes_outer])

    return is_hull_safe_to_approach, coords_sorted


def initial_parse_pucks(observation, pcc, ycc, chaos_radius, pca, yca):
    """

    :param observation: [[x, y, id, r, g, b], [x, y, id, r, g, b]...]
    :param pcc: purple_chaos_center (x, y)
    :param ycc: yellow_chaos_center (x, y)
    :param chaos_radius: const
    :param pca: purple_cells_area
    :param yca: yellow_cells_area
    :return: [[x, y, id, r, g, b], [x, y, id, r, g, b]...] format for each of chaoses and for  pucks_rgb
            (red cell puck, green cell puck, blue cell puck)
    """

    purple_chaos_pucks = []
    yellow_chaos_pucks = []
    purple_pucks_rgb = []
    yellow_pucks_rgb = []
    other_pucks = []
    offset = 0.03

    purple_chaos_center_point = Point(pcc[0], pcc[1])
    yellow_chaos_center_point = Point(ycc[0], ycc[1])

    # create circle buffer from the points
    purple_chaos_buffer = purple_chaos_center_point.buffer(chaos_radius + offset)
    yellow_chaos_buffer = yellow_chaos_center_point.buffer(chaos_radius + offset)

    purple_cell_buffer = Polygon([pca[0], pca[1], pca[2], pca[3]])
    yellow_cell_buffer = Polygon([yca[0], yca[1], yca[2], yca[3]])

    # checkk if the other point lies within
    # within and contains is the same thing
    for puck in observation:
        current_puck = Point(puck[0], puck[1])
        if current_puck.within(purple_chaos_buffer):
            purple_chaos_pucks.append(puck)
        elif current_puck.within(yellow_chaos_buffer):
            yellow_chaos_pucks.append(puck)
        elif purple_cell_buffer.contains(current_puck):
            purple_pucks_rgb.append(puck)
        elif yellow_cell_buffer.contains(current_puck):
            yellow_pucks_rgb.append(puck)
        else:
            other_pucks.append(puck)

    purple_chaos_pucks = np.array(purple_chaos_pucks)
    yellow_chaos_pucks = np.array(yellow_chaos_pucks)
    purple_pucks_rgb.sort(key=lambda t: t[1])
    yellow_pucks_rgb.sort(key=lambda t: t[1])
    purple_pucks_rgb = np.array(purple_pucks_rgb)
    yellow_pucks_rgb = np.array(yellow_pucks_rgb)

    return purple_chaos_pucks, yellow_chaos_pucks, purple_pucks_rgb, yellow_pucks_rgb


def yolo_parse_pucks(observation, area):

    lost_pucks = []

    if len(observation) > 0:
        search_area_buffer = Polygon([area[0], area[1], area[2], area[3]])

        for puck in observation:
            current_puck = Point(puck[0], puck[1])
            if current_puck.within(search_area_buffer):
                lost_pucks.append(puck)
    else:
        print "no pucks on the field"

    lost_pucks = np.array(lost_pucks)
    return lost_pucks


"""
def parse_by_color(new_obs, known):

    colors = ['BLUNIUM', 'GREENIUM', 'REDIUM']
    known = known.tolist()
    colors_in_new_obs = []
    colors_in_known = []
    list_of_pucks = []
    # FIXME: remove repetitive code!

    for puck in new_obs:
        puck_color = get_color(puck)
        colors_in_new_obs.append(puck_color)

    for puck in known:
        puck_color = get_color(puck)
        colors_in_known.append(puck_color)

    if colors_in_new_obs.count("BLUNIUM") == 1:

        ind = colors_in_new_obs.index("BLUNIUM")
        colors_in_new_obs.remove("BLUNIUM")
        list_of_pucks.append(new_obs.pop(ind))
        print "UPDATED BLUNIUM COORDINATE ==========================="
        try:
            ind_old = colors_in_known.index("BLUNIUM")
            colors_in_known.remove("BLUNIUM")
            del known[ind_old]
        except ValueError as Error:
            print Error

    if colors_in_new_obs.count("GREENIUM") == 1:

        ind = colors_in_new_obs.index("GREENIUM")
        colors_in_new_obs.remove("GREENIUM")
        list_of_pucks.append(new_obs.pop(ind))
        print "UPDATED GREENIUM COORDINATE ==========================="
        try:
            ind_old = colors_in_known.index("GREENIUM")
            colors_in_known.remove("GREENIUM")
            del known[ind_old]
        except ValueError as Error:
            print Error

    if colors_in_new_obs.count('REDIUM') == 2:

        ind = colors_in_new_obs.index('REDIUM')
        colors_in_new_obs.remove('REDIUM')
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index('REDIUM')
            colors_in_known.remove('REDIUM')
            # print "old after deleting", colors_in_known
            del known[ind_old]
        except ValueError as Error:
            print Error

        ind = colors_in_new_obs.index('REDIUM')
        colors_in_new_obs.remove('REDIUM')
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index('REDIUM')
            colors_in_known.remove('REDIUM')
            del known[ind_old]
        except ValueError as Error:
            print Error

        print "UPDATED TWO REDIUM COORDINATE ==========================="

        list_of_pucks.extend(known)

    elif colors_in_new_obs.count("REDIUM") < 2:
        # get coords of red in known and append them
        list_of_pucks.extend(known)
    return list_of_pucks
"""

    # def compare_to_update_or_ignore(self, new_observation):
    #     """
    #     Updates coordinates of the puck if camera sees it and is certain that it was moved
    #
    #     In a new list first puck may be absent for at least three reasons:
    #     - it was collected by our robot
    #     - it is not visible (but it's still there) either because of robot in the line of view or light conditions
    #     - it was collected by opponent robot (and it's not there anymore)
    #
    #     Updating procedure for my chaos:
    #     1) Parsing pucks to chaos collections
    #     if we see every puck left in our chaos than simply update all
    #     if we see less than calculated, than update only green and blue.
    #     If we see only one red, it's hard to say which particularly it is, so update red pucks only if both are observed
    #
    #     :param new_observation: [(x, y, id, r, g, b), ...]
    #     :return: [(x, y, id, r, g, b), ...]
    #     """
    #
    #     my_chaos_new = self.my_chaos_pucks.get().copy()
    #     opp_chaos_new = self.opponent_chaos_pucks.get().copy()
    #
    #     observed_my_chaos_collection = []
    #     observed_opponent_chaos_collection = []
    #     other_pucks = []
    #     ref_colors = ['BLUNIUM', 'GREENIUM', 'REDIUM', 'REDIUM']
    #     colors_of_my_observed_chaos = []
    #     colors_of_opp_observed = []
    #     colors_of_my_collected_chaos = []
    #     colors_of_opp_chaos_collected_by_me = []
    #
    #     my_chaos_area = Polygon([self.my_chaos_area[0],
    #                              self.my_chaos_area[1],
    #                              self.my_chaos_area[2],
    #                              self.my_chaos_area[3]])
    #
    #     opponent_chaos_area = Polygon([self.opponent_chaos_area[0],
    #                                    self.opponent_chaos_area[1],
    #                                    self.opponent_chaos_area[2],
    #                                    self.opponent_chaos_area[3]])
    #
    #     # TODO change to list comprehension
    #     # for puck in self.my_collected_chaos.get():
    #     #     colors_of_my_collected_chaos.append(get_color(puck))
    #
    #     # TODO change to list comprehension
    #     # for puck in self.opp_chaos_collected_me.get():
    #     #     colors_of_opp_chaos_collected_by_me.append(get_color(puck))
    #
    #     for puck in new_observation:
    #         unknown_puck = Point(puck[0], puck[1])
    #         if unknown_puck.within(my_chaos_area):
    #             observed_my_chaos_collection.append(puck)
    #             colors_of_my_observed_chaos.append(get_color(puck))
    #         elif unknown_puck.within(opponent_chaos_area):
    #             observed_opponent_chaos_collection.append(puck)
    #             colors_of_opp_observed.append(get_color(puck))
    #         else:
    #             other_pucks.append(puck)
    #
    #     # comparison_our = colors_of_my_observed_chaos[:]
    #     # comparison_our.extend(colors_of_my_collected_chaos)
    #
    #     # comparison_opponent = colors_of_opp_observed[:]
    #     # comparison_opponent.extend(colors_of_opp_chaos_collected_by_me)
    #
    #     print "MY_chaos_observed", sorted(colors_of_my_observed_chaos)
    #     print "OPPONENT_chaos_observed", sorted(colors_of_opp_observed)
    #
    #     if len(observed_my_chaos_collection) == (4 - len(self.my_collected_chaos.get())):
    #     #if sorted(comparison_our) == ref_colors:
    #         print "my chaos, equal"
    #         my_chaos_new = observed_my_chaos_collection
    #     # else:
    #     #     my_chaos_new = self.parse_by_color(observed_my_chaos_collection, my_chaos_new)
    #     #     print "my chaos, need to parse!"
    #
    #     if len(observed_opponent_chaos_collection) == (4 - len(self.opp_chaos_collected_me.get())):
    #     #if sorted(comparison_our) == ref_colors:
    #         print "opp_chaos, equal"
    #         opp_chaos_new = observed_opponent_chaos_collection
    #
    #     # if sorted(comparison_opponent) == ref_colors:
    #     #     print "opp_chaos, equal"
    #     #     opp_chaos_new = observed_opponent_chaos_collection
    #     # else:
    #     #     opp_chaos_new = self.parse_by_color(observed_opponent_chaos_collection, opp_chaos_new)
    #     #     print "opp_chaos, need to parse!"
    #
    #     my_chaos_new = np.array(my_chaos_new)
    #     opp_chaos_new = np.array(opp_chaos_new)
    #
    #     return my_chaos_new, opp_chaos_new