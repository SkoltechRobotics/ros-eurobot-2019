#!/usr/bin/env python
# coding: utf-8
import numpy as np
from core_functions import calculate_distance
from core_functions import batch_calculate_distance
from core_functions import wrap_angle
from core_functions import wrap_back
from core_functions import cvt_local2global


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
    print color_key
    if all(color_key == np.array([1, 0, 0])):
        color_val = "REDIUM"
    elif all(color_key == np.array([0, 1, 0])):
        color_val = "GREENIUM"
    elif all(color_key == np.array([0, 0, 1])):
        color_val = "BLUNIUM"
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


# def compare_to_update_or_ignore(self, new_obs_of_pucks):
#
#     """
#     Input: [(id, x, y), ...]
#     Output: format: [(id, x, y), ...]
#
#     Old
#     [(1, x1, y1),
#      (2, x2, y2)
#      (3, x3, y3)]
#
#      New obs
#      [(2, x2', y2'),
#       (3, x3', y3')]
#
#     In a new list first puck may be absent for at least three reasons:
#     - it was collected by our robot
#     - it is not visible (but it's still there) either because of robot in the line of view or light conditions
#     - it was collected by enemy robot (and it's not there anymore)
#
#     """
#
#     assert new_obs_of_pucks.shape[1] == 3
#
#     known_ids = self.known_chaos_pucks[:, 0]
#     known_pucks = self.known_chaos_pucks
#     for puck in new_obs_of_pucks:
#         puck_id = puck[0]
#         if puck_id in known_ids:
#             ind = known_ids.index(puck_id)  # correct? FIXME
#             x_new, y_new = puck[1], puck[2]
#             x_old, y_old = known_pucks[1], known_pucks[2]
#             if np.sqrt((x_new - x_old) ** 2 + (y_new - y_old) ** 2) > self.coords_threshold:
#                 self.known_chaos_pucks[ind][1:] = puck[1:]
#         else:
#             # wtf happened? blink from sun? wasn't recognised at first time?
#             self.known_chaos_pucks = np.stack((self.known_chaos_pucks, puck), axis=0)
#             # self.known_coords_of_pucks.append(puck)
