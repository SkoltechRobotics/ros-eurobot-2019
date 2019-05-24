#!/usr/bin/env python

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import threading


class BTVariable(object):
    def __init__(self, default_data=None):
        self.data = default_data
        self.mutex = threading.Lock()

    def set(self, data):
        with self.mutex:
            self.data = data

    def get(self):
        with self.mutex:
            data = self.data
        return data


def compare_to_update_or_ignore(new_observation, my_chaos_pucks, my_collected_chaos, opponent_chaos_pucks, my_chaos_area, opponent_chaos_area):
    """
    Updates coordinates of the puck if camera sees it and is certain that it was moved

    In a new list first puck may be absent for at least three reasons:
    - it was collected by our robot
    - it is not visible (but it's still there) either because of robot in the line of view or light conditions
    - it was collected by opponent robot (and it's not there anymore)

    Updating procedure for my chaos:
    1) Parsing pucks to chaos collections
    if we see every puck left in our chaos than simply update all
    if we see less than calculated, than update only green and blue.
    If we see only one red, it's hard to say which particularly it is, so update red pucks only if both are observed

    :param new_observation: [(x, y, id, r, g, b), ...]
    :return: [(x, y, id, r, g, b), ...]
    """

    my_chaos_new = my_chaos_pucks.get().copy()
    opp_chaos_new = opponent_chaos_pucks.get().copy()

    observed_my_chaos_collection = []
    observed_opponent_chaos_collection = []
    other_pucks = []
    ref_colors = ['BLUNIUM', 'GREENIUM', 'REDIUM', 'REDIUM']
    colors_of_my_observed_chaos = []
    colors_of_opp_observed = []
    colors_of_my_collected_chaos = []

    my_chaos_area = Polygon([my_chaos_area[0],
                             my_chaos_area[1],
                             my_chaos_area[2],
                             my_chaos_area[3]])

    opponent_chaos_area = Polygon([opponent_chaos_area[0],
                                   opponent_chaos_area[1],
                                   opponent_chaos_area[2],
                                   opponent_chaos_area[3]])

    for puck in my_collected_chaos.get():
        colors_of_my_collected_chaos.append(get_color(puck))
        print colors_of_my_collected_chaos

    for puck in new_observation:
        unknown_puck = Point(puck[0], puck[1])
        if unknown_puck.within(my_chaos_area):
            observed_my_chaos_collection.append(puck)
            colors_of_my_observed_chaos.append(get_color(puck))
        elif unknown_puck.within(opponent_chaos_area):
            observed_opponent_chaos_collection.append(puck)
            colors_of_opp_observed.append(get_color(puck))
        else:
            other_pucks.append(puck)

    comparison = colors_of_my_observed_chaos[:]

    comparison.extend(colors_of_my_collected_chaos)
    print "colors_of_my_observed_chaos", sorted(colors_of_my_observed_chaos)

    if sorted(colors_of_my_observed_chaos) == ref_colors:
        print "equal"
        my_chaos_new = observed_my_chaos_collection
    else:
        my_chaos_new = parse_by_color(observed_my_chaos_collection, my_chaos_new)
        print "need to parse!"
    # if len(observed_opponent_chaos_collection) == 4:  # TODO or if we started collecting it?
    #     opp_chaos_new = observed_opponent_chaos_collection
    # else:
    #     opp_chaos_new = parse_by_color(observed_opponent_chaos_collection, opp_chaos_new)

    my_chaos_new = np.array(my_chaos_new)
    opp_chaos_new = np.array(opp_chaos_new)

    return my_chaos_new, opp_chaos_new


def parse_by_color(new_obs, known):
    known = known.tolist()
    colors_in_new_obs = []
    colors_in_known = []
    list_of_pucks = []

    for puck in new_obs:
        puck_color = get_color(puck)
        colors_in_new_obs.append(puck_color)
    print "new", colors_in_new_obs

    for puck in known:
        puck_color = get_color(puck)
        colors_in_known.append(puck_color)
    print "old", colors_in_known

    if colors_in_new_obs.count("BLUNIUM") == 1:
        print "Found: BLUNIUM in new. Remove it"
        ind = colors_in_new_obs.index("BLUNIUM")
        colors_in_new_obs.remove("BLUNIUM")
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index("BLUNIUM")
            colors_in_known.remove("BLUNIUM")
            del known[ind_old]
            print "KNOWN", known
        except ValueError as Error:
            print Error
            # print "Inside BLUNIUM: NO pucks of this color seen at previous step"

    print " "
    print "colors in new - empty?", colors_in_new_obs
    print "old", colors_in_known
    if colors_in_new_obs.count("GREENIUM") == 1:
        print "Found: GREENIUM in new. Remove it"
        ind = colors_in_new_obs.index("GREENIUM")
        colors_in_new_obs.remove("GREENIUM")
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index("GREENIUM")
            colors_in_known.remove("GREENIUM")
            print "KNOWN", known
            del known[ind_old]
        except ValueError as Error:
            print Error

    print " "
    print "colors in new", colors_in_new_obs
    print "old", colors_in_known
    print "known before deleting two REDIUMS"
    print known
    if colors_in_new_obs.count('REDIUM') == 2:
        print "TWO REDIUMS, add them!"
        ind = colors_in_new_obs.index('REDIUM')
        colors_in_new_obs.remove('REDIUM')
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index('REDIUM')
            colors_in_known.remove('REDIUM')
            print "old after deleting", colors_in_known
            del known[ind_old]
        except ValueError:
            print "Inside 2-redium: NO pucks of this color seen at previous step"

        ind = colors_in_new_obs.index('REDIUM')
        colors_in_new_obs.remove('REDIUM')
        list_of_pucks.append(new_obs.pop(ind))
        try:
            ind_old = colors_in_known.index('REDIUM')
            colors_in_known.remove('REDIUM')
            del known[ind_old]
        except ValueError:
            print "Inside 2-redium: NO pucks of this color seen at previous step"

        print "known"
        print known
        list_of_pucks.extend(known)

    elif colors_in_new_obs.count("REDIUM") < 2:
        # get coords of red in known and append them
        list_of_pucks.extend(known)
    return list_of_pucks


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
    return color_val


new_observation_pucks = np.array([[0.95, 1.05, 1, 1, 0, 0],
                                    [2.05, 1.05, 4, 1, 0, 0],
                                    [0.5, 0.75, 10, 1, 0, 0],
                                    [1.02, 1.1, 2, 0, 1, 0],
                                    [1, 1, 3, 0, 0, 1],
                                    [2, 1, 5, 0, 0, 1],
                                    #[1.05, 1.05, 6, 1, 0, 0],
                                    [0.5, 0.45, 9, 1, 0, 0],
                                    [1.95, 1.05, 7, 1, 0, 0],
                                    [2, 1.1, 8, 0, 1, 0],
                                    [0.5, 1.05, 11, 1, 0, 0]
                                  ])

# my_chaos_pucks = bt.BTVariable(np.array([
#     [0.95, 1.05, 1, 1, 0, 0],
#     [1.05, 1.05, 6, 1, 0, 0],
#     [1, 1.1, 2, 0, 1, 0],
#     [1, 1, 3, 0, 0, 1]
# ]))

my_chaos_pucks = BTVariable(np.array([
    # [0.9, 1.01, 1, 1, 0, 0],
    # [1.0, 1.01, 6, 1, 0, 0],
    # [1, 1.11, 2, 0, 1, 0],
    # [1, 1.01, 3, 0, 0, 1]
]))


my_collected_chaos = BTVariable(np.array([]))

opponent_chaos_pucks = BTVariable(np.array([
    [2.05, 1.05, 4, 1, 0, 0],
    [2, 1, 5, 0, 0, 1],
    [1.95, 1.05, 7, 1, 0, 0],
    [2, 1.1, 8, 0, 1, 0]
]))

my_chaos_area = [[0.7, 0.6], [1.3, 0.6], [1.3, 1.25], [0.7, 1.25]]
opponent_chaos_area = [[2.3, 0.6], [1.7, 0.6], [1.7, 1.25], [2.3, 1.25]]

my_chaos_pucks, opp_chaos_pucks = compare_to_update_or_ignore(new_observation_pucks,
                                                              my_chaos_pucks,
                                                              my_collected_chaos,
                                                              opponent_chaos_pucks,
                                                              my_chaos_area,
                                                              opponent_chaos_area)

print "result"
print my_chaos_pucks
print " "
#print opp_chaos_pucks
