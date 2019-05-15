import numpy as np
from shapely.geometry import Point

new_observation_pucks = np.array([[0.95, 1.05, 1, 1, 0, 0],
                                    [2.05, 1.05, 4, 1, 0, 0],
                                    [0.5, 0.75, 10, 1, 0, 0],
                                    [1, 1.1, 2, 0, 1, 0],
                                    [1, 1, 3, 0, 0, 1],
                                    [2, 1, 5, 0, 0, 1],
                                    [1.05, 1.05, 6, 1, 0, 0],
                                    [0.5, 0.45, 9, 1, 0, 0],
                                    [1.95, 1.05, 7, 1, 0, 0],
                                    [2, 1.1, 8, 0, 1, 0],
                                    [0.5, 1.05, 11, 1, 0, 0]
                                  ])

purple_chaos_center = [1, 1.05]
yellow_chaos_center = [2.01, 1.05]
chaos_radius = 0.15


def parse_pucks(observation, purple_chaos_center, yellow_chaos_center, chaos_radius):
    """

    :param observation: [[x, y, id, r, g, b], [x, y, id, r, g, b]...]
    :param purple_chaos_center: (x, y)
    :param yellow_chaos_center: (x, y)
    :param chaos_radius: const
    :return: [[x, y, id, r, g, b], [x, y, id, r, g, b]...] format for each of chaoses and for  pucks_rgb
            (red cell puck, green cell puck, blue cell puck)
    """

    purple_chaos_pucks = []
    yellow_chaos_pucks = []
    pucks_rgb = []

    purple_chaos_center_point = Point(purple_chaos_center[0], purple_chaos_center[1])
    yellow_chaos_center_point = Point(yellow_chaos_center[0], yellow_chaos_center[1])

    # create circle buffer from the points
    purple_buffer = purple_chaos_center_point.buffer(chaos_radius)
    yellow_buffer = yellow_chaos_center_point.buffer(chaos_radius)

    # checkk if the other point lies within
    for puck in observation:
        current_puck = Point(puck[0], puck[1])
        if current_puck.within(purple_buffer):
            purple_chaos_pucks.append(puck)
        elif current_puck.within(yellow_buffer):
            yellow_chaos_pucks.append(puck)
        else:
            pucks_rgb.append(puck)

    purple_chaos_pucks = np.array(purple_chaos_pucks)
    yellow_chaos_pucks = np.array(yellow_chaos_pucks)
    pucks_rgb.sort(key=lambda t: t[1])
    pucks_rgb = np.array(pucks_rgb)

    return purple_chaos_pucks, yellow_chaos_pucks, pucks_rgb


purple_chaos_pucks, yellow_chaos_pucks, pucks_rgb = parse_pucks(new_observation_pucks, purple_chaos_center, yellow_chaos_center, chaos_radius)

print "purple_chaos_pucks"
print purple_chaos_pucks

print "yellow_chaos_pucks"
print yellow_chaos_pucks

print "pucks_rgb"
print pucks_rgb
