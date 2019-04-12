#!/usr/bin/env python

import rosbag
import numpy as np

bag = rosbag.Bag("/home/egorpristanskiy/scans.bag", "r")
ranges = np.array([])
intens = np.array([])
for topic, msg, t in bag.read_messages(topics="/secondary_robot/scan"):
    ranges = np.append(ranges, msg.ranges)
    intens = np.append(intens, msg.intensities)
np.save("ranges", ranges)
np.save("intense", intens)