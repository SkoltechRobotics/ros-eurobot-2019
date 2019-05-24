#!/usr/bin/env python
import numpy as np
import rosbag
import tf2_ros
import tf_conversions
from core_functions import  *
from np_Particle import ParticleFilter
from npTriangulation import find_position_triangulation
import scipy.optimize

PF_RATE = 20
BEAC_R = 0.096 / 2
WORLD_X = 3
WORLD_Y = 2
WORLD_BORDER = 0.022
BEAC_R = 0.096 / 2
BEAC_L = 0.100
BEAC_BORDER = 0.022
ORANGE_BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

if __name__ == "__main__":
    dataset = {
        "x_coords": np.array([]),
        "y_coords": np.array([]),
        "angles": np.array([]),
        "times": np.array([]),
    }
    dataset_commands = {
        "times": np.array([]),
        "coords": np.array([[0., 0., 0.]])
    }
    positions = np.array([[0.47, 0.4, 0]])
    # bag = rosbag.Bag('/home/egorpristanskiy/bagfile/filtered.bag')
    bag = rosbag.Bag('/home/egorpristanskiy/bagfile/oldbag.bag')
    distances = np.array([])
    intensities = np.array([])
    x = np.array([])
    y = np.array([])
    angles = np.array([])
    times = np.array([])
    #buffer = tf2_ros.Buffer()
    pf = ParticleFilter()
    beacons = ORANGE_BEACONS
    for topic, msg, t in bag.read_messages(topics="/secondary_robot/scan"):

        angles, distances = pf.get_landmarks(msg)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        landmarks = (np.array([x, y])).T
        start_coords = find_position_triangulation(beacons, landmarks, positions[-1])
        print(start_coords)
        positions = np.append(positions, start_coords[np.newaxis, :], axis=0)
    np.save("triangl_poses_filtered", positions)
    for topic, msg, t in bag.read_messages(topics="/secondary_robot/scan"):
        times = np.append(times, msg.header.stamp.to_sec())
    #dataset_commands["times"] -= dataset_commands["times"][0]
    np.save("trinagle_time2", times)
    i = 0
    # prev_time = dataset_commands["times"][-1]
    # prev_data = np.array
    # for topic, msg, t in bag.read_messages(topics="/secondary_robot/stm_command"):
    #     ++i
    #     data = msg.data.split()
    #     if data[0] == '8':
    #         coords = dataset_commands["coords"][-1, :] + (dataset_commands["times"][i] - dataset_commands["times"][i-1])*\
    #                                               np.array([data[1], data[2], data[3]]).astype(float)
    #         dataset_commands["coords"] = np.append(dataset_commands["coords"], coords[np.newaxis, :], axis=0)
    for topic, msg, t in bag.read_messages(topics='/tf'):
        if msg.transforms[0].header.frame_id == 'secondary_robot_odom' and msg.transforms[0].child_frame_id == 'secondary_robot':
            q = [msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y,
                  msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]
            _, _, a = tf_conversions.transformations.euler_from_quaternion(q)
            dataset["x_coords"] = np.append(dataset["x_coords"], msg.transforms[0].transform.translation.x)
            dataset["y_coords"] = np.append(dataset["y_coords"], msg.transforms[0].transform.translation.y)
            dataset["times"] = np.append(dataset["times"],  msg.transforms[0].header.stamp.to_sec())
            dataset["angles"] = np.append(dataset["angles"], a)
    dataset["start_coords2"] = np.array([0.27, 0.46, 0])
    np.save("record_bagfile_odom2", np.array([dataset], dtype=object))
    np.save("record_bagfile_movements2", np.array([dataset_commands], dtype=object))


