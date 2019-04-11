#!/usr/bin/env python
from core_functions import *
import numpy as np
import yaml

word_params_config_file_path = "/home/odroid/catkin_ws/src/ros-eurobot-2019/eurobot_loc/config/world_params.yaml"
particle_filter_config_path = "/home/odroid/catkin_ws/src/ros-eurobot-2019/eurobot_loc/config/particle_filter_params.yaml"
with open(word_params_config_file_path, 'r') as params:
    world_params = yaml.load(params)
with open(particle_filter_config_path, 'r') as params:
    pf_params = yaml.load(params)
# Dimensions of the playing field
WORLD_X = world_params["world_x"]
WORLD_Y = world_params["world_y"]
WORLD_BORDER = world_params["world_border"]
BEAC_R = world_params["beac_radius"]
BEAC_L = world_params["beac_l"]
BEAC_BORDER = world_params["beac_border"]

PURPLE_BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

YELLOW_BEACONS = np.array([[-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y - BEAC_L / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., BEAC_L / 2.]])

# parameters of lidar

LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)


class ParticleFilter:
    def __init__(self, start_x=0.293, start_y=0.425, start_angle=3 * np.pi / 2, color='orange'):

        self.start_coords = np.array([start_x, start_y, start_angle])
        self.color = color
        if color == 'purple':
            self.beacons = PURPLE_BEACONS
        else:
            self.beacons = YELLOW_BEACONS
        self.particles_num_from_measurement_model = pf_params["particles_num_from_measurement_model"]
        self.particles_num = pf_params["particles_num"]
        self.sense_noise = pf_params["sense_noise"]
        self.distance_noise = pf_params["distance_noise"]
        self.angle_noise = pf_params["angle_noise"]
        self.min_intens = pf_params["min_intens"]
        self.max_dist = pf_params["max_dist"]
        self.last = (start_x, start_y, start_angle)
        self.beac_dist_thresh = pf_params["beac_dist_thresh"]
        self.k_angle = pf_params["k_angle"]
        self.k_bad = pf_params["k_bad"]
        self.num_is_near_thresh = pf_params["num_is_near_thresh"]
        self.distance_noise_1_beacon = pf_params["distance_noise_1_beacon"]
        self.angle_noise_1_beacon = pf_params["angle_noise_1_beacon"]
        self.sigma_r = pf_params["sigma_r"]
        self.num_seeing_beacons = pf_params["num_seeing_beacons"]
        # Create Particles
        x = np.random.normal(start_x, self.distance_noise, self.particles_num)
        y = np.random.normal(start_y, self.distance_noise, self.particles_num)
        angle = np.random.normal(start_angle, self.angle_noise, self.particles_num) % (2 * np.pi)
        self.particles = np.array([x, y, angle]).T
        self.landmarks = np.zeros((2, 0))
        self.sigma_phi = pf_params["sigma_phi"]
        self.min_sin = pf_params["min_sin"]
        self.min_cost_function = pf_params["min_cost_function"]
        self.r_lid = np.array([])
        self.phi_lid = np.array([])
        self.beacon_ind = np.array([])

    @staticmethod
    def gaus(x, mu=0, sigma=1.):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- ((x - mu) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    @staticmethod
    def p_trans(a, d):
        x_beac = d * np.cos(a)
        y_beac = d * np.sin(a)
        return x_beac, y_beac


    def localization(self, delta_coords,  beacons):
        self.motion_model([delta_coords[0], delta_coords[1], delta_coords[2]])
        self.particles = self.measurement_model(self.particles, beacons)
        main_robot = self.calculate_main()
        return main_robot

    def measurement_model(self, particles, beacons):
        self.landmarks = beacons
        particles_measurement_model = self.get_particle_measurement_model(beacons)
        if beacons.shape[0] > 0:
            particles = np.concatenate((particles[:(self.particles_num - self.particles_num_from_measurement_model)], particles_measurement_model), axis=0)
        weights = self.weights(beacons, particles)
        inds = self.resample(weights, self.particles_num)
        # self.min_cost_function = np.mean(self.cost_function)
        particles = particles[inds, :]
        return particles

    def get_particle_measurement_model(self, landmarks):
            if landmarks.shape[0] != 0:
                beacons = cvt_global2local(self.beacons[np.newaxis, :], self.particles[:, np.newaxis])
                buf_beacons = beacons[0, :]
                distance_landmark_beacons = np.sqrt((landmarks[np.newaxis, np.newaxis, :, 0].T - buf_beacons[:, 0]) ** 2 +
                                                    (landmarks[np.newaxis, np.newaxis, :, 1].T - buf_beacons[:, 1]) ** 2)
                self.beacon_ind = np.argpartition(distance_landmark_beacons[:, 0, :], 2)[:, 0]
                r = (np.sqrt((landmarks[np.newaxis, :, 1]) ** 2 + (landmarks[np.newaxis, :, 0]) ** 2)).T
                phi = np.arctan2(landmarks[np.newaxis, :, 1], landmarks[np.newaxis, :, 0])
                phi = wrap_angle(phi).T
                r_lid = r + np.random.normal(0, self.sigma_r, self.particles_num_from_measurement_model)
                phi_lid = phi + np.random.normal(0, self.sigma_phi, self.particles_num_from_measurement_model)
                phi_lid = wrap_angle(phi_lid)
                if (len(self.beacon_ind) > 0 ):
                    y_lid = np.random.uniform(0, 2 * np.pi, self.particles_num_from_measurement_model)
                    x = self.beacons[self.beacon_ind[0],  0] + r_lid[0, :5] * np.cos(y_lid)
                    y = self.beacons[self.beacon_ind[0],  1] + r_lid[0, :5] * np.sin(y_lid)
                    theta = wrap_angle(y_lid - np.pi - phi_lid[0, :self.particles_num_from_measurement_model])
                    index = (x < 3) & (x > 0) & (y < 2) & (y > 0)
                    return np.array([x, y, theta]).T[index]
                else:
                    x = np.zeros(self.particles_num_from_measurement_model)
                    y = np.zeros(self.particles_num_from_measurement_model)
                    theta = np.zeros(self.particles_num_from_measurement_model)
                    return np.array([x, y, theta]).T
            else:
                    x = np.zeros(self.particles_num_from_measurement_model)
                    y = np.zeros(self.particles_num_from_measurement_model)
                    theta = np.zeros(self.particles_num_from_measurement_model)
                    return np.array([x, y, theta]).T


    def motion_model(self, delta):  # delta = [dx,dy,d_rot]
            if self.num_seeing_beacons > 1:
                d_n = self.distance_noise
                a_n = self.angle_noise
            else:
                d_n = self.distance_noise_1_beacon
                a_n = self.angle_noise_1_beacon
            x_noise = np.random.normal(0, d_n, self.particles_num)
            y_noise = np.random.normal(0, d_n, self.particles_num)
            angle_noise = np.random.normal(0, a_n, self.particles_num)
            noise = np.array([x_noise, y_noise, angle_noise]).T
            move_point = delta + noise
            self.particles = cvt_local2global(move_point, self.particles)
            '''
            self.particles[self.particles[:, 1] > 2 - 0.120, 1] = 2 - 0.050
            self.particles[self.particles[:, 1] < 0, 1] = 0
            self.particles[self.particles[:, 0] > 3 - 0.120, 0] = 3 - 0.050
            self.particles[self.particles[:, 0] < 0.120, 0] = 0.050
            '''

    def resample(self, weights, n=1000):
        indices_buf = []
        weights = np.array(weights)
        c = weights[0]
        j = 0
        m = n
        M = 1. / m
        r = np.random.uniform(0, M)
        for i in range(m):
            u = r + i * M
            while u > c:
                j += 1
                c = c + weights[j]
            indices_buf.append(j)
        return indices_buf

    def calculate_main(self):
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        zero_elem = self.particles[0, 2]
        # this helps if particles angles are close to 0 or 2*pi
        temporary = ((self.particles[:, 2] - zero_elem + np.pi) % (2.0 * np.pi)) + zero_elem - np.pi
        angle = np.mean(temporary)
        return np.array((x, y, angle))

    def weights(self, landmarks, particles):
        """Calculate particle weights based on their pose and landmarks"""
        if landmarks.shape[0] != 0:
            beacons = cvt_global2local(self.beacons[np.newaxis, :], particles[:, np.newaxis])
            buf_beacons = beacons[0, :]
            distance_landmark_beacons = np.sqrt((landmarks[np.newaxis, np.newaxis, :, 0].T - buf_beacons[:, 0])**2 +
                                                (landmarks[np.newaxis, np.newaxis, :, 1].T - buf_beacons[:, 1])**2)
            # distance_landmark_beacons = distance_landmark_beacons[np.where(distance_landmark_beacons < 4 * BEAC_R)]
            landmarks = landmarks[np.where(distance_landmark_beacons < 8*BEAC_R)[0]]
            self.beacon_ind = np.argpartition(distance_landmark_beacons[:, 0, :], 2)[:, 0]
            self.beacon_ind = self.beacon_ind[np.where(distance_landmark_beacons < 8*BEAC_R)[0]]
            r = (np.sqrt((landmarks[np.newaxis, :,   1])**2 + (landmarks[np.newaxis, :, 0])**2)).T + self.sigma_r**2
            phi = np.arctan2(landmarks[np.newaxis, :, 1], landmarks[np.newaxis, :, 0]) + self.sigma_phi**2
            phi = wrap_angle(phi).T
            self.r_lid = (np.sqrt(((beacons[:, self.beacon_ind, 1])**2 + (beacons[:, self.beacon_ind, 0])**2))).T
            self.phi_lid = (np.arctan2(beacons[:, self.beacon_ind, 1], beacons[:, self.beacon_ind, 0])).T
            self.phi_lid = wrap_angle(self.phi_lid)
            weights = self.gaus(r - self.r_lid, mu=0, sigma=self.sigma_r) * self.gaus(wrap_angle(phi - self.phi_lid), mu=0, sigma=self.sigma_phi)
            weights = (np.product(weights, axis=0))
            if np.sum(weights) > 0:
                if landmarks.shape[0] != 0:
                    weights = weights**(1./landmarks.shape[0])
                weights /= np.sum(weights)
            else:
                weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]
        else:
            weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]
        return weights

    def filter_scan(self, scan, intense):
        ranges = np.array(scan.ranges)
        intensities = np.array(scan.intensities)
        cloud = cvt_ros_scan2points(scan)
        index0 = (intensities > intense) & (ranges < self.max_dist)
        index1 = self.alpha_filter(cloud, self.min_sin)
        index = index0 * index1
        return np.where(index, ranges, 0)

    @staticmethod
    def alpha_filter(cloud, min_sin_angle):
        x, y = cloud.T
        x0, y0 = 0, 0
        x1, y1 = np.roll(x, 1), np.roll(y, 1)
        cos_angle = ((x - x0) * (x - x1) + (y - y0) * (y - y1)) / (np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0))
                                                                   * np.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)))
        sin_angle = np.sqrt(1 - cos_angle * cos_angle)
        index = sin_angle >= min_sin_angle
        return index

    def get_landmarks(self, scan, intense):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan, intense)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        return angles, distances


