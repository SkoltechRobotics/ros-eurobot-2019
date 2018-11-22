#!/usr/bin/env python
from core_functions import *
import numpy as np
np.set_printoptions(threshold=np.nan)

# Dimensions of the playing field
WORLD_X = 3
WORLD_Y = 2
WORLD_BORDER = 0.022
BEAC_R = 0.096 / 2
BEAC_L = 0.100
BEAC_BORDER = 0.022

ORANGE_BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

GREEN_BEACONS = np.array([[-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y - BEAC_L / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., BEAC_L / 2.]])

# parameters of lidar

LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)


class ParticleFilter:
    def __init__(self, particles_num=500, sense_noise=0.0050, distance_noise=5, angle_noise=0.02,
                 start_x=0.293, start_y=0.425, start_angle=3 * np.pi / 2, color='orange', min_intens=3500.0,
                 max_dist=3.7, beac_dist_thresh=0.200, k_angle=2, num_is_near_thresh=0.1, distance_noise_1_beacon=0.001,
                 angle_noise_1_beacon=0.05, k_bad=1):

        self.start_coords = np.array([start_x, start_y, start_angle])
        self.color = color
        if color == 'orange':
            self.beacons = ORANGE_BEACONS
        else:
            self.beacons = GREEN_BEACONS

        self.particles_num = particles_num
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        self.min_intens = min_intens
        self.max_dist = max_dist
        self.last = (start_x, start_y, start_angle)
        self.beac_dist_thresh = beac_dist_thresh
        self.k_angle = k_angle
        self.k_bad = k_bad
        self.num_is_near_thresh = num_is_near_thresh
        self.distance_noise_1_beacon = distance_noise_1_beacon
        self.angle_noise_1_beacon = angle_noise_1_beacon

        self.num_seeing_beacons = 3
        # Create Particles
        x = np.random.normal(start_x, distance_noise, particles_num)
        y = np.random.normal(start_y, distance_noise, particles_num)
        angle = np.random.normal(start_angle, angle_noise, particles_num) % (2 * np.pi)
        self.particles = np.array([x, y, angle]).T
        self.landmarks = [[], []]
        self.cost_function = []
        self.best_particles = {}
        self.min_sin = 0.4
        self.min_cost_function = 0

    @staticmethod
    def gaus(x, mu=0, sigma=1.):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- ((x - mu) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    @staticmethod
    def p_trans(a, d):
        x_beac = d * np.cos(a)
        y_beac = d * np.sin(a)
        return x_beac, y_beac

    def localisation(self, delta_coords, lidar_data):
        self.move_particles([delta_coords[0], delta_coords[1], delta_coords[2]])
        self.particles = self.particle_sense(lidar_data, self.particles)
        main_robot = self.calculate_main()
        return main_robot

    def particle_sense(self, scan, particles):
        angle, distance = self.get_landmarks(scan)
        x_coords, y_coords = self.p_trans(angle, distance)
        self.landmarks = np.array([x_coords, y_coords]).T
        weights = self.weights(self.landmarks, particles)
        inds = self.resample(weights)
        self.min_cost_function = np.mean(self.cost_function)
        particles = particles[inds, :]
        return particles

    def move_particles(self, delta):  # delta = [dx,dy,d_rot]
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
        self.particles[self.particles[:, 1] > 2 - 0.120, 1] = 2 - 0.120
        self.particles[self.particles[:, 1] < 0, 1] = 0
        self.particles[self.particles[:, 0] > 3 - 0.120, 0] = 3 - 0.120
        self.particles[self.particles[:, 0] < 0.120, 0] = 0.120

    def resample(self, weights):
        indices_buf = []
        weights = np.array(weights)
        c = weights[0]
        j = 0
        m = self.particles_num
        M = 1. / m
        r = np.random.uniform(0, M / 100000)
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
        # determines beacon positions (x,y) for every particle in it's local coords
        #cvt_local2global(self.beacons[np.newaxis, :], particles[:, np.newaxis]) works wrong
        beacons = cvt_global2local(self.beacons[np.newaxis, :], particles[:, np.newaxis])
        # find closest beacons to landmark
        dist_from_beacon = np.linalg.norm(beacons[:, np.newaxis, :, :] -
                                            landmarks[np.newaxis, :, np.newaxis, :], axis=3)
        ind_closest_beacon = np.argmin(dist_from_beacon, axis=2)
        closest_beacons = beacons[np.arange(beacons.shape[0])[:, np.newaxis], ind_closest_beacon]
        # Calculate cos of angle between landmark, beacon and particle
        dist_from_closest_beacon_to_particle = np.linalg.norm(closest_beacons, axis=2)
        dist_from_closest_beacon_to_landmark = np.linalg.norm(closest_beacons - landmarks[np.newaxis, :, :2],
                                                                axis=2)
        scalar_product_landmark = np.sum(closest_beacons * (closest_beacons - landmarks[np.newaxis, :, :2]), axis=2) / \
                                  dist_from_closest_beacon_to_particle
        # Calculate errors of position of landmarks
        errors = np.abs(dist_from_closest_beacon_to_landmark - BEAC_R) ** 2 + \
                 self.k_bad * np.where(scalar_product_landmark > 0, 0, scalar_product_landmark) ** 2
        # Too far real beacons go away: non valid
        is_near = dist_from_closest_beacon_to_landmark < self.beac_dist_thresh
        is_near_sum = np.sum(is_near, axis=0)
        is_near_or = (is_near_sum > is_near.shape[0] * self.num_is_near_thresh)
        num_good_landmarks = np.sum(is_near_or)
        sum_errors = np.sum(errors * is_near_or[np.newaxis, :], axis=1)
        if num_good_landmarks:
            self.cost_function = np.sqrt(sum_errors) / num_good_landmarks
        else:
            self.cost_function = np.ones(sum_errors.shape[0]) * 1000
        weights = self.gaus(self.cost_function, mu=0, sigma=self.sense_noise)
        if np.sum(weights) > 0:
            weights /= np.sum(weights)
        else:
            weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]
        self.best_particles = {}
        best_particles_inds = np.argsort(self.cost_function)[:10]
        self.best_particles["particles"] = particles[best_particles_inds]
        self.best_particles["cost_function"] = self.cost_function[best_particles_inds]
        self.best_particles["num_good_landmarks"] = num_good_landmarks
        self.best_particles["weights"] = weights[best_particles_inds]
        return weights

    def filter_scan(self, scan):
        ranges = np.array(scan.ranges)
        intensities = np.array(scan.intensities)
        cloud = cvt_ros_scan2points(scan)
        index0 = (intensities > self.min_intens) & (ranges < self.max_dist)
        index1 = self.alpha_filter(cloud, self.min_sin)
        index = index0 * index1
        # print(index0)
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

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ranges = np.array(scan.ranges)
        ind = self.filter_scan(scan)
        final_ind = np.where((np.arange(ranges.shape[0]) * ind) > 0)[0]
        # print(final_ind)
        angles = (LIDAR_DELTA_ANGLE * final_ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = ranges[final_ind]
        return angles, distances
