#!/usr/bin/env python
import numpy as np


def cvt_local2global(local_point, src_point):
    size = local_point.shape[-1]
    x, y, a = 0, 0, 0
    if size == 3:
        x, y, a = local_point.T
    elif size == 2:
        x, y = local_point.T
    X, Y, A = src_point.T
    x1 = x * np.cos(A) - y * np.sin(A) + X
    y1 = x * np.sin(A) + y * np.cos(A) + Y
    a1 = (a + A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x1, y1, a1]).T
    elif size == 2:
        return np.array([x1, y1]).T
    else:
        return


def cvt_global2local(global_point, src_point):
    size = global_point.shape[-1]
    x1, y1, a1 = 0, 0, 0
    if size == 3:
        x1, y1, a1 = global_point.T
    elif size == 2:
        x1, y1 = global_point.T
    X, Y, A = src_point.T
    x = x1 * np.cos(A) + y1 * np.sin(A) - X * np.cos(A) - Y * np.sin(A)
    y = -x1 * np.sin(A) + y1 * np.cos(A) + X * np.sin(A) - Y * np.cos(A)
    a = (a1 - A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x, y, a]).T
    elif size == 2:
        return np.array([x, y]).T
    else:
        return


def find_src(global_point, local_point):
    x, y, a = local_point.T
    x1, y1, a1 = global_point.T
    A = (a1 - a) % (2 * np.pi)
    X = x1 - x * np.cos(A) + y * np.sin(A)
    Y = y1 - x * np.sin(A) - y * np.cos(A)
    return np.array([X, Y, A]).T


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

    #
    # def resample(self, weights):
    #     indices = []
    #     weights = np.array(weights)
    #     c = weights[0]
    #     j = 0
    #     m = self.particles_num
    #     M = 1./m
    #     r = np.random.uniform(0, M)
    #     for i in range(m):
    #         u = r + i*M
    #         while u > c:
    #             j += 1
    #             c = c + weights[j]
    #         indices.append(j-1)
    #     return indices


    def resample(self, weights):
        indices_buf = []
        weights = np.array(weights)
        c = weights[0]
        j = 0
        m = self.particles_num
        M = 1./m
        r = np.random.uniform(0, M/100000)
        for i in range(m):
            u = r + i*M
            while u > c:
                j += 1
                c = c + weights[j]
            indices_buf.append(j)
        return indices_buf

    # def resample(self, weights):
    #     """ according to weights """
    #     n = self.particles_num
    #     weigths = np.array(weights)
    #     indices = []
    #     C = np.append([0.], np.cumsum(weigths))
    #     j = 0
    #     u0 = (np.random.rand() + np.arange(n)) / n
    #     for u in u0:
    #         while j < len(C) and u > C[j]:
    #             j += 1
    #         indices += [j - 1]
    #     return indices

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
        # determines 3 beacon positions (x,y) for every particle in it's local coords
        beacons = cvt_local2global(self.beacons[np.newaxis, :], particles[:, np.newaxis])
        print(beacons)
        # find closest beacons to landmark
        dist_from_beacon = np.linalg.norm(beacons[:, np.newaxis, :, :] -
                                          landmarks[np.newaxis, :, np.newaxis, :], axis=3)
        ind_closest_beacon = np.argmin(dist_from_beacon, axis=2)
        closest_beacons = beacons[np.arange(beacons.shape[0])[:, np.newaxis], ind_closest_beacon]
        # Calculate cos of angle between landmark, beacon and particle
        dist_from_closest_beacon_to_particle = np.sum(closest_beacons *
                                                      (closest_beacons - landmarks[np.newaxis, :, :2]), axis=2)
        dist_from_closest_beacon_to_landmark = (np.linalg.norm(closest_beacons - landmarks[np.newaxis, :, :2], axis=2))
        is_near = dist_from_closest_beacon_to_landmark < self.beac_dist_thresh
        is_near_sum = np.sum(is_near, axis=0)
        is_near_or = (is_near_sum > is_near.shape[0] * self.num_is_near_thresh)
        is_beacon_seeing = np.ones(3) * False
        for i in range(3):
            is_beacon_seeing[i] = np.any(i == ind_closest_beacon[:, is_near_or])
        self.num_seeing_beacons = np.sum(is_beacon_seeing)
        num_good_landmarks = np.sum(is_near_or)
        dist_from_closest_beacon_to_landmark -= BEAC_R
        dist_from_closest_beacon_to_landmark *= dist_from_closest_beacon_to_landmark
        self.cost_function = dist_from_closest_beacon_to_landmark
        d_f_b = dist_from_closest_beacon_to_landmark[:, 0] * dist_from_closest_beacon_to_particle[:, 0]
        print(np.linalg.norm(beacons[:, 1]))
        d_f_b /= np.linalg.norm(beacons[:, 1])
        zero_ind = np.where(d_f_b < 0)[0]
        np.put(d_f_b, zero_ind, 0)
        d_f_b = d_f_b[:, np.newaxis]
        self.cost_function += d_f_b
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

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ind = np.where(np.logical_and(scan[:, 1] > self.min_intens, scan[:, 0] < self.max_dist))[0]
        angles = (LIDAR_DELTA_ANGLE * ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = scan[ind, 0]
        return angles, distances
