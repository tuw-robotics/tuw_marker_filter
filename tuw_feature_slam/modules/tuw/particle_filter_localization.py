'''
Created on Jan 6, 2017

@author: max
'''

import numpy as np
import math


def angle_difference(alpha0, angle1):
    return math.atan2(math.sin(alpha0 - angle1), math.cos(alpha0 - angle1))


# class Beam:
#
#     def __init__(self):
#         self.valid = False
#         self.length = 0.0
#         self.angle = 0.0
#         self.end_point = np.array([0, 0], np.double)

class Sample:

    def __init__(self, pose=None, orientation=None, weight=None):
        if pose is None:
            self.pose = np.array([[0], [0]], np.double)
        else:
            self.pose = pose
        if orientation is None:
            self.orientation = 0.0
        else:
            self.orientation = orientation
        if weight is None:
            self.weight = 0.0
        else:
            self.weight = weight

    def get_position(self):
        return self.pose

    def get_weight(self):
        return self.weight

    def get_orientation(self):
        return self.orientation

    def set_position(self, pose):
        self.pose = pose

    def set_weight(self, weight):
        self.weight = weight

    def set_orientation(self, orientation):
        self.orientation = orientation

    def tf(self):
        c = np.cos(self.orientation)
        s = np.sin(self.orientation)

        return np.array([[c, -s, self.pose[0, 0]],
                         [s, c, self.pose[0, 1]],
                         [0, 0, 1]
                         ], dtype=np.double)


class Vehicle(object):
    '''
    classdocs
    '''

    def __init__(self):
        '''
        Constructor
        '''
        self.dt = 0.1
        self.min_particles = 100
        self.max_particles = 5000
        self.laser_z_hit = 0.7
        self.laser_z_max = 5.0
        self.laser_z_short = 0.1
        self.laser_z_rand = 0.4
        self.laser_sigma_hit = 0.2
        self.laser_lambda_short = 0.1
        self.samples = []  # type: List[Sample]
        self.alpha1 = 0.1
        self.alpha2 = 0.1
        self.alpha3 = 0.1
        self.alpha4 = 0.1
        self.alpha5 = 0.1
        self.nr_of_samples = 1500
        self.resample_rate = 0.05
        self.sample_mode = 1  # 1,2
        self.sigma_static_position = 0.1
        self.sigma_static_orientation = 0.2
        self.enable_weighting = True
        self.enable_resample = True
        self.enable_update = True
        self.sigma_init_position = 0.5
        self.sigma_init_orientation = 0.40

    def init_samples(self):
        for i in range(self.nr_of_samples):
            init_pose = self.x[:2, 0] + np.random.normal(0.0, self.sigma_init_position, (2,1))
            init_orientation = self.x[2, 0] + np.random.normal(0.0, self.sigma_init_orientation)
            self.samples.append(Sample(pose=init_pose,
                                       orientation=init_orientation,
                                       weight=1.0))

    def nearest_marker(self, end_point, tolerance=0.15):
        for i in range(self.m.shape[0]):
            x_m = self.m[i, 1]
            y_m = self.m[i, 2]
            x = end_point[0, 0]
            y = end_point[0, 1]
            distance = np.sqrt((x_m - x) * (x_m - x) + (y_m - y) * (y_m - y))
            if distance < tolerance:
                return True, i, [self.m[i, 1], self.m[i, 2]], distance
        return False, None, None, None

    def set_odom(self, pose):
        if hasattr(self, 'x') == False:
            self.x = pose
            self.init_samples()
        self.odom = pose

    def define_map(self, m):
        self.m = m

    def get_marker(self, nr):
        if hasattr(self, 'm') == False:
            return [False, 0, 0]
        for i in range(self.m.shape[0]):
            if nr == int(self.m[i, 0]):
                return [True, self.m[i, 1], self.m[i, 2]]
        return [False, 0, 0]

    def prediction(self, u):
        if not hasattr(self, 'x'):
            return False
        self.update(u)

    def update(self, u):
        dt = self.dt
        v = u[0, 0]
        w = u[1, 0]
        eps = 0.000001

        pow_w = 0.0 if abs(w) < eps else pow(w, 2)
        pow_v = 0.0 if abs(v) < eps else pow(v, 2)

        if pow_w < eps and pow_v < eps:
            return False

        for s in self.samples:
            s_position = s.get_position()
            s_orientation = s.get_orientation()

            sigma_position = pow_v * self.alpha1 + pow_w * self.alpha2
            v_hat = v + np.random.normal(0.0, sigma_position)

            sigma_position = pow_v * self.alpha3 + pow_w * self.alpha4
            w_hat = w + np.random.normal(0.0, sigma_position)

            sigma_position = pow_v * self.alpha4 + pow_w * self.alpha5
            gamma = np.random.normal(0.0, sigma_position)

            if pow_w < eps:
                s_position[0, 0] = dt * v_hat * np.cos(s_orientation) + s_position[0, 0]
                s_position[1, 0] = dt * v_hat * np.sin(s_orientation) + s_position[1, 0]
            elif pow_v < eps:
                s_orientation = s_orientation + w_hat * dt + dt * gamma
            else:
                m_factor = v_hat / w_hat
                s_position[0, 0] = s_position[0, 0] - m_factor * np.sin(s_orientation) + m_factor * np.sin(
                    s_orientation + (w_hat * dt))
                s_position[1, 0] = s_position[1, 0] + m_factor * np.cos(s_orientation) - m_factor * np.cos(
                    s_orientation + (w_hat * dt))
                s_orientation = s_orientation + w_hat * dt + dt * gamma

            s.set_position(s_position)
            s.set_orientation(s_orientation)

        return True

    def weighting(self, z):
        # TODO: do not use all beams same as in mobile robotics
        for s in self.samples:
            s.set_weight(1.0)
            for observation in z:
                end_point_ws = observation[0,
                               1:3]  # TODO: check if transform is correct and also in which coordinate system the beams endpoint is given
                has_marker, id, marker_position, distance = self.nearest_marker(end_point_ws)

                if not has_marker:
                    s.set_weight(s.get_weight() * (self.laser_z_rand / self.laser_z_max))
                    continue
                # TODO: distance from likelyhood field
                s.set_weight(
                    s.get_weight() * (self.laser_z_hit * distance + (self.laser_z_rand / self.laser_z_max)))

        self.samples.sort(key=lambda x: x.get_weight(), reverse=True)
        weight_sum = 0.0
        for s in self.samples:
            weight_sum += s.get_weight()
        weight_max = 0.0
        for s in self.samples:
            s.set_weight(s.get_weight() / weight_sum)
            weight_max = max(s.get_weight(), weight_max)

    def resample(self):
        dt = self.dt  # TODO: what is the time here?
        M = np.floor(self.nr_of_samples * self.resample_rate)

        if self.sample_mode == 1:
            idx = 0
            while idx < M:
                s = self.samples[idx]
                self.samples[len(self.samples) - idx - 1] = s
                s_position = s.get_position()

                # normalization
                s_position[0, 0] = s_position[0, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
                s_position[1, 0] = s_position[1, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
                s.set_position(s_position)

                s_orientation = s.get_orientation() + np.random.normal(0.0, self.sigma_static_orientation * dt)
                s.set_orientation(s_orientation)
                idx += 1
        if self.sample_mode == 2:
            if self.samples[0].get_weight() <= 0.00001:
                return
            M = self.nr_of_samples
            samples_new = []
            r = np.random.uniform(0.0, 1.0 / M)
            c = self.samples[0].get_weight()
            i = 1
            f = False
            for m in range(1, M + 1):
                if f:
                    break
                u = r + (m - 1) * (1.0 / M)
                while u > c:
                    i += 1
                    if i > len(self.samples):
                        f = True
                        break
                    c = c + self.samples[i].get_weight()
                if not f:
                    # normalization
                    s = self.samples[i]
                    s_position = s.get_position()

                    s_position[0, 0] = s_position[0, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
                    s_position[1, 0] = s_position[1, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
                    s.set_position(s_position)

                    s_orientation = s.get_orientation() + np.random.normal(0.0, self.sigma_static_orientation * dt)
                    s.set_orientation(s_orientation)

                    samples_new.append(s)
            self.samples = samples_new
            print("new particle size: {}".format(len(self.samples)))
        else:
            pass

    def measurments(self, z):
        if self.enable_weighting:
            self.weighting(z)
        if self.enable_resample:
            self.resample()
