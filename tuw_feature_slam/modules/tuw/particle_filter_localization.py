'''
Created on Jan 6, 2017

@author: max
'''

import numpy as np
import math
import scipy.stats

from tuw.plot import transform_pose


def angle_difference(alpha0, angle1):
    return math.atan2(math.sin(alpha0 - angle1), math.cos(alpha0 - angle1))


# class Beam:
#
#     def __init__(self):
#         self.valid = False
#         self.length = 0.0
#         self.angle = 0.0
#         self.end_point = np.array([0, 0], np.double)

class Match:

    def __init__(self,obs_id, map_id, distance):
        self.observation_id = obs_id
        self.map_id = map_id
        self.distance = distance

    def __str__(self):
        return "observation {} -> {} marker, dist: {}".format(self.observation_id, self.map_id, self.distance)

class MatchInfo:

    def __init__(self):
        self.sample_to_match = {}

    def add_match(self, s, obs_id, map_id, distance):
        if s.get_idx() in self.sample_to_match:
            self.sample_to_match[s.get_idx()].append(Match(obs_id, map_id, distance))
        else:
            self.sample_to_match[s.get_idx()] = []
            self.sample_to_match[s.get_idx()].append(Match(obs_id, map_id, distance))

    def print_best_sample(self):
        best_dist = np.finfo(np.double).max
        best_idx = 0
        for s_idx, match_obj_list in self.sample_to_match.iteritems():
            dist_sum = 0.0
            for match_obj in match_obj_list:
                dist_sum += match_obj.distance
            dist_sum = dist_sum / len(match_obj_list)
            if dist_sum < best_dist:
                best_dist = dist_sum
                best_idx = s_idx
        print("best particle id: {}".format(best_idx))
        print("Match info:")
        for match_obj in self.sample_to_match[best_idx]:
            print("{}".format(match_obj.__str__()))

class Sample:

    def __init__(self, idx, pose=None, orientation=None, weight=None, x=None, y=None):
        self.idx = idx
        if pose is None:
            self.pose = np.array([[0], [0]], np.double)
        else:
            self.pose = pose
        if x is not None and y is not None:
            self.pose = np.array([[x], [y]], dtype=np.double)
        if orientation is None:
            self.orientation = 0.0
        else:
            self.orientation = orientation
        if weight is None:
            self.weight = 0.0
        else:
            self.weight = weight
        self.hit = False
        self.obs_id_to_marker_id = {}

    def get_idx(self):
        return self.idx

    def set_idx(self, idx):
        self.idx = idx

    def clone(self):
        return Sample(self.idx, self.pose.copy(), self.orientation, self.weight)

    def set_observation_marker_link(self, o_id, m_id):
        self.obs_id_to_marker_id[o_id] = m_id

    def get_observation_marker_links(self):
        return self.obs_id_to_marker_id

    def set_hit(self, val):
        self.hit = val

    def get_pose(self):
        return np.concatenate([self.pose.reshape(1, 2).copy(), np.array([[self.orientation]])], axis=1)

    def get_position(self):
        return self.pose.copy()

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
        self.laser_sigma_hit = 0.5
        self.laser_lambda_short = 0.1
        self.samples = []  # type: List[Sample]
        self.alpha1 = 0.1
        self.alpha2 = 0.1
        self.alpha3 = 0.1
        self.alpha4 = 0.1
        self.alpha5 = 0.1
        self.nr_of_samples = 128
        self.resample_rate = 0.25
        self.sample_mode = 2  # 1,2
        self.sigma_static_position = 0.05
        self.sigma_static_orientation = 0.05
        self.enable_weighting = True
        self.enable_resample = True
        self.enable_update = True
        self.sigma_init_position = 0.10
        self.sigma_init_orientation = 0.10
        self.weight_max = 1.0
        self.is_initialized = False

    def distribute_particles_grid(self, xrange, yrange):
        if self.is_initialized:
            return
        angle_division = 4
        samples_per_angle = self.nr_of_samples / angle_division
        A = (xrange[1] - xrange[0]) * (yrange[1] - yrange[0])
        samples_per_m2 = np.double(samples_per_angle) / np.double(A)
        d = 1.0 / np.sqrt(samples_per_m2)
        del self.samples[:]
        xxrange = np.arange(xrange[0] + d / 2.0, xrange[1], d)
        yyrange = np.arange(yrange[0] + d / 2.0, yrange[1], d)
        angle_range = np.arange(-np.pi, np.pi, (2.0 * np.pi) / angle_division)
        s_id = 0
        for x_in in xxrange:
            for y_in in yyrange:
                for theta_in in angle_range:
                    self.samples.append(Sample(idx=s_id,
                                               x=x_in,
                                               y=y_in,
                                               orientation=theta_in,
                                               weight=1.0))
                    s_id += 1
        self.nr_of_samples = len(self.samples)
        print("{} samples created.".format(len(self.samples)))
        self.is_initialized = True

    def get_weight_max(self):
        return self.weight_max

    def init_samples(self):
        if self.is_initialized:
            return
        for i in range(self.nr_of_samples):
            init_pose = self.x[:2, 0] + np.random.normal(0.0, self.sigma_init_position, (2, 1))
            init_orientation = self.x[2, 0] + np.random.normal(0.0, self.sigma_init_orientation)
            self.samples.append(Sample(idx=i,
                                       pose=init_pose,
                                       orientation=init_orientation,
                                       weight=1.0))
        self.is_initialized = True

    def nearest_marker(self, end_point, tolerance=5000):
        best_marker_dist = np.finfo(np.double).max
        best_marker_id = 0
        for i in range(self.m.shape[0]):
            x_m = self.m[i, 1]
            y_m = self.m[i, 2]
            theta_m = self.m[i, 3]
            x = end_point[0, 0]
            y = end_point[1, 0]
            theta = end_point[2, 0]
            distance = np.sqrt((x_m - x) * (x_m - x) + (y_m - y) * (y_m - y))
            if distance < best_marker_dist:
                best_marker_dist = distance
                best_marker_idx = i
                best_marker_id = self.m[i,0]
        if best_marker_dist < tolerance:
            return True, best_marker_id, [self.m[best_marker_idx, 1], self.m[best_marker_idx, 2]], best_marker_dist
        else:
            return False, best_marker_id, [self.m[best_marker_idx, 1], self.m[best_marker_idx, 2]], best_marker_dist

    def set_odom(self, pose):
        if hasattr(self, 'x') == False:
            self.x = pose
            self.init_samples()
        self.odom = pose
        # self.x = pose

    def normal(self, s, dt):
        s_position = s.get_position()

        # normalization
        s_position[0, 0] = s_position[0, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
        s_position[1, 0] = s_position[1, 0] + np.random.normal(0.0, self.sigma_static_position * dt)
        s_orientation = s.get_orientation() + np.random.normal(0.0, self.sigma_static_orientation * dt)

        s.set_position(s_position)
        s.set_orientation(s_orientation)

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
        if self.enable_resample:
            self.resample()
        if self.enable_update:
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
        match_info = MatchInfo()
        s_idx = 0
        for s in self.samples:
            s.set_hit(False)
            q = 1.0
            qr = self.laser_z_rand / self.laser_z_max
            for o_id, observation in enumerate(z):
                end_point_ws = observation[0, 1:4].reshape(3,1).copy()  # TODO: check if transform is correct and also in which coordinate system the beams endpoint is given
                end_point_ws = transform_pose(end_point_ws.reshape(3,1), s.get_pose().reshape(3, 1))
                has_marker, map_id, marker_position, dist = self.nearest_marker(end_point_ws)
                qh = 0.0
                if has_marker:
                    s.set_hit(True)
                    gauss_sample = scipy.stats.norm.pdf(dist, 0.0, self.laser_z_hit)
                    if gauss_sample < 0.0:
                        raise ValueError("gauss sample < 0.0 dist: {}, sample: {}".format(dist, gauss_sample))
                    qh = self.laser_z_hit * gauss_sample
                    s.set_observation_marker_link(o_id, map_id)
                    match_info.add_match(s,o_id,map_id,dist)
                if (qr + qh) < 0.0:
                    raise ValueError("qr + qh = {} + {} < 0.0".format(qr, qh))
                q *= (qr + qh)
                # if not has_marker:
                #    s.set_weight(s.get_weight() * (self.laser_z_rand / self.laser_z_max))
                #    continue
            s.set_weight(q)
            s_idx += 1
        self.samples.sort(key=lambda x: x.get_weight(), reverse=True)
        weight_sum = 0.0
        for s in self.samples:
            weight_sum += s.get_weight()
        self.weight_max = np.finfo(np.double).min
        self.weight_min = np.finfo(np.double).max
        for s in self.samples:
            s.set_weight(s.get_weight() / weight_sum)
            self.weight_max = np.max([s.get_weight(), self.weight_max])
            self.weight_min = np.min([s.get_weight(), self.weight_min])
        self.x = self.samples[0].get_pose().reshape(3, 1)
        print("max weight: {}".format(self.weight_max))
        print("min weight: {}".format(self.weight_min))
        print("self.particles[0] id: {}".format(self.samples[0].get_idx()))
        #just for visualization purpose
        for k,v in self.samples[0].get_observation_marker_links().iteritems():
            z[k, 0] = v
        # print("odom pose: {}".format(self.odom))
        # print("estimated pose: {}".format(self.x))
        # print("best particle pose: {}".format(self.samples[0].get_pose()))

    def resample(self):
        dt = self.dt
        M = np.floor(self.nr_of_samples * self.resample_rate)
        #assumption: self.samples are sorted based on weight in decreasing order
        if self.sample_mode == 1:
            idx = 0
            while idx < M:
                s = self.samples[idx]
                s_cloned = s.clone()
                self.samples[len(self.samples) - idx - 1] = s_cloned
                s_cloned.set_idx(len(self.samples) - idx - 1)
                self.normal(s_cloned, dt)

                idx += 1
        if self.sample_mode == 2:
            if self.samples[0].get_weight() <= 0.00001:
                return
            M = int(np.floor(self.nr_of_samples * self.resample_rate))
            Minv = 1.0 / M
            samples_new = []
            r = np.random.uniform(0.0, 1.0 / M)
            c = self.samples[0].get_weight()
            i = 0
            for m in range(0, M):
                u = r + m * Minv
                while u > c and i < M:
                    c = c + self.samples[i].get_weight()
                    i = i + 1
                # normalization
                s = self.samples[i]
                s_new = s.clone()
                s_new.set_idx(len(samples_new))

                self.normal(s_new, dt)

                samples_new.append(s_new)
            self.samples = samples_new

            if self.nr_of_samples < len(self.samples):
                del self.samples[(self.nr_of_samples-1):]
            while self.nr_of_samples > len(self.samples):
                parent = self.samples[np.random.randint(0,len(self.samples))]
                child = parent.clone()
                self.normal(child, dt)
                child.set_idx(len(self.samples))
                self.samples.append(child)
            print("new particle size: {}".format(len(self.samples)))
        else:
            pass

    def measurments(self, z):
        if self.enable_weighting:
            self.weighting(z)
