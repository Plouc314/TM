"""
    Imported from https://github.com/nwang57/FastSLAM/blob/master/particle.py
    Implements the particle which has motion model, sensor model and EKFs for landmarks.
"""

import random
import math
import numpy as np
from scipy import linalg
from .slam_helper import *
from .landmark import Landmark


class Particle(object):
    """Represents the robot and particles"""
    TOL = 1E-4

    def __init__(self, x, y, orien, scope=50,  is_robot=False):
        """pos_x: from left to right
           pos_y: from up to down
           orientation: [0,2*pi)
        """
        self.pos_x = x
        self.pos_y = y
        self.orientation = orien
        self.scope = scope
        self.is_robot = is_robot
        self.landmarks = []
        self.set_noise()
        self.weight = 1.0
        # Model error term will relax the covariance matrix
        self.obs_noise = np.array([[0.1, 0], [0, (3.0*math.pi/180)**2]])

    def set_noise(self):
        if self.is_robot:
            # Measurement Noise will detect same feature at different place
            self.bearing_noise = 0.1 # useless
            self.distance_noise = 0.1 # useless
            self.motion_noise = 0.5
            self.turning_noise = 1
        else:
            self.bearing_noise = 0
            self.distance_noise = 0
            self.motion_noise = 0
            self.turning_noise = 0 # unit: degree

    def forward(self, d):
        """Motion model.
           Moves robot forward of distance d plus gaussian noise
        """
        self.pos_x = self.pos_x + d * math.cos(self.orientation) + gauss_noise(0, self.motion_noise)
        self.pos_y = self.pos_y + d * math.sin(self.orientation) + gauss_noise(0, self.motion_noise)

    def turn_left(self, angle):
        self.orientation = (self.orientation + (angle + gauss_noise(0, self.turning_noise)) / 180. * math.pi) % (2 * math.pi)

    def turn_right(self, angle):
        self.orientation = (self.orientation - (angle + gauss_noise(0, self.turning_noise)) / 180. * math.pi) % (2 * math.pi)

    def pos(self):
        return (self.pos_x, self.pos_y)

    def set_pos(self, x, y, orien):
        """The arguments x, y are associated with the origin at the bottom left.
        """
        self.pos_x = x
        self.pos_y = y
        self.orientation = orien

    def reset_pos(self):
        self.set_pos(random.random() * WINDOWWIDTH, random.random() * WINDOWHEIGHT, random.random() * 2. * math.pi)

    def check_pos(self, x, y):
        """Checks if a particle is in the invalid place"""
        if x >= WINDOWWIDTH or y >= WINDOWHEIGHT or x <=0 or y <= 0:
            return True

    def dick(self):
        return [(self.pos_x, self.pos_y), (self.pos_x + self.scope * math.cos(self.orientation), self.pos_y + self.scope * math.sin(self.orientation))]

    def update(self, obs, q):
        """After the motion, update the weight of the particle and its EKFs based on the sensor data"""
        for o in obs:
            prob = np.exp(-70)
            if self.landmarks:
                # find the data association with ML
                prob, landmark_idx, ass_obs, ass_jacobian, ass_adjcov = self.find_data_association(o)
                if prob < self.TOL:
                    # create new landmark
                    self.create_landmark(o)
                else:
                    # update corresponding EKF
                    self.update_landmark(np.transpose(np.array([o])), landmark_idx, ass_obs, ass_jacobian, ass_adjcov)
            else:
                # no initial landmarks
                self.create_landmark(o)
            self.weight *= prob
        
        q.put([self]) ###

    def compute_jacobians(self, landmark):
        dx = landmark.pos_x - self.pos_x
        dy = landmark.pos_y - self.pos_y
        d2 = dx**2 + dy**2
        d = math.sqrt(d2)

        predicted_obs = np.array([[d],[math.atan2(dy, dx)]])
        jacobian = np.array([[dx/d,   dy/d],
                             [-dy/d2, dx/d2]])
        adj_cov = jacobian.dot(landmark.sig).dot(np.transpose(jacobian)) + self.obs_noise
        return predicted_obs, jacobian, adj_cov
    

    def guess_landmark(self, obs):
        """Based on the particle position and observation, guess the location of the landmark. Origin at top left"""
        distance, direction = obs
        lm_x = self.pos_x + distance * math.cos(direction)
        lm_y = self.pos_y + distance * math.sin(direction)
        return Landmark(lm_x, lm_y)

    def find_data_association(self, obs):
        """Using maximum likelihood to find data association"""
        prob = 0
        ass_obs = np.zeros((2,1))
        ass_jacobian = np.zeros((2,2))
        ass_adjcov = np.zeros((2,2))
        landmark_idx = -1
        for idx, landmark in enumerate(self.landmarks):
            predicted_obs, jacobian, adj_cov = self.compute_jacobians(landmark)
            p = multi_normal(np.transpose(np.array([obs])), predicted_obs, adj_cov)
            if p > prob:
                prob = p
                ass_obs = predicted_obs
                ass_jacobian = jacobian
                ass_adjcov = adj_cov
                landmark_idx = idx
        return prob, landmark_idx, ass_obs, ass_jacobian, ass_adjcov

    def create_landmark(self, obs):
        landmark = self.guess_landmark(obs)
        self.landmarks.append(landmark)

    def update_landmark(self, obs, landmark_idx, ass_obs, ass_jacobian, ass_adjcov):
        landmark = self.landmarks[landmark_idx]
        K = landmark.sig.dot(np.transpose(ass_jacobian)).dot(linalg.inv(ass_adjcov))
        new_mu = landmark.mu + K.dot(obs - ass_obs)
        new_sig = (np.eye(2) - K.dot(ass_jacobian)).dot(landmark.sig)
        landmark.update(new_mu, new_sig)

    def __str__(self):
        return str((self.pos_x, self.pos_y, self.orientation, self.weight))




