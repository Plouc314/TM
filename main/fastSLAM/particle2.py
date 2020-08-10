import random
import math
from operator import itemgetter
import numpy as np
from scipy import linalg
from .slam_helper import *
from .landmark import Landmark
from .particle import Particle

# Modified version of https://github.com/nwang57/FastSLAM/blob/master/particle2.py
# modified update(self) to implement multiprocessing, optimised a bit the function
# created select_landmarks method to optimise whole process

counter = Counter()

class Particle2(Particle):
    sensor_scope = 0
    sensor_angles = 0
    SELECT_LMS_THRESHOLD = 1.2
    """Inherit from Particle. Incorporates latest obs in the proposal distribution"""
    def __init__(self, x, y, orien, is_robot=False):
        super(Particle2, self).__init__(x, y, orien, is_robot)
        self.control_noise = np.array([[0.2, 0, 0], [0, 0.2, 0], [0, 0, (3.0*math.pi/180)**2]])
        self.obs_noise = np.array([[0.1, 0], [0, (3.0*math.pi/180)**2]])

    def update(self, obs, q): ###
        """After the motion, update the weight of the particle and its EKFs based on the sensor data"""
        counter.reset()
        # Find data association first
        data_association = []
        for o in obs:
            prob = 1e-30
            landmark_idx = -1
            if len(self.landmarks) != 0:
                # find the data association with ML
                prob, landmark_idx = self.pre_compute_data_association(o)
                if prob < self.TOL:
                    # create new landmark
                    landmark_idx = -1
            data_association.append((o, landmark_idx, prob))
        # Incorporates obs that creates new features last
        data_association.sort(key=itemgetter(1), reverse=True)
        # incorporate multiple obs to get the proposal distribution
        initial_pose = np.array([[self.pos_x], [self.pos_y], [self.orientation]])
        pose_mean = initial_pose
        pose_cov = self.control_noise
        inv_pos_cov = linalg.inv(pose_cov)
        for da in data_association:
            if da[1] > -1:
                # Using EKF to update the robot pose
                predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(self.landmarks[da[1]])
                pose_cov = linalg.inv( np.transpose(pose_jacobian).dot(linalg.inv(adj_cov).dot(pose_jacobian)) + inv_pos_cov )
                pose_mean = initial_pose + pose_cov.dot(np.transpose(pose_jacobian)).dot(linalg.inv(adj_cov)).dot(np.transpose(np.array([da[0]])) - predicted_obs)
                new_pose = np.random.multivariate_normal(pose_mean[:,0], pose_cov)
                self.set_pos(*new_pose)
            else:
                # Using the latest pose to create the landmark
                self.create_landmark(da[0])

        # update the landmarks EKFs
        for da in data_association:
            if da[1] > -1:
                predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(self.landmarks[da[1]])
                self.weight *= multi_normal(np.transpose(np.array([da[0]])), predicted_obs, adj_cov)
                self.update_landmark(np.transpose(np.array([da[0]])), da[1], predicted_obs, featurn_jacobian, adj_cov)
            else:
                self.weight *= da[2]
        prior = multi_normal(initial_pose, initial_pose, self.control_noise)
        prop = multi_normal(initial_pose, pose_mean, pose_cov)
        self.weight = self.weight * prior / prop

        #counter.result()
        q.put([self]) ###

    @counter
    def compute_jacobians(self, landmark):
        dx = landmark.pos_x - self.pos_x
        dy = landmark.pos_y - self.pos_y
        d2 = dx**2 + dy**2
        d = math.sqrt(d2)

        predicted_obs = np.array([[d],[math.atan2(dy, dx)]])
        feature_jacobian = np.array([[dx/d,   dy/d],
                                     [-dy/d2, dx/d2]])
        pose_jacobian = np.array([[-dx/d, -dy/d, 0],
                                  [dy/d2, -dx/d2, -1]])
        adj_cov = feature_jacobian.dot(landmark.sig).dot(np.transpose(feature_jacobian)) + self.obs_noise
        return predicted_obs, feature_jacobian, pose_jacobian, adj_cov

    @counter
    def pre_compute_data_association(self, obs):
        """Tries all the landmarks to incorporate the obs to get the proposal distribution and get the one with maximum likelihood"""
        prob = 0
        ass_obs = np.zeros((2,1))
        ass_jacobian = np.zeros((2,2))
        ass_adjcov = np.zeros((2,2))
        landmark_idx = -1
        initial_pose = np.array([[self.pos_x], [self.pos_y], [self.orientation]])
        inv_control_noise = linalg.inv(self.control_noise)
        tran_arr_obs = np.transpose(np.array([obs]))

        # select landmarks according to the sensor scope
        selected_lms = self.select_landmarks()

        for idx, landmark in selected_lms:
            # Sample a new particle pose from the proposal distribution
            predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(landmark)
            inv_adj_cov = linalg.inv(adj_cov)
            pose_cov = linalg.inv( np.transpose(pose_jacobian).dot(inv_adj_cov.dot(pose_jacobian)) + inv_control_noise )
            pose_mean = initial_pose + pose_cov.dot(np.transpose(pose_jacobian)).dot(inv_adj_cov).dot(tran_arr_obs - predicted_obs)
            new_pose = np.random.multivariate_normal(pose_mean[:,0], pose_cov)

            distance = euclidean_distance((landmark.pos_x, landmark.pos_y), (new_pose[0], new_pose[1]))
            direction = cal_direction((new_pose[0], new_pose[1]), (landmark.pos_x, landmark.pos_y))
            p = multi_normal(tran_arr_obs, np.array([[distance],[direction]]), adj_cov)
            if p > prob:
                prob = p
                landmark_idx = idx
        return prob, landmark_idx

    @counter
    def update_landmark(self, obs, landmark_idx, ass_obs, ass_jacobian, ass_adjcov):
        landmark = self.landmarks[landmark_idx]
        K = landmark.sig.dot(np.transpose(ass_jacobian)).dot(linalg.inv(ass_adjcov))
        new_mu = landmark.mu + K.dot(obs - ass_obs)
        new_sig = (np.eye(2) - K.dot(ass_jacobian)).dot(landmark.sig)
        landmark.update(new_mu, new_sig)
    
    def select_landmarks(self):
        ''' select the landmarks to be tested in pre_compute_data_association'''
        selected_lms = []

        for i, lm in enumerate(self.landmarks):
            # check according to the distance (sensor scope)
            if euclidean_distance(self.pos(), lm.pos()) < self.SELECT_LMS_THRESHOLD*self.sensor_scope:
                # check according to the angle of the observation (angle of vision)
                angle = cal_direction(self.pos(), lm.pos())-pi_2_pi(self.orientation)
                angle = pi_2_pi(angle)
                if abs(angle) < self.SELECT_LMS_THRESHOLD*self.sensor_angles[1]:
                    selected_lms.append((i,lm))

        #print(f'selected: {len(selected_lms)/len(self.landmarks)*100:.0f}%')

        return selected_lms
