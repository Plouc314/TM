#tag: numpy

import numpy as np


cimport numpy as np
from cython.parallel import prange


cdef class Landmark:
    """Data structure of a landmark associated with a particle.
       Origin is the left-bottom point
    """
    cdef public:
        float pos_x
        float pos_y
        np.ndarray mu
        np.ndarray sig

    def __init__(self,float x,float y):
        self.pos_x = x
        self.pos_y = y
        self.mu = np.array([[self.pos_x],[self.pos_y]], dtype=np.float32)
        self.sig = np.eye(2, dtype=np.float32) * 99

    def pos(self):
        return (self.pos_x, self.pos_y)

    cdef void update(self, np.ndarray[float, ndim=2] mu, np.ndarray sig):
        self.mu = mu
        self.sig = sig
        self.pos_x = self.mu[0,0]
        self.pos_y = self.mu[1,0]

    def __str__(self):
        return f'{self.pos_x} {self.pos_y}'


# slam helper

cdef float gauss_noise(float mu, float sig):
    return np.random.normal(mu, sig)

cdef float euclidean_distance((float, float)a, (float, float)b):
    return np.hypot(b[0]-a[0], b[1]-a[1])


cdef float cal_direction((float, float)a, (float, float)b):
    """Calculate the angle of the vector a to b"""
    return np.arctan2(b[1]-a[1], b[0]-a[0])

cdef float multi_normal(np.ndarray[float, ndim=2] x, np.ndarray[float, ndim=2] mean, float[:,:] cov):
    """Calculate the density for a multinormal distribution"""
    cpdef float den = 2 * np.pi * np.sqrt(np.linalg.det(cov))
    cpdef np.ndarray num = np.exp(-0.5*np.transpose((x - mean)).dot(np.linalg.inv(cov)).dot(x - mean))
    cpdef np.ndarray result = num/den
    return result[0][0]

cdef int WINDOWWIDTH = 1600
cdef int linalg.inv(pose_cov)c:
        cpdef float TOL
        cpdef float pos_x
        cpdef float pos_y
        cpdef float orientation
        cpdef list landmarks
        cpdef int is_robot
        cpdef float weight

    cdef np.ndarray obs_noise

    cdef float bearing_noise 
    cdef float distance_noise
    cdef float motion_noise   
    cdef float turning_noise
    cdef np.ndarray control_noise


    def __init__(self, float x, float y, float orien, int is_robot=0):
        """pos_x: from left to right
           pos_y: from up to down
           orientation: [0,2*pi)
        """
        self.TOL = 1E-0
        self.pos_x = x
        self.pos_y = y
        self.orientation = orien
        self.is_robot = is_robot
        self.landmarks = []
        self.set_noise()
        self.weight = 1.0
        # Model error term will relax the covariance matrix
        self.obs_noise = np.array([[0.1, 0], [0, (3.0*np.pi/180)**2]], dtype=np.float32)
        self.control_noise = np.array([[0.2, 0, 0], [0, 0.2, 0], [0, 0, (3.0*np.pi/180)**2]], dtype=np.float32)

    def compute_jacobians(self,Landmark landmark):
        cpdef float dx = landmark.pos_x - self.pos_x
        cpdef float dy = landmark.pos_y - self.pos_y
        cpdef float d2 = dx**2 + dy**2
        cpdef float d = np.sqrt(d2)

        cpdef np.ndarray[float, ndim=2] predicted_obs = np.array([[d],[np.arctan2(dy, dx)]], dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] feature_jacobian = np.array([[dx/d,   dy/d],
                                     [-dy/d2, dx/d2]], dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] pose_jacobian = np.array([[-dx/d, -dy/d, 0],
                                  [dy/d2, -dx/d2, -1]], dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] adj_cov = feature_jacobian.dot(landmark.sig).dot(np.transpose(feature_jacobian)) + self.obs_noise
        return predicted_obs, feature_jacobian, pose_jacobian, adj_cov

    
    def update(self,np.ndarray obs):
        """After the motion, update the weight of the particle and its EKFs based on the sensor data"""
        # Find data association first
        # data_association: shape(n,4) -> [obs[i][0], obs[i][1], landmark_idx, prob]
        cpdef int lenght_data = obs.shape[0]
        cpdef np.ndarray data_association = np.empty((lenght_data, 4),dtype=np.float32)
        cpdef float[:,:] view_data = data_association
        cpdef int i = 0
        cpdef np.ndarray o
        cpdef double prob
        cpdef int landmark_idx
        for i in range(lenght_data):
            prob = 1e-30
            landmark_idx = -1
            if self.landmarks:
                # find the data association with ML
                prob, landmark_idx = self.pre_compute_data_association(obs[i])
                if prob < self.TOL:
                    # create new landmark
                    landmark_idx = -1
            view_data[i][0] = obs[i][0]
            view_data[i][1] = obs[i][1]
            view_data[i][2] = landmark_idx
            view_data[i][3] = prob
            
        # Incorporates obs that creates new features last
        # sort data_association by landmark_idx
        data_association = data_association[np.argsort(data_association[:,2])]
        # incorporate multiple obs to get the proposal distribution
        cpdef np.ndarray[float, ndim=2] initial_pose = np.array([[self.pos_x], [self.pos_y], [self.orientation]], dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] pose_mean = initial_pose
        cpdef np.ndarray[float, ndim=2] pose_cov = self.control_noise
        cpdef np.ndarray[float, ndim=2] inv_pos_cov = np.linalg.inv(self.control_noise)
        cpdef np.ndarray[float, ndim=2] xyorien = np.array([[self.pos_x], [self.pos_y], [self.orientation]], dtype=np.float32)
        cpdef float[:,:] predicted_obs
        cpdef float[:,:] featurn_jacobian 
        cpdef float[:,:] pose_jacobian 
        cpdef float[:,:] adj_cov
        cpdef float[:] new_pose
        view_data = data_association
        for i in range(lenght_data):
            if view_data[i,2] > -1:
                # Using EKF to update the robot pose
                predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(self.landmarks[view_data[i,2]])
                pose_cov = np.linalg.inv( np.transpose(pose_jacobian).dot(np.linalg.inv(adj_cov).dot(pose_jacobian)) + inv_pos_cov )
                pose_mean = xyorien + pose_cov.dot(np.transpose(pose_jacobian)).dot(np.linalg.inv(adj_cov)).dot(np.transpose(np.array([[view_data[i,0], view_data[i,1]]], dtype=np.float32)) - predicted_obs)
                new_pose = np.random.multivariate_normal(pose_mean[:,0], pose_cov)
                self.set_pos(*new_pose)
            else:
                # Using the latest pose to create the landmark
                self.create_landmark(np.array([view_data[i,0], view_data[i,1]]))

        # update the landmarks EKFs
        cpdef np.ndarray current_o
        for i in range(lenght_data):
            if view_data[i,2] > -1:
                current_o = np.array([[view_data[i,0], view_data[i,1]]], dtype=np.float32)
                predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(self.landmarks[view_data[i,2]])
                self.weight *= multi_normal(np.transpose(current_o), predicted_obs, adj_cov)
                self.update_landmark(np.transpose(current_o), view_data[i,2], predicted_obs, featurn_jacobian, adj_cov)
            else:
                self.weight *= view_data[i,3]
        cpdef float prior
        cpdef float prop
        prior = multi_normal(xyorien, initial_pose, pose_cov)
        prop = multi_normal(xyorien, pose_mean, pose_cov)
        self.weight = self.weight * prior / prop

        

   
    def pre_compute_data_association(self,float[:] obs):
        """Tries all the landmarks to incorporate the obs to get the proposal distribution and get the one with maximum likelihood"""
        cpdef float prob = 0
        cpdef np.ndarray[float, ndim=2] ass_obs = np.empty((2,1), dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] ass_jacobian = np.empty((2,2),dtype=np.float32)
        cpdef np.ndarray[float, ndim=2] ass_adjcov = np.empty((2,2),dtype=np.float32)
        cpdef int landmark_idx = -1
        cpdef int idx
        cpdef Landmark landmark
        cpdef float p
        cpdef int lenght_lm = len(self.landmarks)

        cpdef float[:,:] predicted_obs
        cpdef float[:,:] featurn_jacobian 
        cpdef float[:,:] pose_jacobian 
        cpdef float[:,:] adj_cov
        cpdef np.ndarray[float, ndim=2] pose_cov
        cpdef float[:,:] pose_mean
        cpdef float[:] new_pose
        
        cpdef float[:,:] xyorien = np.array([[self.pos_x], [self.pos_y], [self.orientation]], dtype=np.float32)
        cpdef float[:,:] inv_control_noise = np.linalg.inv(self.control_noise)

        cpdef float distance
        cpdef float direction

        for idx in range(lenght_lm):
            landmark = self.landmarks[idx]
            # Sample a new particle pose from the proposal distribution
            predicted_obs, featurn_jacobian, pose_jacobian, adj_cov = self.compute_jacobians(landmark)
            pose_cov = np.linalg.inv( np.transpose(pose_jacobian).dot(np.linalg.inv(adj_cov).dot(pose_jacobian)) + inv_control_noise )
            pose_mean = xyorien + pose_cov.dot(np.transpose(pose_jacobian)).dot(np.linalg.inv(adj_cov)).dot(np.transpose(np.array([obs])) - predicted_obs)
            new_pose = np.random.multivariate_normal(pose_mean[:,0], pose_cov).astype(np.float32)

            distance = euclidean_distance((landmark.pos_x, landmark.pos_y), (new_pose[0], new_pose[1]))
            direction = cal_direction((new_pose[0], new_pose[1]), (landmark.pos_x, landmark.pos_y))
            p = multi_normal(np.transpose(np.array([obs], dtype=np.float32)), np.array([[distance],[direction]], dtype=np.float32), adj_cov)
            if p > prob:
                prob = p
                landmark_idx = idx
        return prob, landmark_idx

    def create_landmark(self,np.ndarray obs):
        cpdef Landmark landmark = self.guess_landmark(obs)
        self.landmarks.append(landmark)

    def update_landmark(self,np.ndarray obs,int landmark_idx,np.ndarray ass_obs,np.ndarray ass_jacobian,np.ndarray ass_adjcov):
        cpdef Landmark landmark = self.landmarks[landmark_idx]
        cpdef np.ndarray K = landmark.sig.dot(np.transpose(ass_jacobian)).dot(np.linalg.inv(ass_adjcov))
        cpdef np.ndarray new_mu = landmark.mu + K.dot(obs - ass_obs)
        cpdef np.ndarray new_sig = (np.eye(2) - K.dot(ass_jacobian)).dot(landmark.sig)
        landmark.update(new_mu, new_sig)
    
    def guess_landmark(self,np.ndarray obs):
        """Based on the particle position and observation, guess the location of the landmark. Origin at top left"""
        cpdef float distance = obs[0]
        cpdef float direction = obs[1]
        cpdef float lm_x = self.pos_x + distance * np.cos(direction)
        cpdef float lm_y = self.pos_y + distance * np.sin(direction)
        return Landmark(lm_x, lm_y)
    
    def set_noise(self):
        if self.is_robot:
            # Measurement Noise will detect same feature at different place
            self.bearing_noise = 0.1
            self.distance_noise = 0.1
            self.motion_noise = 0.5
            self.turning_noise = 1
        else:
            self.bearing_noise = 0
            self.distance_noise = 0
            self.motion_noise = 0
            self.turning_noise = 0 # unit: degree

    def forward(self, float d):
        """Motion model.
           Moves robot forward of distance d plus gaussian noise
        """
        cpdef float x = self.pos_x + d * np.cos(self.orientation) + gauss_noise(0, self.motion_noise)
        cpdef float y = self.pos_y + d * np.sin(self.orientation) + gauss_noise(0, self.motion_noise)
        if self.check_pos(x, y):
            if self.is_robot:
                return
            else:
                self.reset_pos()
                return
        else:
            self.pos_x = x
            self.pos_y = y
    
    def turn_left(self, float angle):
        self.orientation = (self.orientation + (angle + gauss_noise(0, self.turning_noise)) / 180. * np.pi) % (2 * np.pi)

    def turn_right(self, float angle):
        self.orientation = (self.orientation - (angle + gauss_noise(0, self.turning_noise)) / 180. * np.pi) % (2 * np.pi)

    def pos(self):
        return (self.pos_x, self.pos_y)

    def set_pos(self, float x, float y, float orien):
        """The arguments x, y are associated with the origin at the bottom left.
        """
        if x > WINDOWWIDTH:
            x = WINDOWWIDTH
        if y > WINDOWHEIGHT:
            y = WINDOWHEIGHT
        self.pos_x = x
        self.pos_y = y
        self.orientation = orien

    cpdef void reset_pos(self):
        self.set_pos(np.random.random() * WINDOWWIDTH, np.random.random() * WINDOWHEIGHT, np.random.random() * 2. * np.pi)

    cpdef int check_pos(self, float x, float y):
        """Checks if a particle is in the invalid place"""
        if x >= WINDOWWIDTH or y >= WINDOWHEIGHT or x <=0 or y <= 0:
            return 1
        return 0