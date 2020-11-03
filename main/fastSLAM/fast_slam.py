'''
FastSlam object is inspired by https://github.com/nwang57/FastSLAM/blob/master/fast_slam.py

Methods annoted with a 1 are copy of the original object.
'''

import random, math, os
import numpy as np
from multiprocessing import Process, Queue
from copy import deepcopy
from .slam_helper import resampling, timer
from .particle2 import Particle2


class FastSlam:
    """Main class that implements the FastSLAM2.0 algorithm"""
    def __init__(self, x, y, orien, particle_size = 50):
        self.particles = [Particle2(x, y, orien + .1*(random.random()-.5)) for i in range(particle_size)]
        self.robot = Particle2(x, y, orien, is_robot=True)
        self.particle_size = particle_size
        self.resample_timer = 0

    def update_p(self, obs):
        # implement multiprocessing to speed up the particles update
        processes = []
        for p in self.particles:
            try:
                q = Queue()
                pro = Process(target=p.update, args=[obs, q])
                processes.append((pro, q))
                pro.start()
            except:
                print('warning: particle on landmark.')
                return
        for [pro, q], i in zip(processes, range(self.particle_size)):
            pro.join()
            self.particles[i] = q.get()[0]

    def run(self, mov, obs):
        '''
        Main function for localization

        Arguments:  
        mov (distance, angle): displacement of the robot  
        obs list (distance, angle): Observation of landmark(s)  
        '''
        # move particles
        
        # start by turn
        self.turn_left(int(mov[1] * 180/math.pi))
        # then move
        self.move_forward(mov[0])
            
        # update particles
        obs = np.array(obs)
        self.update_p(obs)

        # resample particles every three execution
        self.resample_timer += 1
        if self.resample_timer == 3:
            self.resample_timer = 0
            self.particles = resampling(self.particles, self.particle_size)
            
    def get_mean_pos(self):
        '''return the mean position of the particles'''
        pos_x = [p.pos_x for p in self.particles]
        pos_y = [p.pos_y for p in self.particles]
        mean_x = np.mean(pos_x)
        mean_y = np.mean(pos_y)
        return mean_x, mean_y
    
    def get_mean_orien(self):
        '''return the mean orientation of the particles'''
        angles = [p.orientation for p in self.particles]
        mean_angle = np.mean(angles)
        return mean_angle
    
    def get_mean_landmarks(self):
        

        # get most frequent number of lms
        numbers_lms = [len(p.landmarks) for p in self.particles]
        n_lms = max(set(numbers_lms), key=numbers_lms.count)
        n_particles = numbers_lms.count(n_lms)

        lms = [[0,0] for _ in range(n_lms)]

        for p in self.particles:
            if len(p.landmarks) != n_lms:
                continue
            for i, lm in enumerate(p.landmarks):
                lms[i][0] += lm.pos()[0]
                lms[i][1] += lm.pos()[1]
        
        for lm in lms:
            lm[0] /= n_particles
            lm[1] /= n_particles
        
        return lms

    def move_forward(self, step): # 1 #
        self.robot.forward(step)
        for p in self.particles:
            p.forward(step)

    def turn_left(self, angle): # 1 #
        self.robot.turn_left(angle)
        for p in self.particles:
            p.turn_left(angle)

    def turn_right(self, angle): # 1 #
        self.robot.turn_right(angle)
        for p in self.particles:
            p.turn_right(angle)

    def resample_particles(self): # 1 #
        new_particles = []
        weight = [p.weight for p in self.particles]
        index = int(random.random() * self.particle_size)
        beta = 0.0
        mw = max(weight)
        for i in range(self.particle_size):
            beta += random.random() * 2.0 * mw
            while beta > weight[index]:
                beta -= weight[index]
                index = (index + 1) % self.particle_size
            new_particle = deepcopy(self.particles[index])
            new_particle.weight = 1
            new_particles.append(new_particle)
        return new_particles

    def get_landmarks_dps(self, index=0):
        return [lm.pos() for lm in self.particles[index].landmarks]
    
    def get_particles_dps(self):
        return [particle.pos() for particle in self.particles]

    def stop(self):
        print('normal:',len(self.particles[0].landmarks))
        print('cleaned:',len(self.particles[1].landmarks))
    
    def store_landmarks(self, n_particle=None):
        
        if not n_particle:
            n_particle = self.particle_size

        with open(os.path.join('data','landmarks.csv'), 'w') as file:
            file.write('x,y\n')
            for i in range(n_particle):
                landmarks = self.get_landmarks_dps(i)
                for lm in landmarks:
                    file.write(f'{lm[0]},{lm[1]}\n')