from multiprocessing import Process, Queue
import sys
import random
import math
import numpy as np
from copy import deepcopy
from .slam_helper import resampling, timer
from .particle2 import Particle2

class FastSlam(object):
    """Main class that implements the FastSLAM2.0 algorithm"""
    def __init__(self, x, y, orien, reduce_lms=True, particle_size = 50):
        self.particles = [Particle2(x, y, orien + .5*(random.random()-.5)) for i in range(particle_size)]
        self.robot = Particle2(x, y, orien, is_robot=True)
        self.reduce_lms = reduce_lms
        self.particle_size = particle_size
        self.clean_counter = 0

    @timer
    def clean_lms(self):
        for p in self.particles:
            original_lm = deepcopy(p.landmarks)
            p.landmarks = original_lm[:-10]
            last_ones = list(np.random.choice(original_lm[-10:],8))
            p.landmarks.extend(last_ones)


    @timer
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
        for [pro, q], i in zip(processes, range(len(self.particles))):
            pro.join()
            self.particles[i] = q.get()[0]

    @timer
    def __call__(self, mov, obs):
        '''
        Main function for localization

        Arguments:
        mov (distance, angle): displacement of the robot
        obs list (distance, angle): Observation of landmark(s)
        '''
        # move particles
        if mov[0]: 
            self.move_forward(mov[0])
        else:
            self.turn_left(int(mov[1] * 180/math.pi))
        # update particles
        self.update_p(np.array(obs, dtype=np.float32))

        # try other resampling method
        self.particles = resampling(self.particles, self.particle_size)
       
        # try clean landmarks
        if self.reduce_lms:
            self.clean_counter += 1
            if self.clean_counter == 4:
                if len(self.particles[0].landmarks) > 10:
                    self.clean_lms()
                self.clean_counter = 0
            


        
        
    def get_mean_pos(self):
        ' return the mean position of the particles'
        pos_x = [p.pos_x for p in self.particles]
        pos_y = [p.pos_y for p in self.particles]
        mean_x = np.mean(pos_x)
        mean_y = np.mean(pos_y)
        return mean_x, mean_y
    
    def get_mean_orien(self):
        ' return the mean orientation of the particles'
        angles = [p.orientation for p in self.particles]
        mean_angle = np.mean(angles)
        return mean_angle

    def move_forward(self, step):
        self.robot.forward(step)
        for p in self.particles:
            p.forward(step)

    def turn_left(self, angle):
        self.robot.turn_left(angle)
        for p in self.particles:
            p.turn_left(angle)

    def turn_right(self, angle):
        self.robot.turn_right(angle)
        for p in self.particles:
            p.turn_right(angle)

    def resample_particles(self):
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

    def get_predicted_landmarks(self, index):
        return self.particles[index].landmarks

    def stop(self):
        print('normal:',len(self.particles[0].landmarks))
        print('cleaned:',len(self.particles[1].landmarks))