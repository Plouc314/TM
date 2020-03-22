"""
    This is the main file that starts the simulation.
    It contains a "World" object specifying the world settings, and a set of particles.
    Every time the robot moves, it generates random observations usded to update the particle sets
    to estimate the robot path as well as the landmarks locations.
"""
import sys
import random
import math
import numpy as np
from copy import deepcopy
from .slam_helper import resampling
from .particle2 import Particle2

class FastSlam(object):
    """Main class that implements the FastSLAM1.0 algorithm"""
    def __init__(self, x, y, orien, particle_size = 50):
        self.particles = [Particle2(x, y, orien - .5 + random.random()) for i in range(particle_size)]
        self.robot = Particle2(x, y, orien, is_robot=True)
        self.particle_size = particle_size

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
        for p in self.particles:
            p.update(obs)
        
        #self.particles = self.resample_particles()
        # try other resampling method
        self.particles = resampling(self.particles, self.particle_size)


        # update robot particle position
        #self.robot.pos_x, self.robot.pos_y = self.get_mean_pos()
        
        
    def get_mean_pos(self):
        ' return the mean position of the particles'
        pos_x = [p.pos_x for p in self.particles]
        pos_y = [p.pos_y for p in self.particles]
        mean_x = np.mean(pos_x)
        mean_y = np.mean(pos_y)
        return mean_x, mean_y

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

    def get_predicted_landmarks(self):
        return self.particles[0].landmarks


class SimulFastSlam(FastSlam):
    def __init__(self, x, y, orien, particle_size=50):
        super().__init__(x, y, orien, particle_size)


    def run_simulation(self):
        while True:
            for event in self.world.pygame.event.get():
                self.world.test_end(event)
            self.world.clear()
            key_pressed = self.world.pygame.key.get_pressed()
            if self.world.move_forward(key_pressed):
                self.move_forward(2)
                obs = self.robot.sense(self.world.landmarks, 2)
                for p in self.particles:
                    p.update(obs)
                self.particles = self.resample_particles()
            if self.world.turn_left(key_pressed):
                self.turn_left(5)
            if self.world.turn_right(key_pressed):
                self.turn_right(5)
            self.world.render(self.robot, self.particles, self.get_predicted_landmarks())

    





if __name__=="__main__":
    random.seed(5)
    simulator = FastSlam(80, 140, 0, particle_size=200)
    simulator.run_simulation()
