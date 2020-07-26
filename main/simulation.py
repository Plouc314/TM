import pygame
import math, random, pickle, os
import numpy as np
from geometry import segment_intersect
from fastSLAM.fast_slam import FastSlam
from fastSLAM.slam_helper import euclidean_distance, sense_direction, timer
from interface import Interface, Delayer, C, Const, Robot


class BaseSimulation:
    robot = None
    landmarks, particles = [], []
    def __init__(self, with_robot=True, plan=None):
        
        if with_robot:
            self.robot = Interface.robot

        if plan:
            self.plan_lines = Interface.create_plan_lines(plan)
        else:
            self.plan_lines = Interface.plan_lines

        # create fastslam object
        self.fastslam = FastSlam(Interface.robot.x, Interface.robot.y, Interface.robot.orien, particle_size=50)

        # create 20 angles to simulated the rotating sensor
        self.angles = list(np.linspace(Const['SENSOR_ANGLES'][0], Const['SENSOR_ANGLES'][1], 20))

    def collision(self, position=None, orientation=None):
        '''In the case where the simulation doesn't have robot object, specify orien/pos in arguments'''
        
        if self.robot:
            orien = self.robot.orien
            pos = self.robot.get_pos()
        else:
            orien = orientation
            pos = position

        cols = []
        for line0 in self.plan_lines:
            for angle in self.angles:
                # get the end of the sensor's line
                x = math.cos(orien + angle) * Const['SENSOR_SCOPE'] + pos[0] + Const['SENSOR_MARGE']
                y = math.sin(orien + angle) * Const['SENSOR_SCOPE'] + pos[1] + Const['SENSOR_MARGE']
                
                line1 = [pos, [x,y]]
                
                # test for vertical lines
                try:
                    intersect = segment_intersect(line0, line1)
                except:
                    # if a line is vertical, m of mx + h is inf, it's produce an error in geometry.slope
                    # so swap the coor to get an horizontal line
                    inv_line0 = [dp[::-1] for dp in line0]
                    inv_line1 = [dp[::-1] for dp in line1]
                    intersect = segment_intersect(inv_line0, inv_line1)
                    if intersect != None:
                        intersect = intersect[::-1]
                if intersect != None:
                    noise = self.get_noise(10, 2)
                    intersect = [ c + noise[i] for i, c in enumerate(intersect)]
                    cols.append(intersect)
        return cols

    @staticmethod
    def get_noise(scale, shape):
        random.seed(random.randint(0,100))
        # create noise of desired shape
        noise = [ int(random.randint(0, scale) - scale/2) for i in range(shape)]
        return noise
    
    @staticmethod
    def get_particles_dps(fastslam):
        particles = fastslam.particles
        return [particles[i].pos() for i in range(len(particles))]

    @staticmethod
    def get_landmarks_dps(fastslam, index=0):
        return [lm.pos() for lm in fastslam.get_predicted_landmarks(index)]

    @staticmethod
    def store_landmarks(fastslam,path, n):
        with open(path, 'w') as file:
            file.write('x,y\n')
            for i in range(n):
                landmarks = fastslam.get_predicted_landmarks(i)
                for lm in landmarks:
                    file.write(f'{lm.pos()[0]},{lm.pos()[1]}\n')

        
localization_deco = Delayer(15)

class ManualSimulation(BaseSimulation):
    # for fastSlam
    mov_state = None # True: moving, False: turning, None: deplacement not started
    move_counter = 0
    ml = None
    def __init__(self):
        super().__init__()
        self.sample_robot = Robot(Interface.robot.get_pos(), Interface.robot.orien, display_border=False)
        # store position and angle of before deplacement
        self.history_state = {'pos':self.robot.get_pos(), 'angle':self.robot.orien}
    
    @localization_deco
    def localization_event(self, pressed):
        if pressed[pygame.K_m]:
            self.localization()
            self.history_state['pos'] = self.robot.get_pos() # store position of the robot
            self.history_state['angle']= self.robot.orien
            return True
        return False

    def react_events(self, pressed):
        if pressed[pygame.K_w] or pressed[pygame.K_s]:
            if self.mov_state == False: # was turning
                self.localization()
                self.history_state['pos'] = self.robot.get_pos() # store position of the robot
                self.history_state['angle']= self.robot.orien

            self.mov_state = True

            if pressed[pygame.K_w]:
                self.robot.move(10)
            elif pressed[pygame.K_s]:
                self.robot.move(-10)

        if pressed[pygame.K_a] or pressed[pygame.K_d]:
            if self.mov_state == True: # was moving
                self.localization()
                self.history_state['pos'] = self.robot.get_pos() # store position of the robot
                self.history_state['angle']= self.robot.orien

            self.mov_state = False

            if pressed[pygame.K_a]:
                self.robot.set_orien(self.robot.orien - 0.1)
            elif pressed[pygame.K_d]:
                self.robot.set_orien(self.robot.orien + 0.1)
    
        self.localization_event(pressed)

    @timer
    def localization(self):
        #self.ml.run()
        # compute displacement of the robot with the pos_history
        dis = euclidean_distance(self.history_state['pos'], self.robot.get_pos())
        angle = self.robot.orien - self.history_state['angle']
        mov = [dis, angle]

        cols = self.collision()
        obs = []
        for col in cols:
            if col:
                # get obs and movement of the robot
                dis = euclidean_distance(self.robot.get_pos(), col)
                angle = sense_direction(self.robot.get_pos(), col, 0.0)
                obs.append([dis, angle])

        obs = np.array(obs)
        
        # execute fastslam
        self.fastslam(mov,obs)

        # update dps: landmarks, particles

        landmarks = self.get_landmarks_dps(self.fastslam)

        Interface.add_dps(landmarks, C.BORDEAU)
        
        particles = self.get_particles_dps(self.fastslam)

        Interface.add_dps(particles, C.WHITE)
        

    def __call__(self, pressed):
        
        self.react_events(pressed)

        # update sample robot position
        self.sample_robot.set_pos(self.fastslam.get_mean_pos(), center=True, scale=True)

        self.sample_robot.display()

        lm2 = self.get_landmarks_dps(self.fastslam, 1)

        Interface.add_dps(lm2, C.PURPLE)
        
    def store(self):
        self.store_landmarks(self.fastslam, os.path.join('data','landmarks.csv'), len(self.particles))


POS_START = (800,800)

class MLSimulation(BaseSimulation):
    def __init__(self, model, plan=None, graphics=False):

        # if graphics are enabled, link to Interface.robot to update his pos
        # if plan is not specified, take Interface's plan
        super().__init__(with_robot=graphics, plan=plan) 

        self.model = model

        # set model plan -> training
        if plan:
            self.model.set_plan(plan)
        else:
            self.model.set_plan(Interface.plan)

        self.as_graphics = graphics

        # if graphics -> set robot specification
        if self.as_graphics:
            self.pos = self.robot.get_pos()
            self.orien = self.robot.orien
        else:
            self.pos = POS_START.copy()
            self.orien = 0

    def move(self, distance):
        x = math.cos(self.orien) * distance + self.pos[0]
        y = math.sin(self.orien) * distance + self.pos[1]
        self.pos = (x,y)

    @property
    def running(self):
        return self.model.running

    def run(self):
        # get sensors detections
        try:
            collisions = self.collision(orientation=self.orien, position=self.pos)
        except:
            collisions = []
            print('collision error')
            print('pos:',self.pos[0], self.pos[1], 'angle', self.orien)
        
        angle, distance = self.model.run(self.pos, collisions)

        # execute order
        self.orien = angle
        self.move(distance)

        if self.as_graphics:
            # update robot object position
            self.robot.set_pos(self.pos, center=True, scale=True)
            self.robot.set_orien(self.orien)
            
            # add landmarks
            Interface.add_dps(collisions, C.BORDEAU, is_permanent=True)
            print(f'turn {angle:.1f}, move {distance:.1f}')

        
    