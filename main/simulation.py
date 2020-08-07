import pygame
import math, random, pickle, os
import numpy as np
from geometry import segment_intersect
from fastSLAM.fast_slam import FastSlam
from fastSLAM.slam_helper import euclidean_distance, sense_direction, timer
from interface import Interface, Delayer, C, Robot
from specifications import Specifications as Spec

class BaseSimulation:
    
    sensor_scope = Spec.SENSOR_SCOPE
    noise_scale = 10

    robot = None
    landmarks, particles = [], []
    def __init__(self, with_robot=True, plan=None, with_fastslam=True):
        
        if with_robot:
            self.robot = Interface.robot

        if plan:
            self.plan = plan
            self.plan_lines = Interface.create_plan_lines(plan)
        else:
            self.plan = Interface.plan
            self.plan_lines = Interface.plan_lines

        if with_fastslam:
            # create fastslam object
            self.fastslam = FastSlam(Interface.robot.x, Interface.robot.y, Interface.robot.orien, particle_size=50)

        # create 20 angles to simulated the rotating sensor
        self.angles = list(np.linspace(Spec.SENSOR_ANGLES[0], Spec.SENSOR_ANGLES[1], 20))

    def collision(self, position=None, orientation=None):
        '''In the case where the simulation doesn't have robot object, specify orien/pos in arguments'''
        
        if position == None:
            pos = self.robot.get_pos()
        else:
            pos = position
        
        if orientation == None:
            orien = self.robot.orien
        else:
            orien = orientation

        cols = []
        for line0 in self.plan_lines:
            for angle in self.angles:
                # get the end of the sensor's line
                x = math.cos(orien + angle) * self.sensor_scope + pos[0]
                y = math.sin(orien + angle) * self.sensor_scope + pos[1]
                
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
                    # add noise to measures
                    noise = self.get_noise(self.noise_scale, 2)
                    intersect = [ c + noise[i] for i, c in enumerate(intersect)]
                    cols.append(intersect)

        return cols

    @staticmethod
    def get_noise(scale, size):
        random.seed(random.randint(0,100))
        # create noise of desired size
        noise = [ random.randint(0, scale) - scale//2 for i in range(size)]
        return noise

        
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
        
        Interface.reset_dps()
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

        landmarks = self.fastslam.get_landmarks_dps()

        Interface.add_dps(landmarks, C.BORDEAU)
        
        particles = self.fastslam.get_particles_dps()

        Interface.add_dps(particles, C.WHITE)
        

    def __call__(self, pressed):
        
        self.react_events(pressed)

        # update sample robot position
        self.sample_robot.set_pos(self.fastslam.get_mean_pos(), center=True, scale=True)

        self.sample_robot.display()

        lm2 = self.fastslam.get_landmarks_dps(1)

        Interface.add_dps(lm2, C.PURPLE)
        
    def store(self):
        self.fastslam.store_landmarks()


class MLSimulation(BaseSimulation):
    def __init__(self, model, position=None, plan=None, graphics=False):

        # if graphics are enabled, link to Interface.robot to update his position
        # if plan is not specified, take Interface's plan
        super().__init__(with_robot=graphics, plan=plan, with_fastslam=False) 

        self.started = False
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
            self.pos = position
            self.orien = 0
        

    def move(self, distance):
        x = math.cos(self.orien) * distance + self.pos[0]
        y = math.sin(self.orien) * distance + self.pos[1]
        self.pos = (x,y)

    @property
    def running(self):
        return self.model.running

    def turn_around(self):
        '''Get the collisions equvalent of a 360° of the robot'''
        collisions = []
        for  i in range(4):
            angle = i * math.pi/2
            current_colisions = self.collision(orientation=angle, position=self.pos)
            collisions.extend(current_colisions)
        return collisions

    def run(self):
        # get sensors detections
        if self.started:
            collisions = self.collision(orientation=self.orien, position=self.pos)
        else:
            self.started = True
            collisions = self.turn_around()

        angle, distance = self.model.run(self.pos, collisions)

        # execute order
        self.orien = angle
        self.move(distance)

        if self.as_graphics:
            # update robot object position
            self.robot.set_pos(self.pos, center=True, scale=True)
            self.robot.set_orien(self.orien)

            # add view of inputs
            view_dir = self.model.view_dir
            view_pos = self.model.view_pos
            
            Interface.info_board.hist_pos.set_surf(view_pos)
            Interface.info_board.directions.set_surf(view_dir)

            # add landmarks
            Interface.add_dps(collisions, C.BORDEAU)
            #print(f'turn {angle:.1f}, move {distance:.1f}')
    