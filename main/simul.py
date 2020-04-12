import pygame
from geometry import segment_intersect
import random
from copy import deepcopy
from fastSLAM.fast_slam import FastSlam
from fastSLAM.slam_helper import euclidean_distance, sense_direction, timer
import numpy as np
from interface import Dp, Sensor

BORDEAU = (228,50,50)

class BaseSimulation:
    landmarks, particles = [], []
    def __init__(self, screen, dim_screen, plan, robot):
        self.screen = screen
        self.dim_sreen = dim_screen
        self.robot = robot
        self.plan = plan
        # create fastslam object
        self.fastslam = FastSlam(robot.pos_x, robot.pos_y, robot.alpha, particle_size=50)
        self.create_lines()
    
    def create_lines(self):
        self.lines_p = []
        data_points = self.plan.copy()
        data_points.append(data_points[0])
        for i in range(len(data_points) - 1):
            self.lines_p.append([data_points[i], data_points[i+1]])

    def collision(self, sensors):
        cols = []
        for line in self.lines_p:
            for sensor in sensors:
                # update sensor position
                dp_scope = sensor.dp_scope(self.robot.pos_x, self.robot.pos_y, self.robot.alpha)

                line_1 = [self.robot.pos, dp_scope]
                # test for vertical lines
                try:
                    intersect = segment_intersect(line, line_1)
                except:
                    # if a line is vertical m of mx + h is inf, it's produce an error in geometry.slope
                    # so swap the coor to get an horizontal line
                    inv_line = [dp[::-1] for dp in line]
                    inv_line_1 = [dp[::-1] for dp in line_1]
                    intersect = segment_intersect(inv_line, inv_line_1)
                    if intersect != None:
                        intersect = intersect[::-1]
                if intersect != None:
                    noise = [0,0]#self.get_noise(20, 2)
                    intersect = [ c + noise[i] for i, c in enumerate(intersect)]
                    cols.append(intersect)
        return cols
    
    @staticmethod
    def display_dps(screen, dps):
        for dp in dps:
            screen.blit(dp.surf, (int(dp.pos[0]), int(dp.pos[1])))
    
    @staticmethod
    def get_particles(fastslam):
        particles = fastslam.particles
        particles = [particles[i].pos() for i in range(len(particles))]
        particles = [Dp([10,10], p, (255,255,255)) for p in particles]
        return particles

    @staticmethod
    def get_landmarks(fastslam, index=0, color=BORDEAU):
        landmarks = fastslam.get_predicted_landmarks(index)
        list_landmarks = []
        for lm in landmarks:
            dp = Dp([10,10], lm.pos(), color)
            list_landmarks.append(dp)
        return list_landmarks

    @staticmethod
    def store_landmarks(fastslam,path, n):
        with open(path, 'w') as file:
            file.write('x,y\n')
            for i in range(n):
                landmarks = fastslam.get_predicted_landmarks(i)
                for lm in landmarks:
                    file.write(f'{lm.pos()[0]},{lm.pos()[1]}\n')

        

class ManualSimulation(BaseSimulation):
    # for fastSlam
    mov_state = None # True: moving, False: turning, None: deplacement not started
    move_counter = 0
    def __init__(self, screen, dim_screen, plan, robot):
        super().__init__(screen, dim_screen, plan, robot)
        self.sample_robot = deepcopy(robot)
        # get rid of the sensor to don't display them
        self.sample_robot.sensors = []
        self.sample_robot.img = pygame.transform.scale(self.sample_robot.img, (dim_screen[0]//40, dim_screen[0]//40))
        
        # store position and angle of before deplacement
        self.history_state = {'pos':self.robot.pos.copy(), 'angle':self.robot.alpha}

        # create sensors that represent the totating sensor
        angles = list(np.linspace(-1/3*np.pi, 1/3*np.pi, 20))
        self.sensors = [Sensor(angle) for angle in angles]
    
    def react_events(self, pressed):
        if pressed[pygame.K_w] or pressed[pygame.K_s]:
            if self.mov_state == False: # was turning
                self.localization()
                self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
                self.history_state['angle']= self.robot.alpha

            self.mov_state = True

            if pressed[pygame.K_w]:
                self.robot.move(10)
            elif pressed[pygame.K_s]:
                self.robot.move(-10)

        if pressed[pygame.K_a] or pressed[pygame.K_d]:
            if self.mov_state == True: # was moving
                self.localization()
                self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
                self.history_state['angle']= self.robot.alpha

            self.mov_state = False

            if pressed[pygame.K_a]:
                self.robot.alpha -= 0.1
            elif pressed[pygame.K_d]:
                self.robot.alpha += 0.1
        
        if pressed[pygame.K_m]:
            self.localization()
            self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
            self.history_state['angle']= self.robot.alpha

    @staticmethod
    def get_noise(scale, shape):
        random.seed(random.randint(0,100))
        # create noise of desired shape
        noise = [ int(random.randint(0, scale) - scale/2) for i in range(shape)]
        return noise

    @timer
    def localization(self):
        # compute displacement of the robot with the pos_history
        dis = euclidean_distance(self.history_state['pos'], self.robot.pos)
        angle = self.robot.alpha - self.history_state['angle']
        mov = [dis, angle]

        cols = self.collision(self.sensors)
        obs = []
        for col in cols:
            if col:
                # get obs and movement of the robot
                dis = euclidean_distance(self.robot.pos, col)
                angle = sense_direction(self.robot.pos, col, 0.0)
                obs.append([dis, angle])

        obs = np.array(obs)
        print(obs)
        # execute fastslam
        self.fastslam(mov,obs)

        # get landmarks - update dps
        self.landmarks = self.get_landmarks(self.fastslam)
        
        self.particles = self.get_particles(self.fastslam)

    def __call__(self, pressed):
        
        self.react_events(pressed)

        self.robot.display(self.screen)

        # update sample robot position
        self.sample_robot.pos = self.fastslam.get_mean_pos()


        lm2 = self.get_landmarks(self.fastslam, 1, (50,50,100))
        self.display_dps(self.screen, lm2)

        self.display_dps(self.screen, self.landmarks)
        self.display_dps(self.screen, self.particles)
    
    def store(self):
        self.store_landmarks(self.fastslam, 'landmarks.csv', len(self.particles))

