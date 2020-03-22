import pygame
from geometry import segment_intersect
import random
from copy import deepcopy
from fastSLAM.fast_slam import FastSlam
from fastSLAM.slam_helper import euclidean_distance, sense_direction

BROWN = (244,164,96)
BORDEAU = (128,0,0)

class Dp(pygame.sprite.Sprite):
    def __init__(self,dim, pos, color=BORDEAU):
        super(Dp, self).__init__()
        self.surf = pygame.Surface(dim)
        self.surf.fill(color)
        self.pos = pos


class ManualSimulation:
    landmarks, particles = [], []
    display = True
    delayed, delay = False, 0
    # for fastSlam
    mov_state = None # True: moving, False: turning, None: deplacement not started
    move_counter = 0
    def __init__(self, screen, dim_screen, plan, robot):
        self.screen = screen
        self.dim_sreen = dim_screen
        self.plan = plan
        self.robot = robot

        self.sample_robot = deepcopy(robot)
        self.sample_robot.img = pygame.transform.scale(self.sample_robot.img, (dim_screen[0]//40, dim_screen[0]//40))
        
        # store position and angle of before deplacement
        self.history_state = {'pos':self.robot.pos.copy(), 'angle':self.robot.alpha}
        # create fastslam object
        self.fastslam = FastSlam(robot.pos_x, robot.pos_y, robot.alpha, particle_size=50)
        self.create_lines()
    
    def create_lines(self):
        self.lines_p = []
        data_points = self.plan.copy()
        data_points.append(data_points[0])
        for i in range(len(data_points) - 1):
            self.lines_p.append([data_points[i], data_points[i+1]])

    def display_plan(self):
        for line in self.lines_p:
            pygame.draw.line(self.screen, BROWN, line[0], line[1], 10)
    
    def react_events(self, pressed):
        
        if pressed[pygame.K_w] or pressed[pygame.K_s]:
            if self.mov_state == False: # was turning, so update localization
                self.localization()
                self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
                self.history_state['angle']= self.robot.alpha
                self.move_counter = 0 # reset counter
            self.mov_state = True
            self.move_counter += 1
            if pressed[pygame.K_w]:
                self.robot.move(10)
            elif pressed[pygame.K_s]:
                self.robot.move(-10)

        if pressed[pygame.K_a] or pressed[pygame.K_d]:
            if self.mov_state == True: # was moving, so update localization
                self.localization()
                self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
                self.history_state['angle']= self.robot.alpha
                self.move_counter = 0 # reset counter
            self.mov_state = False
            self.move_counter += 0.5
            if pressed[pygame.K_a]:
                self.robot.alpha -= 0.1
            elif pressed[pygame.K_d]:
                self.robot.alpha += 0.1
        
        # change the state of the plan (displayed/not)
        if pressed[pygame.K_RETURN]:
            # create a delay to don't check 30 times a second
            if self.delayed:
                if self.delay > 15:
                    self.delayed = False
                    self.delay = 0
            else: 
                self.display = not self.display
                self.delayed = True
    
    def collision(self):
        cols = []
        for line in self.lines_p:
            for sensor in self.robot.sensors:
                line_1 = [self.robot.pos, sensor.dp_scope]
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
    
    def display_dps(self, dps):
        for dp in dps:
            self.screen.blit(dp.surf, dp.pos)

    @staticmethod
    def get_noise(scale, shape):
        random.seed(random.randint(0,100))
        # create noise of desired shape
        noise = [ int(random.randint(0, scale) - scale/2) for i in range(shape)]
        return noise
    
    def convert_coordinates(self, pos):
        """Change the origin from bottom left to top left"""
        return (int(pos[0]), int(self.dim_sreen[0] - pos[1]))

    def localization(self):
        # compute displacement of the robot with the pos_history
        dis = euclidean_distance(self.history_state['pos'], self.robot.pos)
        angle = self.robot.alpha - self.history_state['angle']
        mov = [dis, angle]

        cols = self.collision()
        obs = []
        for col in cols:
            if col:
                # get obs and movement of the robot
                dis = euclidean_distance(self.robot.pos, col)
                angle = sense_direction(self.robot.pos, col, 0.0)
                obs.append([dis, angle])

        # execute fastslam
        self.fastslam(mov,obs)

        # get landmarks - update dps
        landmarks = self.fastslam.get_predicted_landmarks()
        self.landmarks = []
        for lm in landmarks:
            pos = lm.pos()
            dp = Dp([10,10], pos)
            self.landmarks.append(dp)
        
        particles = self.fastslam.particles
        self.particles = [particles[i].pos() for i in range(len(particles))]
        self.particles = [Dp([10,10], p, (255,255,255)) for p in self.particles]




    def __call__(self, pressed):
        
        self.react_events(pressed)
        if self.display:
            self.display_plan()

        # display orientation line
        self.robot.orientation()
        # display robot surface
        self.screen.blit(self.robot.img, self.robot.pos)

        # update sample robot position
        self.sample_robot.pos_x, self.sample_robot.pos_y = self.fastslam.get_mean_pos()
        # display robot surface
        self.screen.blit(self.sample_robot.img, self.sample_robot.pos)

        # localization - check once every 10 moves
        if self.move_counter == 5:
            self.localization()
            self.history_state['pos'] = self.robot.pos.copy() # store position of the robot
            self.history_state['angle']= self.robot.alpha
            self.move_counter = 0
            self.mov_state = None

        if self.delayed:
            self.delay += 1

        self.display_dps(self.landmarks)
        #self.display_dps(self.particles)

