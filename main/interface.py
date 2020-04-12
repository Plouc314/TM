import pygame
from math import sin, cos, pi
from pgenerator2 import Generator
from geometry import middle
import os, sys
from copy import deepcopy

pygame.init()

class Dimension:
    plan, original = None, None
    dp_middle = [0,0]
    scale_factor = 1.
    fastslam = None
    def __init__(self,x,y):
        self.window = (x,y)
        self.sensor_scope = y // 4
        self.sensor_marge = y // 40
        self.sensor_width = y // 320
        self.wall = y // 160
        self.robot = [y // 20, y // 20]
        self.unit = y//16

        # list of all the objects that can be rescaled
        self.objects = []
    
    def set_plan(self, plan):
        self.plan = plan
        #self.original = deepcopy(plan)

    def change_scale(self, coef):
        self.scale_factor *= coef

        self.sensor_scope = self.sensor_scope * coef
        self.sensor_marge = self.sensor_marge * coef
        self.sensor_width = self.sensor_width * coef
        self.wall = self.wall * coef
        self.robot[0] = self.robot[0] * coef
        self.robot[1] = self.robot[1] * coef
        self.unit = self.unit * coef

        # base of all the new values
        self.dp_middle = middle(self.plan)
        for obj in self.objects:
            obj.change_scale(coef)
        #for p in self.fastslam.particles:
        #    for lm in p.landmarks:
        #        x,y = lm.pos()
        #        newx, newy = coef * x, coef * y
        #        newx += dp_middle[0] * (1-coef)
        #        newy += dp_middle[1] * (1-coef)
        #        lm.pos_x, lm.pos_y = newx, newy
        self.scale_plan(coef)

    def scale_plan(self,coef):
        
        for dp in self.plan:
            dp[0], dp[1] = dp[0] * coef, dp[1] * coef
            dp[0] += self.dp_middle[0] * (1 - coef)
            dp[1] += self.dp_middle[1] * (1 - coef)
        
dim = Dimension(1600,1600)
screen = pygame.display.set_mode(dim.window)
pygame.display.set_caption('Plan')

font = pygame.font.SysFont("Calibri", 50)
BROWN = (99, 39, 0)
WALL_BROWN = (244,164,96)
LIGHT_BROWN = (255, 169, 79)
WHITE = (255,255,255)
RED = (255,30,30)
BORDEAU = (228,50,50)
YELLOW = (255,255,100)

class Delayed:
    wait = 0
    delayed = False
    def __init__(self, delay):
        self.delay = delay
        
    def __call__(self, func):
        def inner(*args, **kwargs):
            if self.delayed:
                self.wait += 1
                if self.wait == self.delay:
                    self.delayed = False
                    self.wait = 0
            else:
                # first argument if a boolean value of if the tested key was pressed
                executed = func(*args, **kwargs)
                if executed:
                    self.delayed = True
                return executed
        return inner

# change plan state
plan_state_deco = Delayed(15)
change_scale_deco = Delayed(15)

class Dp(pygame.sprite.Sprite):
    def __init__(self,size, pos, color=BORDEAU):
        super(Dp, self).__init__()
        self.surf = pygame.Surface(size)
        self.surf.fill(color)
        self.pos = list(pos)
        self.scale_pos(dim.dp_middle, dim.scale_factor)

    def scale_pos(self, middle, factor):
        self.pos[0] = self.pos[0] * factor + middle[0] * (1 - factor)
        self.pos[1] = self.pos[1] * factor + middle[1] * (1 - factor)

class Robot:
    # load the image of the robot
    img = pygame.image.load('robotic.png')
    img = pygame.transform.scale(img, dim.robot)

    def __init__(self, pos, alpha, sensors):
        self.pos_x = pos[0]
        self.pos_y = pos[1]
        self.alpha = alpha
        self.sensors = sensors

        # add the new object to the list of scalable objects
        dim.objects.append(self)

    @property
    def pos(self):
        return [self.pos_x, self.pos_y]

    @pos.setter
    def pos(self, value):
        self.pos_x, self.pos_y = value

    def display(self, screen):
        # display orientation line
        self.orientation()
        # display robot surface
        screen.blit(self.img, self.pos)

    # vecteur d'oritentation du robot
    def orientation(self):
        for sensor in self.sensors:
            sensor.orientation(self.pos_x, self.pos_y, self.alpha)
        
        if self.alpha > 2 * pi:
            self.alpha -= 2 * pi

    def move(self,length):
        self.pos_x = cos(self.alpha)*length + self.pos_x
        self.pos_y = sin(self.alpha)*length + self.pos_y
    
    def change_scale(self, coef):
        self.img = pygame.image.load('robotic.png')
        self.img = pygame.transform.scale(self.img, [int(dim.robot[0]), int(dim.robot[1])])



class Sensor:
    detect = False
    # deviation: l'angle par rapport à l'avant du robot
    def __init__(self, deviation):
        self.scope = dim.sensor_scope
        self.deviation = deviation
        self.MARGE = dim.sensor_marge
        self.WIDTH = dim.sensor_width

        # add the new object to the list of scalable objects
        dim.objects.append(self)

    def orientation(self, pos_x, pos_y, alpha):
        x,y = self.dp_scope(pos_x, pos_y, alpha)
        pygame.draw.line(screen, LIGHT_BROWN, (pos_x + self.MARGE, pos_y + self.MARGE), (x,y), self.WIDTH)
    
    def dp_scope(self, pos_x, pos_y, alpha):
        x = cos(alpha + self.deviation)*self.scope + pos_x + self.MARGE
        y = sin(alpha + self.deviation)*self.scope + pos_y + self.MARGE
        return [x,y]

    def change_scale(self, coef):
        self.scope = int(dim.sensor_scope)
        self.MARGE = int(dim.sensor_marge)
        self.WIDTH = int(dim.sensor_width)

    def __str__(self):
        return f'Sensor: angle {self.deviation}'

class Interface:
    '''
    Base of the interface

    Update the screen,
    Manage the dimension,unit/dim.scale
    Basic event reaction,
    Can keep track of the movements,
    In simulation, display the plan
    '''
    # list of all the object that can be rescaled
    objects = []
    running = True
    as_robot = False
    # variables for plan display/scale
    as_plan = False
    display = True
    lines_plan = None
    # dps of all the position of the robot
    movement_dps = []
    keep_track = False
    dps_color = None
    auto = False

    def __init__(self, robot=None):
        self.dim = dim
        self.screen = screen
        if robot != None:
            self.robot = robot
            self.as_robot = True
        self.clock = pygame.time.Clock()
    
    def set_robot(self, robot):
        self.robot = robot
        self.as_robot = True

    def display_scale(self, pos):
        text = '{:.1f}'.format(dim.scale_factor)
        text_scale = font.render(text,True,(0,0,0))
        self.screen.blit(text_scale,pos)

        pygame.draw.line(self.screen, LIGHT_BROWN, (pos[0], pos[1]-20), (pos[0]+int(dim.unit), pos[1]-20), int(self.dim.wall))

    def react_events(self, pressed):
        for event in pygame.event.get():
            # check quit
            if event.type == pygame.QUIT:
                self.running = False
        
        # stop the program
        if pressed[pygame.K_ESCAPE]:
            self.running = False
        
        self.change_scale(pressed)


        if self.as_plan:
            # change the state of the plan (displayed/not)
            self.change_plan_state(pressed)

    @plan_state_deco # create a delay to don't check 30 times a second
    def change_plan_state(self, pressed):
        if pressed[pygame.K_RETURN]:
            self.display = not self.display
            return True
        return False

    @change_scale_deco
    def change_scale(self, pressed):
        if pressed[pygame.K_i] and pressed[pygame.K_LCTRL]:
            self.dim.change_scale(1.5)
            return True
        elif pressed[pygame.K_o] and pressed[pygame.K_LCTRL]:
            self.dim.change_scale(2/3)
            return True
        return False

    def display_plan(self):
        # if has to, display the plan
        if self.display:
            for line in self.lines_plan:
                pygame.draw.line(self.screen, WALL_BROWN, line[0], line[1], 10)
    
    def set_plan(self, plan):
        self.as_plan = True
        self.dim.set_plan(plan)
        self.lines_plan = []
        data_points = plan.copy()
        data_points.append(data_points[0])
        for i in range(len(data_points) - 1):
            self.lines_plan.append([data_points[i], data_points[i+1]])

    def keep_track_mov(self, color, auto=False):
        '''
        Keep track of the robot movements and display the movements on the screen

        auto: if True, the position is updated at each frame in the run() method,
        else: to get a new position use the pos setter (self.pos=...)
        '''
        if self.as_robot:
            self.keep_track = True
            self.auto = auto
            self.dps_color = color
            # store depart position
            self.pos = self.robot.pos
        else:
            raise AttributeError('must have a robot assigned')

    @property
    def pos(self):
        return self.robot.pos
    
    @pos.setter
    def pos(self, new_pos):
        ''' Add the new position to the stored ones and update the robot position '''
        new_dp = Dp((10,10), new_pos, self.dps_color)
        self.movement_dps.append(new_dp)
        self.robot.pos_x = new_pos[0]
        self.robot.pos_y = new_pos[1]

    def display_lines(self):
        for i in range(len(self.movement_dps)-1):
            pygame.draw.line(screen,self.dps_color, self.movement_dps[i].pos, self.movement_dps[i+1].pos, 5)


    def run(self):
        '''
        Display the screen, robot, sensors, scale and get the pressed key
        '''
        self.clock.tick(30)
        pygame.display.update()
        self.screen.fill(BROWN)
        self.display_scale((1400,1500))

        if self.as_robot:
            self.robot.display(self.screen)

        if self.as_plan:
            self.display_plan()

        if self.keep_track:
            if self.auto:
                self.pos = self.robot.pos
            self.display_lines()

        pressed = pygame.key.get_pressed()
        
        self.react_events(pressed)
        return pressed
                    

    

if __name__ == '__main__':
    
    robot = Robot((600,500),0,[Sensor(0), Sensor(1/3*pi), Sensor(-1/3*pi)])
    inter = Interface()

    g = Generator(dim.window)
    g.load_data()

    inter.set_plan(g.data[3].copy())

    from simul import ManualSimulation

    simulation = ManualSimulation(screen, dim.window, inter.dim.plan, robot)

    inter.set_robot(simulation.sample_robot)
    inter.keep_track_mov(YELLOW, auto=True)
    inter.dim.fastslam = simulation.fastslam

    while inter.running:
        pressed = inter.run()
        simulation(pressed)

    simulation.store()