import pygame
from math import sin, cos, pi
from pgenerator2 import Generator
from geometry import middle
import os, sys
from copy import deepcopy

pygame.init()

class Dimension:
    plan, original = None, None
    def __init__(self,x,y):
        self.window = (x,y)
        self.sensor_scope = y // 4
        self.sensor_marge = y // 40
        self.sensor_width = y // 320
        self.wall = y // 160
        self.robot = (y // 20, y // 20)
        self.unit = y//16
        self.scale = 1

        # list of all the objects that can be rescaled
        self.objects = []
    
    def set_plan(self, plan):
        self.plan = plan
        self.original = deepcopy(plan)

    def change_scale(self):
        for obj in self.objects:
            obj.change_scale(self.scale)
        self.scale_plan(self.scale)

    def scale_plan(self,coef):
        dp_middle = middle(self.original)
        
        for sdp, odp in zip(self.plan, self.original):
            sdp[0], sdp[1] = int(odp[0] * coef), int(odp[1] * coef)
            sdp[0] += int(dp_middle[0] * (1 - coef))
            sdp[1] += int(dp_middle[1] * (1 - coef))
        
dim = Dimension(1600,1600)
screen = pygame.display.set_mode(dim.window)
pygame.display.set_caption('Plan')

font = pygame.font.SysFont("Calibri", 50)
BROWN = (99, 39, 0)
WALL_BROWN = (244,164,96)
LIGHT_BROWN = (255, 169, 79)
WHITE = (255,255,255)
RED = (255,30,30)


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
        self.img = pygame.transform.scale(self.img, [int(dim.robot[0] * dim.scale), int(dim.robot[1] * dim.scale)])



class Sensor:
    detect = False
    dp_scope = [0,0]
    # deviation: l'angle par rapport à l'avant du robot
    def __init__(self, deviation):
        self.scope = dim.sensor_scope
        self.deviation = deviation
        self.MARGE = dim.sensor_marge
        self.WIDTH = dim.sensor_width

        # add the new object to the list of scalable objects
        dim.objects.append(self)

    def orientation(self, pos_x, pos_y, alpha):
        x = cos(alpha + self.deviation)*self.scope + pos_x + self.MARGE
        y = sin(alpha + self.deviation)*self.scope + pos_y + self.MARGE
        self.dp_scope = (x,y)
        pygame.draw.line(screen, LIGHT_BROWN, (pos_x + self.MARGE, pos_y + self.MARGE), (x,y), self.WIDTH)
    
    def change_scale(self, coef):
        self.scope = int(dim.sensor_scope * coef)
        self.MARGE = int(dim.sensor_marge * coef)
        self.WIDTH = int(dim.sensor_width * coef)


class Interface:
    '''
    Base of the interface

    Update the screen,
    Manage the dimension,
    Basic event reaction,
    In simulation, display the plan
    '''
    # list of all the object that can be rescaled
    objects = []
    running = True
    # variables for plan display/scale
    as_plan = False
    display = True
    delayed, delay = False, 0
    lines_plan = None

    def __init__(self, robot):
        self.dim = dim
        self.screen = screen
        self.robot = robot
        self.clock = pygame.time.Clock()
    
    def display_scale(self, pos):
        text = '{:.1f}'.format(dim.unit/dim.scale)
        text_scale = font.render(text,True,(0,0,0))
        self.screen.blit(text_scale,pos)

        pygame.draw.line(self.screen, LIGHT_BROWN, (pos[0], pos[1]-20), (pos[0]+dim.unit, pos[1]-20), self.dim.wall)

    def react_events(self, pressed):
        for event in pygame.event.get():
            # check quit
            if event.type == pygame.QUIT:
                self.running = False
        
        # stop the program
        if pressed[pygame.K_ESCAPE]:
            self.running = False
        elif pressed[pygame.K_i]and pressed[pygame.K_LCTRL]:
            self.dim.scale += 0.01
            self.dim.change_scale()
        elif pressed[pygame.K_o] and pressed[pygame.K_LCTRL]:
            if self.dim.scale > 0.01:
                self.dim.scale -= 0.01
                self.dim.change_scale()
        
        if self.as_plan:
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

    def display_plan(self):
        # increase delay 
        if self.delayed:
            self.delay += 1
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

    def run(self):
        '''
        Display the screen, robot, sensors, scale and get the pressed key
        '''
        self.clock.tick(30)
        pygame.display.update()
        self.screen.fill(BROWN)
        self.display_scale((1400,1500))

        if self.as_plan:
            self.display_plan()

        pressed = pygame.key.get_pressed()
        
        self.react_events(pressed)
        return pressed
                    

    

if __name__ == '__main__':
    
    robot = Robot((600,500),0,[Sensor(0), Sensor(-pi)])
    inter = Interface(robot)

    g = Generator(dim.window)
    g.load_data()

    inter.set_plan(g.data[13].copy())

    from simul import ManualSimulation

    simulation = ManualSimulation(screen, dim.window, inter.dim.plan, robot)

    while inter.running:
        pressed = inter.run()
        simulation(pressed)
