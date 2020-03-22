import pygame
from math import sin, cos
from time import sleep, time
from pgenerator2 import Generator
from geometry import middle
import os, sys

pygame.init()

start_time = time()

class Dimension:
    def __init__(self,x,y):
        self.window = (x,y)
        self.sensor_scope = y // 8
        self.robot = (y // 20, y // 20)
        self.unit = y//16
        self.scale = 1
    
    def change_scale(self):
        for obj in p.objects:
            obj.change_scale(self.scale)
        self.scale_plan(self.scale)

    def scale_plan(self, coef):
        dp_middle = middle(p.original)
        
        for sdp, odp in zip(p.plan, p.original):
            sdp[0], sdp[1] = int(odp[0] * coef), int(odp[1] * coef)
            sdp[0] += int(dp_middle[0] * (1 - coef))
            sdp[1] += int(dp_middle[1] * (1 - coef))
        

class Param:
    # list of all the object that can be rescaled
    objects = []
    running = True
    plan = None
    original = None

p = Param()

# object dont dépend toutes les grandeurs
dim = Dimension(1600,1600)

screen = pygame.display.set_mode(dim.window)
pygame.display.set_caption('Plan')


font = pygame.font.SysFont("Calibri", 50)
BROWN = (99, 39, 0)
LIGHT_BROWN = (255, 169, 79)
WHITE = (255,255,255)
RED = (255,30,30)
PI = 3.14


class Robot:
    # télécharge l'image du robot
    img = pygame.image.load('robotic.png')
    img = pygame.transform.scale(img, dim.robot)

    def __init__(self, pos_x, pos_y, alpha):
        
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.alpha = alpha
        self.sensor1 = Sensor(0)
        self.sensor2 = Sensor(0.3)
        self.sensor3 = Sensor(-0.3)
        self.sensors = [self.sensor1, self.sensor2, self.sensor3]

        # add the new object to the list of scalable objects
        p.objects.append(self)

    @property
    def pos(self):
        return [self.pos_x, self.pos_y]

    # vecteur d'oritentation du robot
    def orientation(self):
        self.sensor1.orientation(self.pos_x, self.pos_y, self.alpha)
        self.sensor2.orientation(self.pos_x, self.pos_y, self.alpha)
        self.sensor3.orientation(self.pos_x, self.pos_y, self.alpha)
        
        if self.alpha > 2 * PI:
            self.alpha -= 2 * PI

    def move(self,length):
        self.pos_x = cos(self.alpha)*length + self.pos_x
        self.pos_y = sin(self.alpha)*length + self.pos_y
    
    def change_scale(self, coef):
        self.img = pygame.image.load('robotic.png')
        self.img = pygame.transform.scale(self.img, [int(dim.robot[0] * dim.scale), int(dim.robot[1] * dim.scale)])



class Sensor:
    detect = False
    # deviation: l'angle par rapport à l'avant du robot
    def __init__(self, deviation):
        self.scope = dim.sensor_scope
        self.deviation = deviation
        self.MARGE = dim.robot[0] / 2
        self.WIDTH = dim.robot[0] // 8
        self.dp_scope = [0,0]

        # add the new object to the list of scalable objects
        p.objects.append(self)

    def orientation(self, pos_x, pos_y, alpha):
        x = cos(alpha + self.deviation)*self.scope + pos_x + self.MARGE
        y = sin(alpha + self.deviation)*self.scope + pos_y + self.MARGE
        self.dp_scope = [x,y]
        pygame.draw.line(screen, LIGHT_BROWN, (pos_x + self.MARGE, pos_y + self.MARGE), (x,y), self.WIDTH)
    
    def change_scale(self, coef):
        self.scope = int(dim.sensor_scope * coef)
        self.MARGE = int(dim.robot[0] / 2 * coef)
        self.WIDTH = int(dim.robot[0] // 8 * coef)

def display_scale(pos):
    text = '{:.1f}'.format(dim.unit/dim.scale)
    text_scale = font.render(text,True,(0,0,0))
    screen.blit(text_scale,pos)

    pygame.draw.line(screen, LIGHT_BROWN, (pos[0], pos[1]-20), (pos[0]+dim.unit, pos[1]-20), 10)

def react_events():
    for event in pygame.event.get():
        # check quit
        if event.type == pygame.QUIT:
            p.running = False
    
    # stop the program
    if pressed[pygame.K_ESCAPE]:
        p.running = False
    elif pressed[pygame.K_i]and pressed[pygame.K_LCTRL]:
        dim.scale += 0.01
        dim.change_scale()
    elif pressed[pygame.K_o] and pressed[pygame.K_LCTRL]:
        if dim.scale > 0.01:
            dim.scale -= 0.01
            dim.change_scale()


g = Generator(dim.window)
g.load_data()

from simul import ManualSimulation

p.plan = g.data[0].copy()
# get a copy of p.plan
p.original = []
for dp in p.plan:
    p.original.append(dp.copy())

robot = Robot(600,500,0)
simulation = ManualSimulation(screen, dim.window, p.plan, robot)


clock = pygame.time.Clock()

print('START')




while p.running:
    screen.fill(BROWN)
    display_scale((1400,1500))

    pressed = pygame.key.get_pressed()
    
    react_events()

    simulation(pressed)
            
    clock.tick(30)
    pygame.display.update()

print('Exit program in {:.3f}'.format(time() - start_time))

pygame.quit()
sys.exit()