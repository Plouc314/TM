import pygame
import os, sys
from math import sin, cos, pi
from geometry import pi_2_pi
from lib.interface import Interface as BaseInterface, Form, TextBox, Font, C as BaseC, Delayer, Cadre
from specifications import Specifications as Spec
        
class C(BaseC): # get more colors
    BROWN = (99, 39, 0)
    WALL_BROWN = (244,164,96)
    LIGHT_BROWN = (255, 169, 79)
    RED = (255,30,30)
    BORDEAU = (228,50,50)
    YELLOW = (255,255,100)


# change plan state
plan_state_deco = Delayer(15)
change_scale_deco = Delayer(15)

class ConstClass:
    dim = None
    values = {
        'TOTAL_WINDOW': (2400, 1600),
        'WALL': 10,
        'UNIT': 100,
        'DP': (10,10),
        'ROBOT': (100,100),
        'SMALL_WIDTH': 5,
    }

    def __getitem__(self, key):
        if key in self.values.keys():
            return self.values[key]
        elif key.upper() in self.values.keys():
            return self.dim.scale(self.values[key.upper()])

Const = ConstClass()

from fastSLAM.particle2 import Particle2

# set sensor specifications
Particle2.sensor_scope = Spec.SENSOR_SCOPE
Particle2.sensor_angles = Spec.SENSOR_ANGLES

class InfoBoard:
    def __init__(self):
        self.cadre = Cadre((800, 1600), (1600,0), C.LIGHT_BROWN)
        self.text_inputs = TextBox((400, 100), (1800,100), C.LIGHT_BROWN,"Inputs:",font=Font.f(75))
        self.text_directions = TextBox((250, 80), (1700,300), C.LIGHT_BROWN,"Directions",font=Font.f(30))
        self.text_history = TextBox((250, 80), (2050,300), C.LIGHT_BROWN,"Origine",font=Font.f(30))
        self.text_n_order = TextBox((400, 50), (1700,700), C.LIGHT_BROWN,"Exécutions:",font=Font.f(30), centered=False)
        self.text_max_obs = TextBox((400, 50), (1700,750), C.LIGHT_BROWN,"Max Observations:",font=Font.f(30), centered=False)
        self.text_fitness = TextBox((400, 50), (1700,800), C.LIGHT_BROWN,"Fonction d'évaluation:",font=Font.f(30), centered=False)
        self.text_running = TextBox((400, 50), (1700,850), C.LIGHT_BROWN,"Running state:",font=Font.f(30), centered=False)
        self.directions = Form((250, 250), (1700, 400))
        self.hist_pos = Form((250, 250), (2050, 400))
    
    def set_n_order(self, n):
        self.text_n_order.set_text(f"Exécutions: {n}")
    
    def set_max_obs(self, n):
        self.text_max_obs.set_text(f"Max Observations: {n}")

    def set_fitness(self, n):
        self.text_fitness.set_text(f"Fonction d'évaluation: {n:.1f}")
    
    def set_running(self, value):
        self.text_running.set_text(f'Running state: {value}')

    def display(self):
        self.cadre.display()
        self.text_inputs.display()
        self.text_directions.display()
        self.text_history.display()
        self.text_n_order.display()
        self.text_max_obs.display()
        self.text_fitness.display()
        self.text_running.display()
        self.hist_pos.display()
        self.directions.display()

class Interface(BaseInterface):
    '''
    Base of the interface  
    Update the screen,  
    Manage the dimension,  
    Basic event reaction,  
    Can keep track of the movements,  
    In simulation, display the plan  
    '''
    
    running = True
    robot = None
    
    # dps
    dps = []
    dps_data = []

    # plan
    plan = None
    plan_lines = None
    
    # keep track
    keep_track = False
    kt_last_pos = None
    kt_color = None
    kt_auto = False
    kt_dps = []
    
    # scalebar
    scalebar_pos = (1400, 1500)

    @classmethod
    def setup(cls, dimension, title, FPS=30, font_color=C.WHITE):
        super().setup(Const['TOTAL_WINDOW'], title, FPS=FPS, font_color=font_color)
        Const.dim = cls.dim
        cls.text_scalebar = TextBox((Const['UNIT'], 40), cls.scalebar_pos, text='1', color=C.BROWN, text_color=C.WHITE, font=Font.f(30))
        cls.info_board = InfoBoard()
        cls.is_plan_displayed = True

    @classmethod
    def set_robot(cls, robot):
        cls.robot = robot

    @classmethod
    def display_scalebar(cls):
        cls.text_scalebar.display()

        start = cls.dim.scale((cls.scalebar_pos[0], cls.scalebar_pos[1] - 20))
        end = cls.dim.scale((cls.scalebar_pos[0] + Const['UNIT'], cls.scalebar_pos[1] - 20))

        pygame.draw.line(cls.screen, C.LIGHT_BROWN, start, end, Const['wall'])

    @classmethod
    def react_events(cls, pressed):
        if cls.plan:
            # change the state of the plan (displayed/not)
            if pressed[pygame.K_RETURN]:
                cls.is_plan_displayed = not cls.is_plan_displayed

    @classmethod
    def display_plan(cls):
        # if has to, display the plan
        if cls.plan:
            for start, end in cls.plan_lines:
                # scale datapoints to current window size
                start = cls.dim.scale(start)
                end = cls.dim.scale(end)
                pygame.draw.line(cls.screen, C.WALL_BROWN, start, end, Const['wall'])

    @classmethod
    def set_plan(cls, plan):
        cls.plan = plan
        cls.plan_lines = cls.create_plan_lines(plan)

    @staticmethod
    def create_plan_lines(plan):
        dps = plan.copy()
        dps.append(dps[0])
        return [ [dps[i], dps[i+1]] for i in range(len(dps)-1) ]

    @classmethod
    def add_dps(cls, dps, color, is_permanent=True, scale=True):
        '''
        Datapoints added to Interface are displayed  
        By default the dps are diplayed once, if is_permanent=True: displayed each time
        '''
        for dp in dps:
            current_data = {'pos':dp, 'color':color, 'permanent':is_permanent}
            if not current_data in cls.dps_data:
                new_dp = Form(Const['DP'], dp, color=color, scale_pos=scale)
                cls.dps_data.append(current_data)
                cls.dps.append(new_dp)

    @classmethod
    def reset_dps(cls):
        '''Remove every dps stored'''
        cls.dps = []
        cls.dps_data = []

    @classmethod
    def keep_track_mov(cls, color, auto=True):
        '''
        Keep track of the robot movements and display the movements on the screen

        auto: if True, the position is updated at each frame in the run() method,
        else: to get a new position use update_pos method
        '''
        if cls.robot:
            cls.keep_track = True
            cls.kt_color = color
            cls.kt_auto = auto
            # store depart position
            cls.update_kt(cls.robot.get_pos(), scale=True)
        else:
            raise AttributeError('Must have a robot assigned')
    
    @classmethod
    def set_pos(cls, pos, scale=True):
        '''Update robot position, if keep track: update keep track position'''
        
        cls.robot.set_pos(pos, center=True, scale=scale)

        if cls.keep_track and not cls.auto_kt:
            cls.update_kt(cls.robot.get_pos(), scale)

    @classmethod
    def update_kt(cls, pos, scale=False):
        if cls.kt_last_pos != pos: # check if robot moved
            cls.kt_last_pos = pos.copy()
            new_dp = Form(Const['DP'], pos, cls.kt_color, scale_pos=scale)
            cls.kt_dps.append(new_dp)

    @classmethod
    def display_kt(cls):
        for i in range(len(cls.kt_dps)-1):
            pygame.draw.line(cls.screen,cls.kt_color, cls.kt_dps[i].pos, cls.kt_dps[i+1].pos, Const['small_width'])

    @classmethod
    def run(cls):
        '''
        Display the screen, plan, dps, robot, scale and get the inputs
        '''
        pressed, events = super().run()

        if cls.plan and cls.is_plan_displayed:
            cls.display_plan()

        to_remove = []
        # display dps
        for i in range(len(cls.dps)):
            cls.dps[i].display()
            if not cls.dps_data[i]['permanent']:
                to_remove.append(i)

        # remove not permanent ones
        for idx in to_remove[::-1]:
            cls.dps.pop(idx)
            cls.dps_data.pop(idx)

        if cls.keep_track:
            if cls.kt_auto:
                cls.update_kt(cls.robot.get_pos(), scale=True)
            cls.display_kt()
        
        if cls.robot:
            cls.robot.display() 
        
        cls.display_scalebar()
        cls.info_board.display()

        cls.react_events(pressed)

        return pressed, events

# load the image of the robot
img_robot = pygame.image.load(os.path.join('data','robotic.png'))
img_robot = pygame.transform.scale(img_robot, Const['ROBOT'])

class Robot(Form):
    '''
    Robot that is display on the screen

    Attributes:
        Form's attributes
        move()
        x
        y
        get_pos()
        set_pos(): NOTE always set center=True, NOTE(2) scale=False by default
    '''
    
    def __init__(self, pos, orientation, display_border=True):
        
        super().__init__(Const['ROBOT'], pos, surface=img_robot, center=True)
        
        self.orien = orientation
        self.display_border = display_border

    def display(self):
        if self.display_border:
            # display orientation lines
            start = self.center

            x = cos(self.orien + Spec.SENSOR_ANGLES[0]) * Spec.SENSOR_SCOPE + self.x
            y = sin(self.orien + Spec.SENSOR_ANGLES[0]) * Spec.SENSOR_SCOPE + self.y

            x,y = Interface.dim.scale((x,y))

            pygame.draw.line(Interface.screen, C.LIGHT_BROWN, start, (x,y), Const['small_width'])

            x = cos(self.orien + Spec.SENSOR_ANGLES[1]) * Spec.SENSOR_SCOPE + self.x
            y = sin(self.orien + Spec.SENSOR_ANGLES[1]) * Spec.SENSOR_SCOPE + self.y

            x,y = Interface.dim.scale((x,y))

            pygame.draw.line(Interface.screen, C.LIGHT_BROWN, start, (x,y), Const['small_width'])

        # display robot image
        super().display()

    @property
    def x(self):
        return Interface.dim.inv_scale(self.center[0])
    
    @property
    def y(self):
        return Interface.dim.inv_scale(self.center[1])
    
    def set_orien(self, value):
        value = pi_2_pi(value)
        self.orien = value

    def get_pos(self):
        return Interface.dim.inv_scale(self.center)

    def move(self, distance, scale=True):
        ''' Move the robot of a given distance, if scale: distance is scaled to current screen size'''
        if scale:
            x = cos(self.orien) * distance + self.x
            y = sin(self.orien) * distance + self.y
            pos = Interface.dim.scale((x,y))
        else:
            x = cos(self.orien) * distance + self.pos[0]
            y = sin(self.orien) * distance + self.pos[1]
            pos = [x,y]
        
        self.set_pos(pos, center=True)
