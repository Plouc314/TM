'''
Interface is an extension of the pygame module, it is written to create GUI while still using raw pygame beside.
The main features are: static object Interface, Form, TextBox, Button, InputText & Cadre
'''

import pygame
from pygame.locals import *

pygame.init()

warning_msg = 'interface.py - WARNING: '

def mean(array):
    total = 0
    for v in array:
        total += v
    return total/len(array)

def center_text(dim_box, font, text, *, execption=False):
    error_msg = 'Dimension too small for text'
    width, height = font.size(text)
    if width > dim_box[0] or height > dim_box[1]:
        if execption:
            raise ValueError(error_msg)
        else:
            print(warning_msg + error_msg)
    
    x_marge = int((dim_box[0] - width)/2)
    y_marge = int((dim_box[1] - height)/2)
    return x_marge, y_marge

def rl(*args):
    '''Round list of floats'''

    if len(args) == 1:
        args = args[0]
    args = list(args)
    
    for i in range(len(args)):
        args[i] = int(args[i])
    return args

class Delayer:
    '''
    Creates decorators,

    The decorated function should return True/False depending on whether or not it has been activated,
    if true, creates a delay in order to not be spammed.
    '''
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

class Dimension:
    f = 1
    def __init__(self, dim, f):
        self.f = f
        self.WINDOW = dim # original dim: CONST!
        
        # unscaled
        self.center_x = int(dim[0]/2)
        self.center_y = int(dim[1]/2)
        self.center = (self.center_x, self.center_y)
        self.x = dim[0]
        self.y = dim[1]
    
    @property
    def rx(self):
        '''Scaled x dimension'''
        return self.E(self.WINDOW[0])
    
    @property
    def ry(self):
        '''Scaled y dimension'''
        return self.E(self.WINDOW[1])

    @property
    def size(self):
        '''Scaled dimension of the window'''
        return self.scale(self.WINDOW)

    def scale(self, x, factor=None, fp=None):
        
        if factor:
            f = factor
        else:
            f = self.f

        if type(x) == list or type(x) == tuple:
            x = list(x)
            for i in range(len(x)):
                x[i] = round(x[i]*f, fp)
        else:
            x = round(x*f, fp)
        return x

    def inv_scale(self, x, fp=5):
        '''
        Inverse the scale of the number: scale -> unscale ( note: unscale -> ?!@#@ )

        Arguments:
            - fp : floating point of the unscaled number (to keep precision)

        '''
        f = 1/self.f
        if type(x) == list or type(x) == tuple:
            x = list(x)
            for i in range(len(x)):
                x[i] = round(x[i]*f, fp)
        else:
            x = round(x*f, fp)
        return x

    def E(self, x, *, fp=None):
        return round(x*self.f, fp)

class C:
    '''Container of predefined colors'''
    WHITE = (255,255,255)
    BLACK = (0,0,0)
    LIGHT_BLUE = (135,206,250)
    BLUE = (65,105,225)
    DARK_BLUE = (7, 19, 134)
    LIGHT_GREY = (200,200,200)
    XLIGHT_GREY = (230,230,230)
    LIGHT_RED = (255, 80, 80)
    RED = (225, 50, 50)
    LIGHT_GREEN = (124,252,100)
    GREEN = (94,222,70)
    DARK_GREEN = (17, 159, 26)
    LIGHT_BROWN = (225, 167, 69)
    DARK_PURPLE = (140, 17, 159)
    PURPLE = (180, 57, 199)
    LIGHT_PURPLE = (210, 87, 229)
    YELLOW = (253, 240, 49)

class Font:
    '''Create pygame fonts, use .f method'''
    interface = None
    fontname = 'Arial'
    @classmethod
    def f(cls, size):
        '''Create a font of the given size: { 'size' :  size, 'font' :  pygame.font }'''
        if cls.interface:
            return {'size':size , 'font':pygame.font.SysFont(cls.fontname, cls.interface.dim.E(size))}
        else:
            return {'size':size , 'font':pygame.font.SysFont(cls.fontname, size)}

class Form:
    '''
    Base object of the module, extension of pygame Surface object

    Form's dimension and position are auto rescaled when window is rescaled

    Form can take either a color, in that case the Form's surface will be uni-color, or a custom surface (can have a colored font)

    Methods:
        - set_pos : set a new position for the object
        - set_dim_pos : set new position and dimension, keep the same surface
        - set_surf : set a new surface, can be a pygame surface or a numpy.ndarray
        - set_color : set a new color to uni-color surface or font
        - on_it : return if the mouse is on the Form's surface
    '''
    screen = None
    MARGE_WIDTH = 4
    rs_marge_width = 4
    MARGE_TEXT = 5
    rs_marge_text = 5
    dim_object = None
    def __init__(self, dim, pos, color=C.WHITE, *, rescale=True, scale_pos=True, scale_dim=True, center=False, surface=None, surf_font_color=None):
        
        self.set_dim_attr(dim, scale=scale_dim, update_surf=False)
        
        self.set_pos(pos, scale=scale_pos, center=center)
        
        self.COLOR = color
    
        self.set_surf(surface, surf_font_color=surf_font_color)
        
        if rescale:
            # add every gui obj to interface to be able to rezise gui objs auto
            Interface.gui_objects.append(self)

    def set_surf(self, surface=None, surf_font_color:tuple = None):
        '''
        Set the surface attribute, 
        
        a dict: main surface, original surface (when custom), font surface (optional), surface type (intern purposes)
        
        Arguments:
            surface : can be either a pygame.Surface or numpy.ndarray object
            surf_font_color : if a custom surface is set & is partly transparent, surf_font color will fill the blanks
            note : if surface not specified, set a uni-color surface with the current COLOR attribute
        '''

        # dict with type, original surf, main surf, font surf, font surf color
        self.surf = {}
        try:
            if not surface:
                # by default: create a uni-colored surface
                self.surf['type'] = 'default'
                self.surf['original'] = pygame.Surface(rl(self.dim), pygame.SRCALPHA)
                self.surf['main'] = pygame.Surface(rl(self.dim), pygame.SRCALPHA)
                self.surf['main'].fill(self.COLOR)
                self.surf['original'].fill(self.COLOR)
            else:
                # if custom surface: keep a original surf to rescale properly
                self.surf['type'] = 'custom'
                self.surf['original'] = surface
                self.surf['main'] = pygame.transform.scale(surface, rl(self.dim))
        except ValueError:
            # numpy array
            self.surf['type'] = 'custom'
            self.surf['original'] = pygame.surfarray.make_surface(surface)
            self.surf['main'] = pygame.transform.scale(self.surf['original'], rl(self.dim))

        # if defined set font surfs
        self.surf['font'] = None
        self.surf['font color'] = None
        if surf_font_color:
            self.surf['font'] = pygame.Surface(rl(self.dim))
            try:
                self.surf['font'].fill(surf_font_color)
                self.surf['font color'] = surf_font_color
            except:
                print(warning_msg + "surf_font argument must be a tuple (color)")
                self.surf['font'] = None # reset font

    def rotate(self, angle: int, *, rotate_font=True):
        '''
        Rotate the surface of a given angle (degree)

        Args: rotate_font: if a font as been specified, if it has to be rotated
        '''
        if angle == 0:
            return # potential more efficient

        new_main = pygame.transform.scale(self.surf['original'], rl(self.dim))

        # get none rotated center
        x1, y1 = new_main.get_rect().center

        # rotate surf
        self.surf['main'] = pygame.transform.rotate(new_main, angle)

        # get rotated center
        x2, y2 = self.surf['main'].get_rect().center
        
        # get deviation between the twos centers
        dx = x2 - x1
        dy = y2 - y1

        # adjust position to patch deviation
        self.set_pos([self.pos[0] - dx, self.pos[1] - dy])

        if self.surf['font'] and rotate_font:
            self.surf['font'] = pygame.transform.rotate(pygame.Surface(rl(self.dim), pygame.SRCALPHA), angle)
            self.surf['font'].fill(self.surf['font color'])
        
    def set_dim_attr(self, dim, *, scale=False, update_original=True, update_surf=True):
        dim = list(dim)
        if scale:
            self.dim = self.dim_object.scale(dim)
            if update_original:
                self.unscaled_dim = list(dim)
        else:
            self.dim = list(dim)
            if update_original:
                self.unscaled_dim = self.dim_object.inv_scale(dim)
        
        if update_surf:
            self.rescale_surf()
    
    def set_pos_attr(self, pos, *, scale=False, update_original=True):
        if scale:
            self.pos = self.dim_object.scale(pos)
            if update_original:
                self.unscaled_pos = list(pos)
        else:
            self.pos = list(pos)
            if update_original:
                self.unscaled_pos = self.dim_object.inv_scale(pos)

    def set_color(self, color: tuple, *, marge=False):
        '''
        Set the color of the surface

        Arguments:
            - marge : set to True if the object as marge
        '''
        # if has custom surf -> change font color, else change normal color
        if self.surf['font color']:
            self.surf['font color'] = color
            self.surf['font'].fill(color)
        else:
            self.surf['main'].fill(color)
            self.COLOR = color
        
        if marge:
            self.set_highlight_color()
            self.MARGE_COLOR = self.high_color

    def set_highlight_color(self):
        
        # check if has a font color, else base color is .COLOR
        if self.surf['font color']:
            base_color = self.surf['font color']
        else:
            base_color = self.COLOR

        light_color = []
        for i in range(3):
            if base_color[i] <= 235:
                light_color.append(base_color[i] + 20)
            else:
                light_color.append(255)
        
        dark_color = []
        for i in range(3):
            if base_color[i] >= 20:
                dark_color.append(base_color[i] - 20)
            else:
                dark_color.append(0)

        if mean(self.COLOR) < 130:
            self.high_color = light_color
        else:
            self.high_color = dark_color

    def display_margin(self):
        pygame.draw.line(self.screen, self.MARGE_COLOR, self.TOPLEFT   , self.TOPRIGHT   , self.rs_marge_width)
        pygame.draw.line(self.screen, self.MARGE_COLOR, self.TOPLEFT   , self.BOTTOMLEFT , self.rs_marge_width)
        pygame.draw.line(self.screen, self.MARGE_COLOR, self.TOPRIGHT  , self.BOTTOMRIGHT, self.rs_marge_width)
        pygame.draw.line(self.screen, self.MARGE_COLOR, self.BOTTOMLEFT, self.BOTTOMRIGHT, self.rs_marge_width)

    def display(self, *, pos=None, marge=False):
        '''Display form
        
        Arguments:
            pos : can specify position where the form is displayed
            marge : if the marges are also displayed
        '''

        if pos:
            pos = rl(pos)
        else:
            pos = rl(self.pos)
        
        # if surface has a font color, display it before the main surf
        if self.surf['font']:
            self.screen.blit(self.surf['font'], pos)

        self.screen.blit(self.surf['main'], pos)
        
        if marge:
            self.display_margin()
    
    def on_it(self):
        '''Return if the mouse is on the surface'''
        mouse_pos = pygame.mouse.get_pos()
        if mouse_pos[0] > self.TOPLEFT[0] and mouse_pos[0] < self.TOPRIGHT[0]:
            if mouse_pos[1] > self.TOPLEFT[1] and mouse_pos[1] < self.BOTTOMLEFT[1]:
                return True
        return False

    def set_corners(self):
        self.TOPLEFT = rl(self.pos)
        self.TOPRIGHT = rl(self.pos[0]+self.dim[0],self.pos[1])
        self.BOTTOMLEFT = rl(self.pos[0], self.pos[1]+self.dim[1])
        self.BOTTOMRIGHT = rl(self.pos[0]+self.dim[0],self.pos[1]+self.dim[1])

    @property
    def center(self):
        return [round(self.TOPLEFT[0] + self.dim[0]/2), round(self.TOPLEFT[1] + self.dim[1]/2)]

    def rescale_surf(self):
        '''Rescale the surf attribute to the current dimension'''
        
        if self.surf['type'] == 'default':
            self.surf['main'] = pygame.Surface(rl(self.dim))
            self.surf['main'].fill(self.COLOR)
        else:
            self.surf['main'] = pygame.transform.scale(self.surf['original'], rl(self.dim))
        
        if self.surf['font']:
            self.surf['font'] = pygame.Surface(rl(self.dim))
            self.surf['font'].fill(self.surf['font color'])

    def set_pos(self, pos, *, center=False, scale=False):
        '''Set a new position
        
        Arguments:
            - pos : the new position
            - center : if true, the form will be centered on the new pos, else the new pos is the top left of the obj
            - scale : if the position needs to be rescaled to the current window's dimension
        '''
        pos = list(pos)
        if not center:
            self.set_pos_attr(pos, scale=scale)
            self.set_corners()
        else:
            if scale:
                pos = self.dim_object.scale(pos)

            top_left = [int(pos[0]-self.dim[0]/2), int(pos[1]-self.dim[1]/2)]
            self.set_pos_attr(top_left)
            self.set_corners()

    def set_dim_pos(self, dim, pos, *, scale_dim=False, scale_pos=False, update_original=True):
        '''
        Reset dimension & position of form
        
        Arguments:
            - dim, pos : new dimension, position
            - scale_dim/pos : if new dim/pos need to be scaled to current window's dimension
            - update_original : if new dim/pos are kept when window is resized
        '''
        self.set_dim_attr(dim, scale=scale_dim, update_original=update_original)
        self.set_pos_attr(pos, scale=scale_pos, update_original=update_original)
        self.set_corners()

class Cadre(Form):
    def __init__(self, dim, pos, color=C.WHITE, *, set_transparent=False, scale_dim=True, scale_pos=True):
        
        super().__init__(dim, pos, color, scale_dim=scale_dim, scale_pos=scale_pos)
        
        self.set_corners()
        self.set_highlight_color()
        self.MARGE_COLOR = self.high_color
        self.is_transparent = set_transparent
        if set_transparent:
            self.surf['main'].set_colorkey(color)

    def rescale_surf(self):
        # overwrite rescale method to keep the cadre transparent when rescaling
        super().rescale_surf()
        if self.is_transparent:
            self.surf['main'].set_colorkey(self.COLOR)

    def display(self):
        super().display()
        self.display_margin()

class Button(Form):
    ''' 
    It's a button, can display text, has marge, can be pushed.

    Inherited from Form.

    Arguments:
        - text_color : color of the text
        - centered : if the text is centered
        - highlight : if the surface is highlighted when the mouse pass on it
        - surface : can set a custom surface, can be an image, numpy.ndarray, pygame.Surface
        - surf_font_color : if a custom surface is set & is partly transparent, surf_font color will fill the blanks
        - text : the text that is present at the beginning
    
    Methods:
        - pushed : Return if the surface has been clicked
        - display
    '''
    def __init__(self, dim, pos, color=C.WHITE, text='', *, text_color=C.BLACK, centered=True, font=Font.f(50), 
                    surface=None, surf_font_color=None, scale_dim=True, scale_pos=True, highlight=True):
        
        super().__init__(dim, pos, color, scale_dim=scale_dim, scale_pos=scale_pos, surface=surface, surf_font_color=surf_font_color)

        self.text = text
        
        self.text_color = text_color
        self.set_corners()
        self.highlighted = highlight
        self.centered = centered
        self.font = font
        self.set_highlight_color()
        self.MARGE_COLOR = self.high_color

    def pushed(self, events):
        '''Return True if the object was clicked'''
        if self.on_it():
            for event in events:
                if event.type == pygame.MOUSEBUTTONUP:
                    return True

    def highlight(self):
        '''
        Set the highlight color on surf, if custom surf with font: set highlight on surf font
        '''
        if self.on_it():
            # check if custom surf & font or normal unicolor
            if self.surf['type'] == 'custom':
                if self.surf['font']: # check if has a font
                    self.surf['font'].fill(self.high_color)
                else:
                    self.surf['main'].fill(self.high_color)
            else:
                self.surf['main'].fill(self.high_color)
        else:
            # check if custom surf
            if self.surf['type'] == 'custom':
                if self.surf['font']: # check if has a font
                    self.surf['font'].fill(self.surf['font color'])
                else:
                    self.surf['main'] = pygame.transform.scale(self.surf['original'], rl(self.dim))
            else:
                self.surf['main'].fill(self.COLOR)
                
    def display(self):
        # if highlight actived, handeln highlight color changes
        if self.highlighted:
            self.highlight()
        
        # display the surf
        super().display()
        
        self.display_margin()
        
        # dusplay the text with marges
        if self.text: # check that there is text
            x_marge, y_marge = center_text(self.dim, self.font['font'], self.text)
            if not self.centered:
                x_marge = self.rs_marge_text
            font_text = self.font['font'].render(self.text,True,self.text_color)
            self.screen.blit(font_text,rl(self.pos[0]+x_marge,self.pos[1]+y_marge))

class TextBox(Form):
    ''' 
    Display text, can have multiple lines and marges.

    Inherited from Form.

    Arguments:
        - text_color : color of the text
        - centered : if the text is centered
        - text : the text that is present at the beginning
        - marge : if it has marge or not
    
    Methods:
        - set_text : pass a string, can be on multiple lines
        - display
    '''
    def __init__(self, dim, pos, color=C.WHITE, text='', *,
                    text_color=C.BLACK, centered=True, font=Font.f(50), marge=False, scale_dim=True, scale_pos=True):
        
        super().__init__(dim, pos, color, scale_dim=scale_dim, scale_pos=scale_pos)
        
        self.text = text
        self.centered = centered
        self.font = font
        self.lines = text.split('\n')
        self.text_color = text_color
        self.set_corners()
        self.as_marge = marge
        if marge:
            self.set_highlight_color()
            self.MARGE_COLOR = self.high_color

    def set_text(self, text):
        self.text = text
        self.lines = text.split('\n')

    def display(self):
        super().display()
        if self.as_marge:
            self.display_margin()

        # split the box in n part for n lines
        y_line = round(self.dim[1]/len(self.lines))
        for i, line in enumerate(self.lines):
            x_marge, y_marge = center_text((self.dim[0],y_line), self.font['font'], line)
            if not self.centered:
                x_marge = self.rs_marge_text
            font_text = self.font['font'].render(line,True,self.text_color)
            self.screen.blit(font_text,rl(self.pos[0]+x_marge,self.pos[1]+i*y_line+y_marge))

get_input_deco = Delayer(3)
cursor_deco = Delayer(15)

class InputText(Button):
    '''
    Get text input when triggered (clicked). Input text is stored in .content attribute

    Inherited from Button.

    Arguments:
        - text_color : color of the text
        - centered : if the text is centered
        - limit : the limit of character that can be entered
        - cache : if the input is hidden (with $ )
        - text : the text that is present at the beginning
        - pretext : text that disapear when the surface is clicked
    
    Methods:
        - pushed : Return if the surface has been clicked
        - run : Get input ( execute once by frame )
        - display
    '''
    CURSOR_WIDTH = 2
    bool_cursor = True
    # store decorators to be able to adjust delay in func of fps
    input_deco = get_input_deco
    cursor_deco = cursor_deco
    def __init__(self, dim, pos, color=C.WHITE, text='', *, text_color=C.BLACK, centered=False, font=Font.f(30),
                    limit=None, cache=False, scale_dim=True, scale_pos=True, pretext=None):

        super().__init__(dim, pos, color, text_color=text_color, centered=centered, font=font, scale_dim=scale_dim, scale_pos=scale_pos)
        
        self.active = False
        self.cursor_place = len(text)
        self.limit = limit # max char
        self.cache = cache # if true text -> ***
        self.pretext = pretext # if something: add a text that disapear when active
        # text: text that is display
        # content: text that is input
        self.set_text(text, with_pretext=self.pretext)
    
    def set_text(self, text, with_pretext=False):
        text = str(text)
        self.content = text
        self.cursor_place = len(text)
        if with_pretext:
            self.text = self.pretext
        else:
            if self.cache:
                self.text = '$' * len(self.content)
            else:
                self.text = self.content

    @get_input_deco
    def is_moving_left(self, pressed):
        if pressed[pygame.K_LEFT]:
            return True

    @get_input_deco
    def is_moving_right(self, pressed):
        if pressed[pygame.K_RIGHT]:
            return True

    @get_input_deco
    def get_input(self, events, pressed):
        self.active = True
        
        # check for end active
        if pressed[pygame.K_RETURN]:
            self.active = False
            self.highlighted = False
            return False
        elif not self.on_it():
            pushed = False
            for event in events:
                if event.type == pygame.MOUSEBUTTONUP:
                    pushed =  True 
            if pushed:
                self.active = False
                self.highlighted = False
                return False

        # check for new char
        key = get_pressed_key(pressed)
        if key:
            self.content = self.content[:self.cursor_place] + key + self.content[self.cursor_place:]
            self.cursor_place += 1
            if self.limit:
                if len(self.content) > self.limit:
                    self.content = self.content[:-1]
                    self.cursor_place -= 1
                
            try:
                center_text(self.dim, self.font['font'], self.content,  execption=True)
            except ValueError:
                self.content = self.content[:-1]
                self.cursor_place -= 1
            return True
        
        # check for char deletion
        if pressed[pygame.K_BACKSPACE]:
            self.content = self.content[:self.cursor_place-1] + self.content[self.cursor_place:]
            self.cursor_place -= 1
            return True
    
        # check for cursor change
        if self.is_moving_left(pressed):
            if self.cursor_place > 0:
                self.cursor_place -= 1
        
        if self.is_moving_right(pressed):
            if self.cursor_place < len(self.content):
                self.cursor_place += 1

        if self.cache:
            self.text = '$' * len(self.content)
        else:
            self.text = self.content

        return False
    
    def display_text_cursor(self):
        width, height = self.font['font'].size(self.text[:self.cursor_place])
        x_marge, y_marge = center_text(self.dim, self.font['font'], self.text[:self.cursor_place])
        if not self.centered:
            x_marge = self.rs_marge_text

        bottom_pos = rl(self.TOPLEFT[0] + x_marge + width, self.BOTTOMLEFT[1]-y_marge)
        top_pos = rl(self.TOPLEFT[0] + x_marge + width, self.TOPLEFT[1]+y_marge)
        
        if self.bool_cursor:
            pygame.draw.line(self.screen, C.BLACK, top_pos, bottom_pos, self.CURSOR_WIDTH)
        self.change_cursor_state()

    @cursor_deco
    def change_cursor_state(self):
        self.bool_cursor = not self.bool_cursor
        return True

    def run(self, events, pressed):
        if self.pushed(events):
            self.active = True
            # if pretext, remove pretext
            if self.pretext:
                self.set_text(self.content)
        
        if self.active:
            self.highlighted = True
            self.display_text_cursor()
            self.get_input(events, pressed)


class Interface:
    '''
    Manage the window and all the gui objects (Form, Button, ...), update the screen and mangae the fps.
    The window always appear in fullscreen (to be fixed), the window can be resized and all objects will be rescaled auto.

    Work with pygame, screen object store as an attribute. So can create own pygame object and add them to the Interface using .add_resizable_objects.

    Must be setup (using .setup method) before be used.

    Methods:
        - setup : Initialize the module, create the window
        - add_resizable_objects : Add custom objects to be rescaled auto by Interface
        - run : update the screen, get the input for current frame, check for quit events...
    '''
    clock = pygame.time.Clock()
    running = True
    gui_objects = [] # all gui objects, ex: button, form...
    resize_objects = [] # must have a on_resize(self, factor) method

    @classmethod
    def setup(cls, dim: (int, int), win_title: str, *, FPS=30, font_color=C.WHITE):
        '''
        Arguments:
            dim: default window dimension, used to set other object's dimension
        '''
        
        # create dimension
        cls.dim = Dimension(dim, 1)
        cls.font_color = font_color
        
        # create screen in full screen dimension: resize to specified dim

        infoObject = pygame.display.Info()
        fullscreen_dim = rl(cls.dim.scale((infoObject.current_w, infoObject.current_h), factor=.96))

        cls.screen = pygame.display.set_mode(fullscreen_dim, HWSURFACE|DOUBLEBUF|RESIZABLE)
        cls.screen.fill(cls.font_color)
        pygame.display.set_caption(win_title)

        cls.FPS = FPS

        # adjust InputText deco to fps
        InputText.input_deco.delay = round(FPS/10) # 10 times a sec
        InputText.cursor_deco.delay = round(FPS/2) # 2 times a sec

        cls.set_screen(cls.screen)
        Form.dim_object = cls.dim # set dimension to rescale pos, dim in Font.__init__
        Font.interface = cls # must be set this way as Interface is declared after Font

        cls.x_padding = Form((0,0),(0,0),cls.font_color, rescale=False)
        cls.y_padding = Form((0,0),(0,0),cls.font_color, rescale=False)

        # rescale window to correct dim
        cls.rescale(fullscreen_dim)
        
    @classmethod
    def set_screen(cls, screen):
        Form.screen = cls.screen
        TextBox.screen = cls.screen
        Button.screen = cls.screen
        InputText.screen = cls.screen
        Cadre.screen = cls.screen

        for gui_obj in cls.gui_objects:
            gui_obj.screen = screen

    @classmethod
    def add_resizable_objs(cls, objects: list):
        '''
        Object's method on_rezise(self, scale_factor) will be called when window is rezised
        
        scale_factor: factor of dimension compared to initial dimension
        '''
        # resize a first time
        for obj in objects:
            # check that each object has a on_resize method, else: don't add obj to resizable ones
            if hasattr(obj, 'on_resize'):
                obj.on_resize(cls.dim.f)
                cls.resize_objects.append(obj)
            else:
                print(warning_msg + 'resizable objects must have an on_resize(self, scale_factor) method')

    @classmethod
    def rescale(cls, new_dim):

        # set new scale factor
        scale_factor = min(new_dim[0]/cls.dim.x, new_dim[1]/cls.dim.y)

        cls.dim.f = scale_factor

        # create padding to fill blank space
        dim_px = [new_dim[0] - cls.dim.size[0], cls.dim.size[1]]
        if dim_px[0] > 0:
            pos = [cls.dim.size[0], 0]
            cls.x_padding.set_dim_pos(dim_px, pos, update_original=False)
        
        dim_py = [cls.dim.size[0], new_dim[1] - cls.dim.size[1]]
        if dim_px[1] > 0:
            pos = [0, cls.dim.size[1]]
            cls.y_padding.set_dim_pos(dim_py, pos, update_original=False)
        
        # resize marges
        Form.rs_marge_width = cls.dim.E(Form.MARGE_WIDTH)
        Form.rs_marge_text = cls.dim.E(Form.MARGE_TEXT)

        # resize every objects
        for gui_obj in cls.gui_objects:
            gui_obj.set_dim_pos(gui_obj.unscaled_dim, gui_obj.unscaled_pos, scale_pos=True, scale_dim=True, update_original=False)
            
            # rezise existing fonts
            if hasattr(gui_obj, 'font'):
                fontsize = gui_obj.font['size']
                gui_obj.font = Font.f(fontsize)

        for rz_obj in cls.resize_objects:
            rz_obj.on_resize(scale_factor)

    @classmethod
    def run(cls, fill=True):
        '''
        Execute once a frame
        Update the screen, get the input for current frame, check for quit events...
        Return:
            - pressed : pygame object
            - events : pygame object
        '''
        cls.x_padding.display()
        cls.y_padding.display()
        cls.clock.tick(cls.FPS)
        
        pygame.display.update()

        if fill:
            cls.screen.fill(cls.font_color)

        pressed = pygame.key.get_pressed()
        events = pygame.event.get()

        cls.mouse_pos = pygame.mouse.get_pos()

        for event in events:
            # check quit
            if event.type == pygame.QUIT:
                cls.running = False
            # check window resize
            if event.type == VIDEORESIZE:
                cls.rescale(event.dict['size'])
                
        if pressed[pygame.K_ESCAPE]:
            cls.running = False

        return pressed, events

def get_pressed_key(pressed):
    for i in range(97, 123):
        if pressed[i]:
            if pressed[pygame.K_LSHIFT]:
                return chr(i).upper()
            else:
                return chr(i)
    
    for i in range(48, 58):
        if pressed[i]:
            return chr(i)
    
    if pressed[pygame.K_SPACE]:
        return ' '
    elif pressed[pygame.K_MINUS] and pressed[pygame.K_LSHIFT]:
        return '_'
    elif pressed[pygame.K_QUOTE] and pressed[pygame.K_LSHIFT]:
        return '?'