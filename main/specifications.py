from math import pi

class Specifications:
    '''Contain all the specifications of the simulations'''
    WINDOW = (1600,1600)
    VIEW_ANGLE = 1/2*pi
    SENSOR_SCOPE = 400
    SENSOR_ANGLES = (-1/4*pi, 1/4*pi)
    GRID_SIZE = 20
    GRID_SHAPE = (20,20)
    N_SECTOR = 12
    MAX_ORDERS = 40
    SCALE_FACTOR = WINDOW[0] / GRID_SIZE
    DIF = 2 * pi / N_SECTOR