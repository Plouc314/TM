from interface import Interface, Robot, Sensor, Dp
from controller import BaseController
from simul import BaseSimulation
from math import pi
import pygame

YELLOW = (255,255,100)

robot = Robot((600,500),0,[Sensor(0)])
inter = Interface(robot)

inter.keep_track_mov(YELLOW)

controller = BaseController(robot.pos, robot.alpha)


while inter.running:
    pressed = inter.run()

    if controller.done and controller.connected:
        order = input('Enter an order: ')
        if order == 'quit':
            inter.running = False
            break
        controller.send_order(order)

    if controller.conn.recieved:
        controller.handeln_data(controller.conn.msg)
        
        # update robot
        inter.pos = controller.pos
        inter.robot.alpha = controller.alpha

    # get and display particles and landmarks
    landmarks = BaseSimulation.get_landmarks(controller.fastslam)
    particles = BaseSimulation.get_particles(controller.fastslam)
    BaseSimulation.display_dps(inter.screen, particles)
    BaseSimulation.display_dps(inter.screen, landmarks)

BaseSimulation.store_landmarks(controller.fastslam, 'landmarks.csv', 50)
controller.conn.stop()

