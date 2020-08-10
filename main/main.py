from interface import Interface, Robot, Const, C
from controller import BaseController
import pygame

Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN)

Interface.set_robot(Robot((800,800), 0))

Interface.keep_track_mov(C.YELLOW)

controller = BaseController(Interface.robot.get_pos(), Interface.robot.orien)


while Interface.running:
    
    if controller.done and controller.connected:
        order = input('Enter an order: ')
        if order == 'quit':
            Interface.running = False
            break
        controller.send_order(order)

    if controller.conn.recieved:
        controller.handeln_data(controller.conn.msg)
        
        # update robot
        Interface.set_pos(controller.pos)
        Interface.robot.orien = controller.alpha

    # get and display particles and landmarks
    landmarks = controller.fastslam.get_landmarks_dps()
    particles = controller.fastslam.get_particles_dps()
    Interface.add_dps(particles, C.WHITE, is_permanent=False)
    Interface.add_dps(landmarks, C.BORDEAU, is_permanent=False)

    pressed = Interface.run()

controller.fastslam.store_landmarks()
controller.conn.stop()

