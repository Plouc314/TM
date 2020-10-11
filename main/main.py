from interface import Interface, Robot, Const, C
from controller import BaseController
from math import cos, sin

Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN)

Interface.set_robot(Robot((800,800), 0))

Interface.keep_track_mov(C.YELLOW)

controller = BaseController()

def display_raw_obs(controller, color=C.BORDEAU):
    '''
    Display the observations of the robot
    '''
    if controller.obs == None:
        return

    dps = []
    for dist, angle in controller.obs:
        x = controller.robot.x + cos(angle) * dist
        y = controller.robot.y + sin(angle) * dist
        dps.append((x,y))

        Interface.add_dps(dps, color, is_permanent=True)

def display_fastslam_obs(controller, color=C.BORDEAU):
    '''
    Display the landmarks of one of the particles of fastslam
    '''
    landmarks = controller.fastslam.get_landmarks_dps()
    Interface.add_dps(landmarks, color, is_permanent=False)

while Interface.running:
    
    if controller.has_new_msg:
        controller.handeln_msg()

        display_raw_obs(controller)

        # update robot position
        controller.run_fastslam()

    # don't ask for user input in same frame as msg received -> display new obs
    elif not controller.order_pending and controller.connected:
        order = input('Enter an order: ')
        if order == 'stop':
            Interface.running = False
        
        controller.send_order(order)

    #display_fastslam_obs(controller)

    # get and display particles
    particles = controller.fastslam.get_particles_dps()
    Interface.add_dps(particles, C.WHITE, is_permanent=False)

    pressed = Interface.run()

controller.fastslam.store_landmarks()
controller.conn.stop()

