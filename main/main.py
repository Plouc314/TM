from interface import Interface, Robot, Sensor
from controller import BaseController
from simul import BaseSimulation
from math import pi

robot = Robot((600,500),0,[Sensor(0)])
inter = Interface(robot)

controller = BaseController(robot.pos, robot.alpha)


while inter.running:
    pressed = inter.run()
    if controller.conn.recieved:
        controller.handeln_data(controller.conn.msg)
        if controller.done:
            order = input('Enter an order: ')
            if order == 'quit':
                inter.running = False
                break
            controller.send_order(order)
        # update robot
        inter.robot.pos_x, robot.pos_y = controller.pos
        inter.robot.alpha = controller.alpha

    # display robot
    inter.robot.display(inter.screen)

    # get and display particles and landmarks
    landmarks = BaseSimulation.get_landmarks(controller.fastslam)
    particles = BaseSimulation.get_particles(controller.fastslam)
    BaseSimulation.display_dps(inter.screen, particles)
    BaseSimulation.display_dps(inter.screen, landmarks)

controller.conn.stop()

