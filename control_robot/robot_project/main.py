#!/usr/bin/env pybricks-micropython

from time import sleep, time
import threading

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from umqtt.robust import MQTTClient as Client



class General:
    '''
    Contain all the variables and functions

    As everything is executed in get_message() all the variables need to be mutable in local scope
    '''
    def __init__(self):
        self.running = True
        self.topic_input = 'topic/order'
        self.input = []
        self.move_speed = 100
        self.turn_speed = 50
        self.cum_angle = 0
        self.current_order = 'off'
        self.current_value = 0 # store the distance/angle covered
        self.thread = threading.Thread() # thread executing the move/turn function
        self.thread_is_alive = False # manual implementation of Thread.is_alive() as it's not available
        self.send_done = False

        self.motor1 = Motor(Port.B)
        self.motor2 = Motor(Port.C)
        self.robot = DriveBase(self.motor1, self.motor2, 56, 121)
        self.ultra_sensor = UltrasonicSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S4)
        print('initializated')

    def move(self, distance):
        '''
        Move the robot forward

        Executed in a separate thread to still be able to receive order
        '''
        self.thread_is_alive = True
        previous_distance = 0
        self.gyro_sensor.reset_angle(0)
        self.robot.drive(self.move_speed, 0)
        start_time = time()
        while time() - start_time < distance/10 + self.move_speed/2000:
            if self.ultra_sensor.distance() > 300:
                wait(500)
                dis_covered = (time() - start_time)*10 - previous_distance - self.move_speed/2000
                previous_distance += dis_covered
                # send the distance covered and the distance measured by the ultrasensor
                msg = 'move {} {}'.format(dis_covered, self.ultra_sensor.distance())
                print(msg)
                client.publish('topic/robot', msg)
            else:
                break

        self.robot.stop(Stop.BRAKE)
        self.cum_angle += self.gyro_sensor.angle()
        self.current_value = (time() - start_time)*10 - previous_distance
        self.thread_is_alive = False
        self.send_done = False
        

    def turn(self, angle):
        '''
        Turn the robot

        Executed in a separate thread to still be able to receive order
        '''
        self.thread_is_alive = True
        previous_angle = 0
        start_delay = time()
        self.gyro_sensor.reset_angle(0)
        self.robot.drive(0, self.turn_speed)
        while self.gyro_sensor.angle() < angle - 8:
            if time() - start_delay > .5:
                angle_covered = self.gyro_sensor.angle() - previous_angle
                previous_angle += angle_covered
                # send the distance measured by the ultrasensor
                msg = 'turn {} {}'.format(angle_covered, self.ultra_sensor.distance())
                print(msg)
                client.publish('topic/robot', msg)
                start_delay = time()
        sleep(0.1)
        self.robot.stop(Stop.BRAKE)
        self.current_value = self.gyro_sensor.angle() + self.turn_speed/10 - previous_angle
        print('covered:', self.current_value + previous_angle)
        self.thread_is_alive = False
        self.send_done = False

g = General()



# connect to the computer
client = Client('robot','10.42.0.1')
client.connect()

def get_messages(topic, msg):
    '''
    Function activated when receive order from the computer

    Start the execution of the order in separate thread
    '''
    if topic == g.topic_input.encode():
        g.input = str(msg.decode())
        print('input order {}'.format(g.input))
        # wait for the robot to stop
        sleep(0.5)
        if g.input == 'stop':
            g.running = False
        elif not g.thread_is_alive:
            g.input = g.input.split(' ')
            # execute the order in a separate thread
            if g.input[0] == 'move':
                g.current_order = 'move'
                g.thread = threading.Thread(target=g.move, args=[int(g.input[1])])
                g.thread.start()
            else:
                g.current_order = 'turn'
                g.thread = threading.Thread(target=g.turn, args=[int(g.input[1])])
                g.thread.start()
            sleep(.1)                


client.set_callback(get_messages)
client.subscribe(g.topic_input)
print('connected')


while g.running:
    if not g.thread_is_alive:
        if not g.send_done:
            g.send_done = True
            msg = '{} {} {} done'.format(g.current_order, g.current_value, g.ultra_sensor.distance())
            print(msg)
            client.publish('topic/robot', msg)
    sleep(0.1)
    client.check_msg()

print('cumulative angle:',g.cum_angle)
client.disconnect()
print('disconnected')
