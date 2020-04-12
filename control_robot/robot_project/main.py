#!/usr/bin/env pybricks-micropython

from time import time
import threading

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from umqtt.robust import MQTTClient as Client


ANGLE_MARGE = 8

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
        self.last_measure = 0
        self.current_order = 'off'
        self.current_value = 0 # store the distance/angle covered
        self.thread = threading.Thread() # thread executing the move/turn function
        self.thread_is_alive = False # manual implementation of Thread.is_alive() as it's not available

        self.motor1 = Motor(Port.B)
        self.motor2 = Motor(Port.C)
        self.sensor_motor = Motor(Port.A)
        self.robot = DriveBase(self.motor1, self.motor2, 56, 121)
        self.ultra_sensor = UltrasonicSensor(Port.S4)
        self.gyro_sensor = GyroSensor(Port.S1)
        print('initializated')

    def rotate(self, angle, measures):
        self.sensor_motor.run_angle(60,angle,Stop.COAST,False)
        wait(200)
        while self.sensor_motor.speed() != 0:
            # test to stop the motor when it's going crazy
            if abs(self.sensor_motor.speed()) > 400:
                self.sensor_motor.stop()
                print('Motor sensor bug')
                # stop program
                self.running = False
            measures.append((self.sensor_motor.angle(), self.ultra_sensor.distance()))
            wait(200)

    def corrige_sensor_angle(self):
        angle = self.sensor_motor.angle()
        self.sensor_motor.run_angle(60,-angle,Stop.COAST,True)

    def take_rotate_measure(self, done=True):
        measures = []
        self.rotate(60, measures)
        wait(500)
        self.rotate(-120, measures)
        wait(500)
        self.rotate(60, measures)
        wait(200)

        # replace each time the sensor
        self.corrige_sensor_angle()
        # prepare msg to send
        msg = '{} {}'.format(g.current_order, g.current_value)
        for m in measures:
            msg += ' ' + str(m).replace(' ','')
        msg += ' done'
        client.publish('topic/robot', msg)
        

    def check_bug(self):
        if self.motor1.speed() > 500 or self.motor2.speed() > 500:
            self.motor1.stop()
            self.motor2.stop()
            return True
        if abs(abs(self.motor1.speed()) - abs(self.motor2.speed())) > 100:
            self.motor1.stop()
            self.motor2.stop()
            return True



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
            
            if self.ultra_sensor.distance() < 300:
                break

            if self.check_bug():
                print('large motor bug')
                break

            #    wait(500)
            #    dis_covered = (time() - start_time)*10 - previous_distance - self.move_speed/2000
            #    previous_distance += dis_covered
            #    # check if the measure is valid
            #    measure = self.ultra_sensor.distance()
            #    if self.is_noise(measure):
            #        print('noise: {:.2f}'.format(measure))
            #        measure = self.last_measure
            #    # send the distance covered and the distance measured by the ultrasensor
            #    msg = 'move {} {}'.format(dis_covered, measure)
            #    print(msg)
            #    client.publish('topic/robot', msg)
            #else:
            #    break
            wait(300)

        self.robot.stop(Stop.BRAKE)
        self.current_value = (time() - start_time)*10 - previous_distance
        wait(100)
        self.cum_angle += self.gyro_sensor.angle()
        self.take_rotate_measure()
        self.thread_is_alive = False
        
        

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
        while self.gyro_sensor.angle() < angle - ANGLE_MARGE:
            wait(100)

            if self.check_bug():
                print('large motor bug')
                break

            if time() - start_delay > .5:
                angle_covered = self.gyro_sensor.angle() - previous_angle
                previous_angle += angle_covered
                # send the distance measured by the ultrasensor
                msg = 'turn {} {}'.format(angle_covered, self.ultra_sensor.distance())
                print(msg)
                client.publish('topic/robot', msg)
                start_delay = time()

        self.robot.stop(Stop.BRAKE)
        self.current_value = self.gyro_sensor.angle() + ANGLE_MARGE - previous_angle
        print('covered:', self.current_value + previous_angle)
        wait(100)
        self.take_rotate_measure()
        self.thread_is_alive = False

    def is_noise(self, measure):
        if self.last_measure != 0:
            if self.last_measure - measure > 200:
                self.last_measure = self.last_measure - 50
                return True
        self.last_measure = measure
        return False 

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
        wait(500)
        if g.input == 'stop':
            g.running = False
        elif not g.thread_is_alive:
            g.current_value = 0
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
            wait(100)              


client.set_callback(get_messages)
client.subscribe(g.topic_input)
print('connected')

client.publish('topic/robot', 'connected')

while g.running:
    wait(100)
    client.check_msg()

print('cumulative angle:',g.cum_angle)
client.disconnect()
print('disconnected')
