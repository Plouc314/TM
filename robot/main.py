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
MOVE_MARGE = 0.05

class General:
    '''
    Contain all the variables and functions

    As everything is executed in get_message() all the variables need to be mutable in local scope
    '''
    def __init__(self):
        self.running = True
        self.topic_input = 'topic/order'

        self.move_speed = 100
        self.turn_speed = 50
        
        self.cum_angle = 0

        self.current_order = 'off'
        self.current_value = 0 # store the distance/angle covered
        
        # thread
        self.thread_is_alive = False # manual implementation of Thread.is_alive() as it's not available

        # components
        self.motor1 = Motor(Port.B)
        self.motor2 = Motor(Port.C)
        self.sensor_motor = Motor(Port.A)
        self.sensor_motor.reset_angle(0)
        self.robot = DriveBase(self.motor1, self.motor2, 56, 121)
        self.ultra_sensor = UltrasonicSensor(Port.S4)
        self.gyro_sensor = GyroSensor(Port.S1)
        
        print('Initializated...')

    def send_state_msg(self, measures):
        '''
        Send order executed, distance/angle covered and all the measure made.
        '''
        # prepare msg to send
        msg = '{} {}'.format(g.current_order, round(g.current_value))
        for angle, distance in measures:
            msg += '\n{} {}'.format(distance, angle)
    
        client.publish('topic/robot', msg)

    def send_critical_error_msg(self):
        '''
        Send that one of the large motor went crazy. 
        And thus that the robot position and orientation are potentially very different now.
        '''
        client.publish('topic/robot', 'ERROR')

    def rotate_sensor(self, angle):
        '''
        Rotate the sensor of the given angle and take 5 measures while rotating.  
        Return the taken measures.
        '''
        measures = []
        
        for i in range(5):
            self.sensor_motor.run_angle(60, angle//5, Stop.COAST, True)
            wait(200)

            measures.append((self.sensor_motor.angle(), self.ultra_sensor.distance()))

        return measures

    def reset_sensor_angle(self):
        '''
        Reposition the sensor to the default alignment.
        '''
        angle = self.sensor_motor.angle()
        self.sensor_motor.run_angle(60,-angle,Stop.COAST,True)

    def take_rotate_measure(self):
        '''
        Take a set of measures with the sensor.  
        In case of sensor motor bug, will try again to take the measures.  
        '''
        self.sensor_motor.reset_angle(0)
        measures = []

        measures += self.rotate_sensor(60)
        wait(500)
        
        measures += self.rotate_sensor(-120)
        wait(500)
        
        measures += self.rotate_sensor(60)
        wait(500)

        # reset the sensor angle
        self.reset_sensor_angle()
        
        return measures

    def check_large_motor_bug(self):
        '''
        Check if one of the large motor is going crazy.
        If so, stop the motors (both of them).
        '''
        if self.motor1.speed() > 500 or self.motor2.speed() > 500:
            self.motor1.stop()
            self.motor2.stop()
            print('Large motor bug.')
            return True
        
        if abs(abs(self.motor1.speed()) - abs(self.motor2.speed())) > 100:
            self.motor1.stop()
            self.motor2.stop()
            print('Large motor bug.')
            return True

    def move(self, distance):
        '''
        Move the robot forward

        Executed in a separate thread to still be able to receive order
        '''
        self.thread_is_alive = True
        
        # reset gyroscope angle
        self.gyro_sensor.reset_angle(0)

        # start to advance
        self.robot.drive(self.move_speed, 0)
        
        start_time = time()

        # estimate the distance covered using the elasped time
        # MOVE_MARGE is a little correction added to improve the precision of the movement
        while time() - start_time < distance/10 + MOVE_MARGE: 
            
            # stop the robot if it's going to close from a wall
            if self.ultra_sensor.distance() < 300:
                break

            if self.check_large_motor_bug():
                self.send_critical_error_msg()
                break

            wait(100)

        self.robot.stop(Stop.BRAKE)

        # compute covered distance
        self.current_value = (time() - start_time)*10
        
        # wait to let the robot stop
        wait(200)

        self.cum_angle += self.gyro_sensor.angle()
        
        # take measures
        measures = self.take_rotate_measure()
        # send message to computer
        self.send_state_msg(measures)

        self.thread_is_alive = False
        
    def turn(self, angle):
        '''
        Turn the robot

        Executed in a separate thread to still be able to receive order
        '''
        self.thread_is_alive = True

        # reset gyroscope angle
        self.gyro_sensor.reset_angle(0)
        
        # start the rotation
        self.robot.drive(0, self.turn_speed)

        # estimate the angle covered using the gyroscope sensor
        # MOVE_MARGE is a correction added to the gyroscope angle, it greatly improve the precision
        while self.gyro_sensor.angle() < angle - ANGLE_MARGE:

            if self.check_large_motor_bug():
                self.send_critical_error_msg()
                break

            wait(100)

        self.robot.stop(Stop.BRAKE)

        # compute covered angle
        self.current_value = self.gyro_sensor.angle() + ANGLE_MARGE
        
        # take measures
        measures = self.take_rotate_measure()
        # send message to computer
        self.send_state_msg(measures)

        self.thread_is_alive = False


def get_messages(topic, msg):
    '''
    Function activated when receive order from the computer

    Start the execution of the order in separate thread
    '''
    if topic == g.topic_input.encode():
        msg = str(msg.decode())

        print('input order {}'.format(msg))

        if msg == 'stop':
            g.running = False

        elif not g.thread_is_alive:
            
            g.current_value = 0

            # try to get the order type and the value of it
            try:
                g.current_order, value = msg.split(' ')
            except:
                print('Incorrect order...')
                client.publish('topic/robot', "EXEC FAILURE")
                return
            
            if g.current_order == 'move':
                g.thread = threading.Thread(target=g.move, args=[int(value)])
                g.thread.start()
            elif g.current_order == 'turn':
                g.thread = threading.Thread(target=g.turn, args=[int(value)])
                g.thread.start()

        else:
            print("Order already under execution...")
            client.publish('topic/robot', "EXEC FAILURE")

# create general object
g = General()

# connect to the computer
client = Client('robot','10.42.0.1')
client.connect()

client.set_callback(get_messages)
client.subscribe(g.topic_input)
print('Connected...')

client.publish('topic/robot', 'connected')

while g.running:
    wait(100)
    client.check_msg()

print('Cumulative angle:',g.cum_angle)
client.disconnect()
print('Disconnected.')
