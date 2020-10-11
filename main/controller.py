from fastSLAM.fast_slam import FastSlam
from interface import Interface
import paho.mqtt.client as mqtt
from time import sleep
from math import pi

class Connection:
    '''
    Handeln connection and send/receive message
    '''
    has_new_msg = False
    msg = None

    def __init__(self):
        self.client = mqtt.Client()
        self.client.connect("localhost",1883,60)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.loop_start()
        sleep(.5) # waiting for connection

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe("topic/robot")

    def on_message(self, client, userdata, msg):
        self.msg = msg.payload.decode()
        self.has_new_msg = True
    
    def on_disconnect(client, userdata,rc=0):
        self.client.loop_stop()

    def send_order(self, order):
        self.client.publish("topic/order", order)

    def stop(self):
        self.client.publish("topic/order", 'stop')
        self.client.disconnect()
        print('Disconnected...')


class BaseController:
    
    connected = False
    order_pending = False
    robot_error = False
    mov = None
    obs = None

    def __init__(self, robot=None):
        self.conn = Connection()

        if robot:
            self.robot = robot
        else:
            self.robot = Interface.robot
        
        self.fastslam = FastSlam(self.robot.x, self.robot.y, self.robot.orien)

    @property
    def has_new_msg(self):
        return self.conn.has_new_msg

    def send_order(self, order):
        self.order_pending = True
        self.conn.send_order(order)
    
    def run_fastslam(self):

        if self.mov == None or self.obs == None:
            return

        self.fastslam.run(self.mov, self.obs)

        # reset variables in case of error/incorrect order
        self.mov, self.obs = None, None

        self.robot.set_pos(self.fastslam.get_mean_pos(), center=True, scale=True)
        self.robot.set_orien(self.fastslam.get_mean_orien())

    def decrypt_robot_state(self, msg):
        '''
        Extract the order, movement value (distance/angle) and all the observations (distance, angle from robot).  
        Scale the movement value and observations.  
        Return the mov variable (see fastslam), observations.
        '''
        lines = msg.split('\n')

        # line 0 contains the order and mov val 
        order, value = lines[0].split(' ')
        
        # scale the movement value
        if order == 'turn':
            # get angle in radian
            angle = int(value) * pi/180
            mov = [0, angle]
        
        elif order == 'move':
            # distance in cm: 1px = 0.5 cm
            distance = int(value) * 2
            mov = [distance, 0]

        # other lines contains observations
        obs = [line.split(' ') for line in lines[1:]]

        # scale the observations
        for i in range(len(obs)):
            # scale distance knowing that 200 px = 1 meter, 1 px = 5 mm
            distance = int(obs[i][0]) / 5
            angle = int(obs[i][1]) * pi/180
            obs[i] = [distance, angle]

        return mov, obs

    def handeln_msg(self):
        '''
        Handeln incoming message from robot.  
        Store informations in relevant attributes.  
        '''
        # reset has_new_msg val -> can receive other msg
        self.conn.has_new_msg = False

        print(f'[ROBOT] {self.conn.msg}')

        if self.conn.msg == 'connected':
            self.connected = True

        elif self.conn.msg == 'ERROR': # check if an error occured
            self.robot_error = True

        elif self.conn.msg == "EXEC FAILURE": # check if order can be executed
            self.order_pending = False

        else:
            # order is done
            self.order_pending = False
            # get robot movement data
            self.mov, self.obs = self.decrypt_robot_state(self.conn.msg)

            
