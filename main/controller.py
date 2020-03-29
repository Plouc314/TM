from fastSLAM.fast_slam import FastSlam
import paho.mqtt.client as mqtt
from time import sleep
from math import pi

class Connection:
    recieved = False
    msg = 'move 0 0'

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
        self.recieved = True
    
    def on_disconnect(client, userdata,rc=0):
        self.client.loop_stop()

    def send_order(self, order):
        self.client.publish("topic/order", order)

    def stop(self):
        self.client.publish("topic/order", 'stop')
        self.client.disconnect()
        print('disconnected...')


class BaseController:
    done = True
    def __init__(self, pos, alpha):
        self.conn = Connection()
        self.pos = pos
        self.alpha = alpha
        self.fastslam = FastSlam(pos[0], pos[1], alpha)
    
    def send_order(self, order):
        self.done = False
        self.conn.send_order(order)

    def update(self, mov, obs):
        if obs == 255: # max range of sensor
            obs = []
        else:
            obs = [(5*obs, self.fastslam.robot.orientation)]

        #print('fastslam: {} {}'.format(mov, obs))
        self.fastslam(mov, obs)

        self.pos = self.fastslam.get_mean_pos()
        self.alpha = self.fastslam.robot.orientation

    def handeln_data(self, data):
        print('Incoming msg:', data)
        self.conn.recieved = False
        data = data.split(' ')
        if len(data) == 4:
            self.done = True

        if data[0] == 'off':
            return
        elif data[0] == 'move':
            mov = [5*float(data[1]), 0]
        elif data[0] == 'turn':
            mov = [0, float(data[1]) * 3.14/180]
        
        obs = float(data[2])/10

        self.update(mov, obs)
        
    

