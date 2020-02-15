#!/usr/bin/env pybricks-micropython

from time import sleep, time


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
    Contient toutes les variables

    Comme les actions dépendent des messages reçus, toutes les fonctions sont exécutées dans
    la fonction get_messages. Pour être sûr que les variables sont dans le scope de la fonction
    get_messages, toutes les variables sont déclarées comme attribut de l'object General.
    '''
    def __init__(self):
        self.running = True
        self.topic_input = 'topic/order'
        self.input = []
        self.move_speed = 100
        self.turn_speed = 50
        self.cum_angle = 0

        # Déclare les moteurs et capteurs
        self.motor1 = Motor(Port.B)
        self.motor2 = Motor(Port.C)
        self.robot = DriveBase(self.motor1, self.motor2, 56, 121)
        self.ultra_sensor = UltrasonicSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S4)
        print('initializated')

g = General()

def move(distance):
    '''
    Fait avancer le robot

    Le robot dévie toujours en démarrant, le gyroscope mesure la déviation qui est ajoutée à
    cum_angle qui stocke l'angle résultant des déviations cumulées.

    drive_time() n'est pas utilisé vu que le capteur à ultrason mesure en permanence la distance
    pour pouvoir s'arrêter en cas d'obstacle.

    Après quelques essais, il se trouve que le robot n'avance pas assez loin (de quelques millimètres)
    pour compenser self.move_speed/2000 est ajouté à la distance cible

    Retourne:
        vrai/faux selon qu'un object soit détecté ou pas
        la distance parcourue
    '''
    self.gyro_sensor.reset_angle(0)
    self.robot.drive(self.move_speed, 0)
    start_time = time()
    while time() - start_time < distance/10 + self.move_speed/2000:
        if self.ultra_sensor.distance() > 300:
            wait(10)
        else:
            self.robot.stop(Stop.BRAKE)
            print('object detected')
            
            self.cum_angle += self.gyro_sensor.angle()
            
            # (time() - start_time)*10 est la distance couverte avant la détection
            return True, (time() - start_time)*10

    self.robot.stop(Stop.BRAKE)
    self.cum_angle += self.gyro_sensor.angle()
    return False, distance
    
def turn(angle):
    '''
    Fait tourner le robot

    drive_time() n'est pas utilisé pour la même raison que dans move().
    
    De base le robot tourne trop, parce que le temps que le robot freine il a déjà dépassé l'angle cible,
    du coup, self.turn_speed/10 est enlevé à l'angle cible (après quelques essais ça marche pas mal).

    Retourne:
        vrai/faux selon qu'un object soit détecté ou pas
        l'angle tourné
    '''
    self.gyro_sensor.reset_angle(0)
    self.robot.drive(0, self.turn_speed)
    while self.gyro_sensor.angle() < angle - self.turn_speed/10:
        if self.ultra_sensor.distance() > 300:
            print(self.ultra_sensor.distance())
            wait(10)
        else:
            self.robot.stop(Stop.BRAKE)
            print('object detected')
            
            # return the distance covered before the detection
            return True, self.gyro_sensor.angle() - self.turn_speed/10
    self.robot.stop(.BRAKE)
    return True, angle

# connecte le robot à l'ordinateur
client = Client('robot','10.42.0.1')
client.connect()

def get_messages(topic, msg):
    '''
    En fonction des messages (ordres) reçus, exécute les fonctions demandées et envoie 
    un message à l'ordinateur (le résultat)

    Commence par décoder l'ordre et tester si c'est 'stop', 'move' ou 'turn'
    selon l'ordre le robot va avancer ou tourner, 
    dans les deux cas une valeur est donné dans l'ordre (par ex: 'move 20')

    Envoie un output, qui contient:
        normal/detected selon qu'un object est été détecté ou pas
        la distance/angle parcourue
    '''
    if topic == g.topic_input.encode():
        g.input = str(msg.decode())
        print('input order {}'.format(g.input))
        # attends pour que le robot soit stoppé
        sleep(0.5)
        if g.input == 'stop':
            g.running = False
        else:
            g.input = g.input.split(' ')
            # exécute l'ordre
            if g.input[0] == 'move':
                object_detected, distance = move(int(g.input[1]))
                if object_detected:
                    output = 'detected {:.1f}'.format(distance)
                else:
                    output = 'normal {:.1f}'.format(distance)
            else:
                object_detected, angle = turn(int(g.input[1]))
                if object_detected:
                    output = 'detected {:.1f}'.format(angle)
                else:
                    output = 'normal {:.1f}'.format(angle)
            
            # envoie l'output
            client.publish("topic/robot", output)


client.set_callback(get_messages)
client.subscribe(g.topic_input)
print('connected')

# boucle infinie qui attends les messages de l'ordinateur
while g.running:
    client.check_msg()
    sleep(0.1)

print('cumulative angle:',g.cum_angle)
client.disconnect()
print('disconnected')