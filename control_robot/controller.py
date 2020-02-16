import paho.mqtt.client as mqtt



client = mqtt.Client()
client.connect("localhost",1883,60)



class General:
    '''
    Contient toutes les variables

    Comme les actions dépendent des messages reçus, toutes les fonctions sont exécutées dans
    la fonction on_message. Pour changer les variables de scope global dans la fonction
    on_message, toutes les variables sont déclarées comme attribut de l'object General.
    La fonction give_order() l'est aussi parce que sela évite de devoir passer tout plein
    d'attributs comme argument de la fonction.
    '''

    msg = ''
    received = False
    # liste des ordres qui vont être donnés
    orders = [['turn',30],['move',20],['turn',120],['move',20],['turn',120],['move',20],['turn',90]]
    index = 1

    def give_order(self):
        '''
        Envoie l'ordre au robot.
        '''
        if self.index < len(self.orders):
            order = self.orders[self.index][0] + ' ' + str(self.orders[self.index][1])
            self.index += 1
            client.publish("topic/order", order)
            print('give order: {}'.format(order))
        else:
            print('disconnecting...')
            client.publish("topic/order", 'stop')
            client.disconnect()

g = General()

# donne le premier ordre
client.publish("topic/order", self.orders[0][0] + ' ' + str(self.orders[0][1]))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("topic/robot")

def on_message(client, userdata, msg):
    '''
    Quand un message (output) est reçu, donne le prochain ordre
    '''
    g.msg = msg.payload.decode()
    print('input data: {}'.format(g.msg))
    g.give_order()
        

client.on_connect = on_connect
client.on_message = on_message

# boucle infini qui attends de recevoir un message (output)
client.loop_forever()

print('home disconnected')

