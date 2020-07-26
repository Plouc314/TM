import neat
import os, pickle, time
from simulation import MLSimulation
from planGenerator import Generator
from mlneat.base import Train, config, Model
from interface import Interface, Robot, Const, C

def use_simulation(plan_idx):
    '''
    Test the current best genome on a chosen plan
    '''
    # load plan
    g = Generator(Const['WINDOW'])
    g.load_data()

    # setup Interface
    Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

    Interface.set_robot(Robot((600,600), 0))

    Interface.set_plan(g.data[plan_idx])

    Interface.keep_track_mov(C.YELLOW)

    # load genome
    with open(os.path.join('data','gen.pickle'),'rb') as file:
        genome = pickle.load(file)

    model = Model(genome, training=False)

    simulation = MLSimulation(model, graphics=True)

    while Interface.running:
        pressed, events = Interface.run()

        simulation.run()

        time.sleep(.5)


#Train.train()
use_simulation(0)