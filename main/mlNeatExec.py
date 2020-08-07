import neat
import numpy as np
import os, pickle, time
from simulation import MLSimulation
from planGenerator import Generator
from mlneat.base import Train, config, Model, POS_START
from interface import Interface, Robot, Const, C

def use_simulation(plan_idx, filename='gen.pickle'):
    '''
    Test the current best genome on a chosen plan
    '''
    # load plan
    g = Generator(Const['WINDOW'])
    g.load_data()

    # setup Interface
    Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

    Interface.set_robot(Robot(POS_START, 0))

    Interface.set_plan(g.data[plan_idx])

    Interface.keep_track_mov(C.YELLOW)

    # load genome
    with open(os.path.join('data',filename),'rb') as file:
        genome = pickle.load(file)

    genome.fitness = 0

    model = Model(genome, training=False, demo=True)

    simulation = MLSimulation(model, graphics=True)

    while Interface.running:
        pressed, events = Interface.run()

        simulation.run()

        Interface.info_board.set_n_order(simulation.model.n_order)
        Interface.info_board.set_max_obs(np.max(simulation.model.grid))
        Interface.info_board.set_fitness(simulation.model.ge.fitness)
        Interface.info_board.set_running(simulation.running)

        time.sleep(.4)

    simulation.model.store_grid()


#Train.train(100)

use_simulation(6, filename='gen1_s400_i10.pickle')