import neat
import numpy as np
import os, pickle, time
from simulation import MLSimulation
from planGenerator import Generator
from mlneat.base import Train, config, Model, POS_START
from interface import Interface, Robot, Const, C

def use_simulation(plan_idx, filename='gen.pickle', frame_time=.5, with_fastslam=False):
    '''
    Test the genome on a chosen plan
    '''
    # load plan
    g = Generator(Const['WINDOW'])
    plans = g.load_plans()

    # setup Interface
    Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

    Interface.set_robot(Robot(POS_START, 0))

    Interface.set_plan(plans[plan_idx])

    Interface.keep_track_mov(C.YELLOW)

    # load genome
    with open(os.path.join('data',filename),'rb') as file:
        genome = pickle.load(file)

    genome.fitness = 0

    model = Model(genome, training=False, demo=True)

    simulation = MLSimulation(model, graphics=True, with_fastslam=with_fastslam)

    while True:
        pressed, events = Interface.run()
        if not Interface.running:
            break

        simulation.run()

        Interface.info_board.set_n_order(simulation.model.n_order)
        Interface.info_board.set_max_obs(np.max(simulation.model.grid))
        Interface.info_board.set_fitness(simulation.model.ge.fitness)
        Interface.info_board.set_running(simulation.running)

        time.sleep(frame_time)

    simulation.model.store_grid()


#Train.train(120, filename_genome='gen2_s400_i50.pickle')

use_simulation(30, filename='gen2_s400_i50.pickle', frame_time=.2, with_fastslam=False)