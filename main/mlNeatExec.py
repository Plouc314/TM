import neat
import numpy as np
import os, pickle, time
from simulation import MLSimulation
from planGenerator import Generator
from mlneat.train import Train, Model
from interface import Interface, Robot, Const, C
from specifications import Specifications as Spec
import pygame

def break_in_simul():
    '''Start a infinite loop until Enter is pressed'''
    while True:
        pressed, events = Interface.run()
        if pressed[pygame.K_ESCAPE]:
            return

def use_simulation(plan_idx, genome_filename='gen.pickle', frame_time=.5, with_fastslam=False, stop_when_finish=True):
    '''
    Test the genome on a chosen plan
    '''
    # load plan
    g = Generator(Const['WINDOW'])
    plans = g.load_plans()

    # setup Interface
    Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

    Interface.set_robot(Robot(Spec.POS_START, 0))

    Interface.set_plan(plans[plan_idx])

    Interface.keep_track_mov(C.YELLOW)

    # load genome
    with open(os.path.join('data',genome_filename),'rb') as file:
        genome = pickle.load(file)

    genome.fitness = 0

    model = Model(genome,demo=True)

    simulation = MLSimulation(model, graphics=True, with_fastslam=with_fastslam)

    while True:
        pressed, events = Interface.run()
        if not Interface.running:
            break

        simulation.run()

        Interface.info_board.set_n_order(simulation.model.n_order)
        Interface.info_board.set_max_obs(np.max(simulation.model.grid))
        Interface.info_board.set_fitness(simulation.model.ge.fitness)
        Interface.info_board.set_running(simulation.running, simulation.success)

        if stop_when_finish:
            if not simulation.running:
                # start a break to visualize the final situation
                break_in_simul()
                break

        time.sleep(frame_time)

    simulation.model.store_grid()

def test_simulation(plan_slice, genome_filename, with_fastslam=False):
    # load plan
    g = Generator(Const['WINDOW'])
    plans = g.load_plans()

    # select plans according to slice
    plans = plans[plan_slice]

    # load genome
    with open(os.path.join('data',genome_filename),'rb') as file:
        genome = pickle.load(file)

    # store number of success
    n_success = 0

    for i, plan in enumerate(plans):

        print(i)

        genome.fitness = 0

        model = Model(genome)

        simulation = MLSimulation(model, Spec.POS_START, with_fastslam=with_fastslam, plan=plan)

        while simulation.running:
            simulation.run()
        
        if simulation.success:
            n_success += 1
    
    return n_success / len(plans) # return ratio of success

#Train.train(120, filename_genome='gen4_s400_i50.pickle')

use_simulation(12, genome_filename='gen2_s400_i50.pickle', frame_time=.2, with_fastslam=False)

#r = test_simulation(slice(0,200), genome_filename='gen2_s400_i50.pickle')
#
#print(r)