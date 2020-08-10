import neat
import numpy as np
import os, pickle
from multiprocessing import Process, Queue, Manager
from planGenerator import Generator
from simulation import MLSimulation
from mlneat.model import Model
from specifications import Specifications as Spec


# imported from https://github.com/techwithtim/NEAT-Flappy-Bird/blob/master/flappy_bird.py
# Determine path to configuration file. This path manipulation is
# here so that the script will run successfully regardless of the
# current working directory.
local_dir = os.path.dirname(__file__)
config_path = os.path.join(local_dir, 'config-feedforward.txt')

config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                    neat.DefaultSpeciesSet, neat.DefaultStagnation,
                    config_path)
####

POS_START = (Spec.WINDOW[0]//2,Spec.WINDOW[0]//2)

# set some attributes to model class (avoid recursive import)
Model.config = config

class Train:
    '''
    Static Object.  
    Train the neural network using NEAT.  
    Use train method.
    '''
    # store mean and best fitness of each generation
    generation_results = []
    @classmethod
    def set_plans(cls, plans):
        cls.plans = []
        for plan in plans:
            # verify that the starting position is in the room
            if Model.is_in_room(plan, POS_START):
                cls.plans.append(plan)
        cls.plan_idx = 0

    @classmethod
    def create_pop(cls):
        
        print('creating population...')
        # Create the population, which is the top-level object for a NEAT run.
        p = neat.Population(config)

        # Add a stdout reporter to show progress in the terminal.
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)

        
        print('store pop...')
        # save pop
        with open(os.path.join('data','pop.pickle'),'wb') as file:
            pickle.dump(p, file)
        
        return p

    @classmethod
    def load_pop(cls):
        with open(os.path.join('data','pop.pickle'),'rb') as file:
            p = pickle.load(file)
        return p

    @staticmethod    
    def eval_genomes(genomes, config):
        simuls = []
        models = []
        plan = Train.plans[round(Train.plan_idx)]
        for i, genome in genomes:
            genome.fitness = 0  # start with fitness level of 0
            # run each generation on a diferent plan
            model = Model(genome)
            simul = MLSimulation(model, POS_START, plan)
            simuls.append(simul)
            models.append(model)
        
        Train.plan_idx += .25 # train 4 times on each plan

        n_order = 0
        running = True
        # train
        while running:
            n_order += 1
            
            to_remove = []

            for simul in simuls:
                simul.run()
                if not simul.running:
                    to_remove.append(simul)

            for simul in to_remove:
                simuls.remove(simul)
            
            if n_order == Spec.MAX_ORDERS:
                running = False
        
        # add bonus/malus of end state
        for model in models:
            model.update_fitness_end()

        # get mean/max of fitness
        fitnesses = []
        for model in models:
            fitnesses.append(model.ge.fitness)

        mean = np.mean(fitnesses)
        best = np.max(fitnesses)

        Train.generation_results.append((mean, best))
   
    @classmethod
    def train(cls, generation, filename_genome='gen.pickle'):
        '''
        Train for the given number of generations,  
        Store the fitness stats in data/fitness.pickle,  
        Store the resulting genome in data/filename_genome,  
        '''
        p = cls.create_pop()

        generator = Generator(Spec.WINDOW)
        plans = generator.load_plans()

        cls.set_plans(plans)

        print('start training...')
        # start training
        winner = p.run(cls.eval_genomes, generation)

        # save mean fitness progression
        with open(os.path.join('data', 'fitness.pickle'), 'wb') as file:
            pickle.dump(cls.generation_results, file)

        # save as pickle object
        with open(os.path.join('data',filename_genome), 'wb') as file:
            pickle.dump(winner, file)






    