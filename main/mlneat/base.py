import neat
import numpy as np
import pandas as pd
import os, pickle, math
from multiprocessing import Process, Queue, Manager
from planGenerator import Generator
from simulation import MLSimulation
from interface import Interface, Const
from mlneat.helper import get_max_multiple, get_dps, link_dps, check_for_isolated
from fastSLAM.slam_helper import euclidean_distance, cal_direction

# Determine path to configuration file. This path manipulation is
# here so that the script will run successfully regardless of the
# current working directory.
local_dir = os.path.dirname(__file__)
config_path = os.path.join(local_dir, 'config-feedforward.txt')

config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                    neat.DefaultSpeciesSet, neat.DefaultStagnation,
                    config_path)


GRID_SIZE = 20
MAX_ORDERS = 20
SCALE_FACTOR = Const['WINDOW'][0] / GRID_SIZE

s = lambda x: x/SCALE_FACTOR
invs = lambda x: x*SCALE_FACTOR

class Model:
    def __init__(self, ge, training=True):
        
        self.running = True
        self.is_training = training
        self.previous_pos = None
        
        self.ge = ge
        self.net = neat.nn.FeedForwardNetwork.create(self.ge, config)

        self.grid = pd.DataFrame(np.zeros((GRID_SIZE,GRID_SIZE)))
        self.n_order = 0

    def set_plan(self, plan):
        self.plan = np.array(plan)

    def set_new_dps(self, dps):
        
        # if detect nothing -> lose
        if len(dps) == 0:
            self.ge.fitness -= 5 
        
        is_new_obs = False
        for dp in dps:
            # get coord of each dps
            # floor -> get correct index of case
            x = math.floor(s(dp[0]))
            y = math.floor(s(dp[1]))

            if self.is_training:
                if self.update_grid_fitness(x,y):
                    is_new_obs = True
            
            # add new dp to grid
            self.grid.iloc[x,y] += 1
        
        if not is_new_obs:
            # promote to observe new dps
            self.ge.fitness -= 1

    def update_grid_fitness(self, x, y):
        '''Update the fitness according to the new dp coord'''
        is_new_obs = False
        if self.grid.iloc[x,y] == 0: 
            # if it's a new obervation
            self.ge.fitness += 2
            is_new_obs = True
        elif 0 < self.grid.iloc[x,y] < 5: 
            self.ge.fitness += .3
        elif 5 < self.grid.iloc[x,y] < 10:
            self.ge.fitness -= .3
        else:
            # to avoid none moving robot
            self.ge.fitness -= 1
        return is_new_obs

    def get_predictions(self, position):
        '''Get inputs from neural network'''
        inputs = self.grid.values.flatten()
        
        # get robot position in grid
        x = s(position[0])
        y = s(position[1])
        
        '''Inputs: values of the flatten grid (grid size²) robot position (2)'''
        '''Outputs: a flatten 2d array of size (grid size²)'''

        # add robot infos to flatten grid
        inputs = np.append(inputs, [x,y])
        outputs = self.net.activate(inputs)
        
        # get selected case
        idx = outputs.index(max(outputs))
        
        # get case coord
        dest_y = get_max_multiple(GRID_SIZE,idx)
        dest_x = idx - GRID_SIZE*dest_y

        # inverse scale to get position instead of coordinates
        dest = (invs(dest_x), invs(dest_y))

        angle = cal_direction(position, dest)

        distance = euclidean_distance(position, dest)
        
        return angle, distance

    def check_finish(self):
        '''Check if the simulation covered the entire room'''
        dps = get_dps(self.grid)
        if len(dps) > 50:
            df_dps = pd.DataFrame(dps, columns=['x','y'])
            
            try:
                lines = link_dps(df_dps)
            except ValueError:
                print("can't link dps")
                return

            if not check_for_isolated(lines):
                return True

    def check_in_room(self, position):
        if position[0] < np.min(self.plan[:,0]) or position[0] > np.max(self.plan[:,0]):
            return False
        if position[1] < np.min(self.plan[:,1]) or position[1] > np.max(self.plan[:,1]):
            return False
        return True

    def update_running_state(self):
        '''When training, update running state according to position, output, number of orders'''
        # check if done
        if self.check_finish():
            # add a supplement to the fitness func (according to the number of order given)
            self.ge.fitness += (MAX_ORDERS - self.n_order) * 5 + 50
            self.running = False
        elif not self.check_in_room(position): # if out of room -> stop
            self.ge.fitness -= 300
            self.running = False
        elif position == self.previous_pos:
            self.ge.fitness -= 300
            self.running = False

    def run(self, position, collisions):
        self.n_order += 1

        self.set_new_dps(collisions)
        
        angle, distance = self.get_predictions(position)
        
        if self.is_training:
            self.update_running_state()
        
            self.previous_pos = position.copy()

        return angle, distance
    
    def store_grid(self):
        with open(os.path.join('data','grid.pickle'),'wb') as file:
            pickle.dump(self.grid, file)


class Train:
    @classmethod
    def set_plans(cls, plans):
        cls.plans = plans
        cls.plan_idx = 3

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
        for i, genome in genomes:
            genome.fitness = 0  # start with fitness level of 0
            # run each generation on a diferent plan
            model = Model(genome)
            simul = MLSimulation(model, Train.plans[Train.plan_idx])
            simuls.append(simul)
        
        #Train.plan_idx += 1
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
            
            if n_order == MAX_ORDERS:
                running = False

    @staticmethod
    def eval_batch(simuls, q=None):
        '''Evalue fitness func on batch of simulations, use queue object'''
        n_order = 0
        running = True
        done_simuls = [] # contains all simuls that finish session
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
                done_simuls.append(simul)
            
            if n_order == MAX_ORDERS:
                running = False
        
        # put remaining simuls in done_simuls
        for simul in simuls:
            done_simuls.append(simul)
        
        if q:
            q.put([done_simuls])

        return

    @staticmethod
    def eval_genomes_mp(genomes, config):
        '''Same as eval_genomes but with multiprocessing'''
        
        simuls = []
        for i, genome in genomes:
            genome.fitness = 0  # start with fitness level of 0
            # run each generation on a diferent plan
            model = Model(genome)
            simul = MLSimulation(model, Train.plans[Train.plan_idx])
            simuls.append(simul)
        
        # split simulations in batchs: each batch is evalued in a separated process
        b_size = 50
        n_batch = len(simuls) // b_size

        # the neat algorithm can change the number of genomes in pop -> not always 200 members
        if n_batch * b_size != len(simuls):
            batch = simuls[:n_batch*b_size-1:-1]
            Train.eval_batch(batch)

        manager = Manager() # if queue not created from manager -> process never stop(?)
        processes = []
        queues = []
        # create processes
        for i in range(n_batch):
            q = manager.Queue()
            batch = simuls[b_size*i:b_size*(i+1)]
            p = Process(target=Train.eval_batch, args=[batch, q])
            processes.append(p)
            queues.append(q)
            p.start()

        for i in range(n_batch):
            processes[i].join()
            eval_simuls = queues[i].get()[0]
            batch = simuls[b_size*i:b_size*(i+1)]
            # as object manipulated in other process aren't reference of original ones -> set fitness
            for e, simul in enumerate(batch):
                simul.model.ge.fitness = eval_simuls[e].model.ge.fitness
        
    @classmethod
    def train(cls):
        p = cls.create_pop()

        generator = Generator(Const['WINDOW'])
        generator.load_data()

        cls.set_plans(generator.data)

        print('start training...')
        # start training
        winner = p.run(cls.eval_genomes_mp, 30)

        # save as pickle object
        with open(os.path.join('data','gen.pickle'), 'wb') as file:
            pickle.dump(winner, file)



if __name__ == '__main__':
    
    # train the neural networks
    Train.train()




    