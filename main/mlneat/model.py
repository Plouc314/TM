import neat
import numpy as np
import os, pickle, math, itertools
from geometry import is_in_rect
from mlneat.helper import link_dps, clean_dps, check_for_isolated, select_sector, dist, get_cummulated_values, normalize
from fastSLAM.slam_helper import euclidean_distance, cal_direction
from specifications import Specifications as Spec


s = lambda x: x/Spec.SCALE_FACTOR
invs = lambda x: x*Spec.SCALE_FACTOR

def create_sectors_grid():
    '''Create the base sectors grid'''
    
    PADDED_SHAPE = (3*Spec.GRID_SIZE, 3*Spec.GRID_SIZE)
    MIDDLE = (PADDED_SHAPE[0]//2, PADDED_SHAPE[0]//2)

    sectors_grid = np.zeros(PADDED_SHAPE, dtype='int16')
    
    for i, angle in enumerate(np.linspace(0, 2*math.pi-Spec.DIF, Spec.N_SECTOR)):
        
        idxs = select_sector(PADDED_SHAPE, angle, Spec.DIF, MIDDLE)
        
        sectors_grid[idxs[:,0], idxs[:,1]] = i
    
    return sectors_grid

BASE_SECTORS_GRID = create_sectors_grid()

class Model:
    
    # will be set in base.py
    config = None

    def __init__(self, ge, training=True, demo=False):
        
        self.running = True
        self.is_training = training
        self.is_demo = demo

        self.position_history = []
        self.order_history = []
        self.order_count = np.zeros((4), dtype='int16')
        self.blocked_duration = 0
        
        # for room done detection
        self.dps = []

        self.ge = ge
        self.net = neat.nn.FeedForwardNetwork.create(self.ge, self.config)

        self.grid = np.zeros(Spec.GRID_SHAPE, dtype='int16')
        self.n_order = 0

        self.sectors_grid = BASE_SECTORS_GRID.copy()

    def set_plan(self, plan):
        self.plan = np.array(plan)

    def get_coord(self, position):
        # floor -> get correct index of case
        return [
            math.floor(s(position[0])),
            math.floor(s(position[1]))
        ]

    def get_sectors_grid(self, coord):
        '''Return a sectors grid adapted to the coord'''
        x = Spec.GRID_SIZE-coord[0]
        y = Spec.GRID_SIZE-coord[1]
        d_plus = int(Spec.GRID_SIZE*1.5)
        d_minus = int(Spec.GRID_SIZE*0.5)
        selected_grid = self.sectors_grid[x+d_minus:x+d_plus, y+d_minus:y+d_plus]
        return selected_grid

    def get_idxs(self, sectors_grid, n):
        '''Return the index of the selected sector'''
        idxs_x, idxs_y = np.where(sectors_grid==n)
        return np.vstack((idxs_x, idxs_y)).T

    def set_new_dps(self, dps):
        
        # if detect nothing -> loose fitness
        if len(dps) == 0:
            self.ge.fitness -= 5 
        
        self.dps.extend(dps)

        is_new_obs = False
        for dp in dps:
            try:
                # get coord of each dps
                x, y = self.get_coord(dp)

                if self.is_training or self.is_demo:
                    if self.update_grid_fitness(x,y):
                        is_new_obs = True
                
                # add new dp to grid
                self.grid[x,y] += 1
            except:
                pass

    def update_grid_fitness(self, x, y):
        '''Update the fitness according to the new dp coord'''
        
        # intermediate value, limit of the acceptable number of observations
        inter_1 = Spec.MAX_OBS // 4
        inter_2 = Spec.MAX_OBS // 2

        is_new_obs = False
        if self.grid[x,y] == 0: 
            # if it's a new obervation
            self.ge.fitness += 2
            is_new_obs = True
        elif 0 < self.grid[x,y] <= inter_1: 
            self.ge.fitness += .5
        elif inter_1 < self.grid[x,y] <= inter_2:
            self.ge.fitness -= .3
        else:
            # to avoid none moving robot
            self.ge.fitness -= 1
        return is_new_obs

    def update_pos_history(self, position):
        '''update position history, return oldest position's coord'''

        x,y = self.get_coord(position)
        if 0 < x < Spec.GRID_SIZE and 0 < y < Spec.GRID_SIZE:
            self.position_history.append((x,y))
        
        if len(self.position_history) >= 10:
            return self.position_history[:5]
        else:
            return []

    def create_inputs(self, position):
        '''Create the inputs of the neural networks'''

        # get coord of position in grid
        coord = self.get_coord(position)
        
        # create both inputs arrays
        inputs_directions = np.zeros((Spec.N_SECTOR))
        inputs_hist_pos = np.zeros((Spec.N_SECTOR))

        # update history positions and get the oldest one
        hist_pos_coords = self.update_pos_history(position)

        current_s_grid = self.get_sectors_grid(coord)
        
        for i in range(Spec.N_SECTOR): 
            
            idxs = self.get_idxs(current_s_grid, i)
            
            if idxs.ndim == 2:

                # set value of directions inputs (see helper)
                inputs_directions[i] = get_cummulated_values(self.grid, idxs, coord)

                # put history position in a sector
                for hist_pos_coord in hist_pos_coords:
                    if current_s_grid[hist_pos_coord[0], hist_pos_coord[1]] == i:
                        inputs_hist_pos[i] += 1

        if self.is_demo:
            # when testing simulation -> display representation of inputs
            self.create_inputs_views(inputs_directions, inputs_hist_pos)

        # normalize inputs -> speed up the training strongly (might be essential)
        
        inputs_directions = inputs_directions/8 #normalize(inputs_directions)

        inputs_hist_pos = normalize(inputs_hist_pos)

        return np.hstack([ inputs_directions, inputs_hist_pos ])

    def create_inputs_views(self, inputs_directions, inputs_hist_pos):
        '''Create representation of the inputs'''
        
        # create images
        self.view_dir = np.zeros((Spec.GRID_SIZE, Spec.GRID_SIZE, 3), dtype='int32')
        self.view_pos = np.zeros((Spec.GRID_SIZE, Spec.GRID_SIZE, 3), dtype='int32')

        middle = (Spec.GRID_SIZE//2, Spec.GRID_SIZE//2)

        current_s_grid = self.get_sectors_grid(middle)
        
        for i in range(Spec.N_SECTOR): 
            
            idxs = self.get_idxs(current_s_grid, i)

            if idxs.ndim == 2:
                self.view_dir[idxs[:,0], idxs[:,1],:] = inputs_directions[i] * 255
                self.view_pos[idxs[:,0], idxs[:,1],:] = inputs_hist_pos[i] * 255

    def get_predictions(self, position):
        '''Get outputs from the neural network'''
        
        inputs = self.create_inputs(position)

        outputs = self.net.activate(inputs)
        
        # get selected outputs
        output = outputs.index(max(outputs))
        
        self.update_order_history(output)

        # map outputs
        if output == 0:
            dx, dy = 1, 0 # right
        elif output == 1:
            dx, dy = -1, 0 # left
        elif output == 2:
            dx, dy = 0, -1 # up
        elif output == 3:
            dx, dy = 0, 1 # down

        # inverse scale to get position instead of coordinates - *2 -> move in a relevant way
        dx, dy = invs(dx), invs(dy)
        
        # get destination
        dest = (position[0] + dx, position[1] + dy)

        angle = cal_direction(position, dest)

        distance = euclidean_distance(position, dest)
        
        return angle, distance

    @staticmethod
    def is_in_room(plan, position):
        '''Return if the given position is in the room (plan argument)'''
        # 1 0 7 - 3 4 5
        # check if in rectangle
        a, b, c = plan[0], plan[1], plan[2]
        return is_in_rect(a, b, c, position)

    def check_finish(self):
        '''Check if the simulation covered the entire room'''
        dps = clean_dps(self.dps)
            
        try:
            lines = link_dps(dps)
        except ValueError:
            print("Can't link dps")
            return

        if not check_for_isolated(lines):
            return True

    def get_finish_bonus(self):
        '''Add a bonus according to the number of orders needed to cover the room'''
        return 5*(Spec.MAX_ORDERS - self.n_order)

    def update_order_history(self, output):
        '''
        Update order_history,  
        Check if the robot is blocked into order repetition,  
        Update fitness according to the output's values repartition  
        '''

        self.order_count[output] += 1

        if len(self.order_history) >= 2:
            # search 0, 1, 0, 1, 0, 1 pattern
            if output == self.order_history[-2] and output != self.order_history[-1]: 
                self.blocked_duration += 1
            else:
                self.blocked_duration = 0
        
        self.order_history.append(output)

    def get_fitness_orders(self):
        '''
        Add a bonus/malus depending on how equilibrated the orders were
        '''

        mse = np.sum(np.square(self.order_count-np.mean(self.order_count)))

        bonus = self.n_order - mse

        if bonus > 0:
            return 3 * bonus
        else:
            return bonus

    def get_fitness_pole(self):
        '''
        Add a bonus/malus depending on if the robot passed by the four pole
        '''
        inc = 0
        for dx, dy in itertools.product((1, -1), (1, -1)):
            passed = False
            
            for i in range(10,0, -1): # get the furthest passage
                x,y = (10 + dx*i, 10 + dy*i)
                if (x,y) in self.position_history:
                    inc += i
                    passed = True
                    break
            
            if not passed:
                inc -= 4
        
        return 4*inc

    def update_fitness_end(self):
        '''
        Update fitness,  
        Add orders and pole bonus/malus  
        '''
        self.ge.fitness += self.get_fitness_orders()
        self.ge.fitness += self.get_fitness_pole()

    def update_running_state(self, position):
        '''When training, update running state according to position, output, number of orders'''
        
        if not self.is_in_room(self.plan, position): # if out of room -> stop
            print('out', self.n_order)
            self.ge.fitness -= 300
            self.running = False
        elif np.max(self.grid) > Spec.MAX_OBS: # avoid none moving robot
            print('obs', self.n_order)
            self.ge.fitness -= 250
            self.running = False
        elif self.blocked_duration == 5:
            print('block', self.n_order)
            self.ge.fitness -= 250
            self.running = False

        # check if done
        if self.ge.fitness > 150:
            if self.check_finish():
                # add a base bonus for having covered the entire room
                self.ge.fitness += 100
                # add a supplement to the fitness func (according to the number of order given)
                self.ge.fitness += self.get_finish_bonus()
                self.running = False

    def run(self, position, collisions):
        self.n_order += 1

        self.set_new_dps(collisions)
        
        angle, distance = self.get_predictions(position)
        
        if self.is_training or self.is_demo:
            self.update_running_state(position)
                
        return angle, distance
    
    def store_grid(self):
        '''Store the raw datapoints as well as the grid into .pickle files'''

        with open(os.path.join('data','dps.pickle'),'wb') as file:
            pickle.dump(self.dps, file)

        with open(os.path.join('data','grid.pickle'),'wb') as file:
            pickle.dump(self.grid, file)