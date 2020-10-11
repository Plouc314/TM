from random import randint, random
import numpy as np
from geometry import rotate, area as get_area
from math import pi
import pickle

PATH = 'data/room.pickle'

class Generator:
    '''
    Generator of rooms, can have either: 4, 6 or 8 corners

    Methods: 
        - generate_plans
        - load_plans
    '''
    def __init__(self, dimension, MARGIN=160):
        # store the postion of the dps, split in two list, one for each side (SW - NE) 
        self.dps = [[],[]]
        self.dim = dimension
        self.MARGIN = MARGIN
        
    def create_seeds(self, coef_marge=1.5):
        marge = self.MARGIN * coef_marge
        seed1 = [randint(marge, self.dim[0]//3 - marge),randint(marge, self.dim[1]//3 - marge)]
        seed2 = [randint(self.dim[0]*2//3, self.dim[0] - marge), randint(self.dim[1]*2//3, self.dim[0] - marge)]
        self.dps[0].append(seed1)
        self.dps[1].append(seed2)
    
    def generate4(self):
        dp = [self.dps[0][0][0], self.dps[1][0][1]]
        self.dps[0].append(dp)
        dp = [self.dps[1][0][0], self.dps[0][0][1]]
        self.dps[1].append(dp)
        return self.return_result()


    def create_way(self,dp1, dp2, direction, limit=False):
        '''
        Create 3 datapoints between dp1 and dp2

        Arguments:
            dp1 (x,y): datapoint in NW
            dp2 (x,y): datapoint in SE
            direction (bool): if True way go in SW, if False in NE
            limit (x,y):  
        '''
        rand_bool = bool(randint(0,1))
        if direction: # true -> SW
            # create the two random datapoints 
            if rand_bool:
                # check for limit
                if limit:
                    rdp1_rlimit = randint(limit[1] + self.MARGIN, self.dim[1]-2*self.MARGIN)
                    rdp2_rlimit = randint(dp1[0] + self.MARGIN, limit[0] - self.MARGIN)
                else:
                    # pick random coordinate in limit
                    rdp1_rlimit = randint(dp1[1] + self.MARGIN, self.dim[1]-2*self.MARGIN)
                    rdp2_rlimit = randint(dp1[0] + self.MARGIN, dp2[0] - self.MARGIN)
            else:
                if limit:
                    rdp1_rlimit = randint(limit[1] + self.MARGIN, dp2[1]-self.MARGIN)
                    rdp2_rlimit = randint(2*self.MARGIN, limit[0] - self.MARGIN)
                else:
                    # pick random coordinate in limit
                    rdp1_rlimit = randint(dp1[1] + self.MARGIN, dp2[1]-self.MARGIN)
                    rdp2_rlimit = randint(2*self.MARGIN, dp2[0] - self.MARGIN)
                
            rdp1 = [dp1[0], rdp1_rlimit]
            rdp2 = [rdp2_rlimit ,dp2[1]]
        else: # false -> NE
            # create the two random datapoints 
            if rand_bool:
                if limit:
                    rdp1_rlimit = randint(2*self.MARGIN, limit[1] - self.MARGIN)
                    rdp2_rlimit = randint(limit[0] + self.MARGIN, dp2[0] - self.MARGIN)
                else:
                    # pick random coordinate in limit
                    rdp1_rlimit = randint(2*self.MARGIN, dp2[1] - self.MARGIN)
                    rdp2_rlimit = randint(dp1[0] + self.MARGIN, dp2[0] - self.MARGIN)
                
            else:
                if limit:
                    rdp1_rlimit = randint(dp1[1] + self.MARGIN, limit[1] - self.MARGIN)
                    rdp2_rlimit = randint(limit[0] + self.MARGIN, self.dim[0] - 2*self.MARGIN)
                else:
                    # pick random coordinate in limit
                    rdp1_rlimit = randint(dp1[1] + self.MARGIN, dp2[1] - self.MARGIN)
                    rdp2_rlimit = randint(dp1[0] + self.MARGIN, self.dim[0] - 2*self.MARGIN)

            rdp1 = [dp2[0], rdp1_rlimit]
            rdp2 = [rdp2_rlimit ,dp1[1]]
                
        
        ldp = [rdp2[0],rdp1[1]]
        return [rdp1, ldp, rdp2]


    def generate6(self):
        way = self.create_way(self.dps[0][0], self.dps[1][0], True)
        self.dps[0].extend(way)
        self.dps[1].append([self.dps[1][0][0], self.dps[0][0][1]])
        return self.return_result()

    def generate8(self):
        # create_way() with limit fail ~10% of the time so try again in this case
        done = False
        while not done:
            try:
                way_SW = self.create_way(self.dps[0][0], self.dps[1][0], True)
                way_NE = self.create_way(self.dps[0][0], self.dps[1][0], False, limit=way_SW[1])
                done = True
            except:
                pass
        self.dps[0].extend(way_SW)
        self.dps[1].extend(way_NE)
        return self.return_result()

    def return_result(self):
        result = self.dps[0]
        result.extend(self.dps[1])
        return result
    
    @staticmethod
    def rotate(plan):
        rotated_plan = []
        for dp in plan:
            rotated_plan.append(dp[::-1])
        return rotated_plan

    def generate(self, ncorner):
        
        self.dps = [[],[]]
        
        if ncorner == 4:
            # create seeds
            self.create_seeds()
            return self.generate4()
        elif ncorner == 6:
            # create seeds
            self.create_seeds()
            return self.generate6()
        elif ncorner == 8:
            # create seeds
            self.create_seeds()
            return self.generate8()

    def load_plans(self):
        '''
        Load previously generated plans
        '''
        with open(PATH,'rb') as file:
            plans = pickle.load(file)
        
        return plans

    def generate_plans(self, n, nwall):
        '''
        Generate n plans with a given number of corners, each plan is rotated of a random angle  
        Store the generated plans in a .pickle  
        Return the generated plans  
        '''
        plans = []
        for _ in range(n):
            plan = self.generate(nwall)
            angle = 2*pi * random()
            plans.append(rotate(plan, angle))

        with open(PATH,'wb') as file:
            pickle.dump(plans, file)
        
        return plans

def get_mean_area(plans, scale_factor=200):
    '''Return the mean area of the given plans'''

    plans = np.array(plans)
    plans = plans / scale_factor
    areas = np.zeros((plans.shape[0]))

    for i in range(plans.shape[0]):
        area = get_area(plans[i,0], plans[i,1], plans[i,2])
        areas[i] = area
    
    return np.mean(areas)

if __name__ == '__main__':

    g = Generator((1600,1600))

    plans = g.generate_plans(200, 4)

    print(f'meters: {get_mean_area(plans):.1f}, pixels: {get_mean_area(plans, 1):.0f}')

    


