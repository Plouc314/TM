from scipy.spatial.distance import cdist
import numpy as np
import pandas as pd
import itertools

#####
# inspired from https://codereview.stackexchange.com/questions/28207/finding-the-closest-point-to-a-list-of-points
def closest_point(point, points):
    """ Find closest point from a list of points. """
    index = cdist([point], points).argmin()
    return points.iloc[index,:], points.iloc[[index,index-1],:].index[0]
#####

def find_closest_dps(idx, dps):
    
    dp = dps.iloc[idx,:]
    c1dp, c1idx = closest_point(dp, dps.drop(idx,axis=0))

    c2dp, c2idx = closest_point(dp, dps.drop([idx, c1idx],axis=0))
    return c1dp, c2dp

def build_slines_from_unique(df_dps, unique):
    slines = []
    for name1, name2 in unique:
        dp1 = df_dps.iloc[name1,:]
        dp2 = df_dps.iloc[name2,:]
        slines.append((dp1, dp2))
    return slines

def drop_same_lines(slines, dps):
    names = []
    for sline in slines:
        cname = [sline[0].name, sline[1].name]
        cname.sort()
        names.append(cname)

    names = np.array(names)
    unique = np.unique(names, axis=0)
    slines = build_slines_from_unique(dps, unique)
    return slines

def link_dps(dps):
    df_dps = pd.DataFrame(dps, columns=['x','y'])
    slines = []
    for dp in df_dps.iterrows():
        idx, dp = dp
        # find closest dps
        cdp1, cdp2 = find_closest_dps(idx, df_dps)
        slines.append((dp,cdp1))
        slines.append((dp,cdp2))
    slines = drop_same_lines(slines, df_dps)
    return slines

def check_for_isolated(slines):
    '''Check if one of the dps is linked  3 times'''
    # get the name of 
    names = []
    for sline in slines:
        cname = [sline[0].name, sline[1].name]
        cname.sort()
        names.append(cname)
    names = np.array(names)
    
    unique, counts = np.unique(names, return_counts=True)
    # check if a dp is linked 3 times
    return 3 in counts

from sklearn.cluster import KMeans

def clean_dps(dps):
    '''Reduce the number of dps to 1/20 while keeping the informations'''
    n_clusters = len(dps)//20
    kmeans = KMeans(n_clusters=n_clusters)

    kmeans.fit(dps)
    clean_dps = kmeans.cluster_centers_
    return clean_dps

def get_dps(arr):
    '''Get the indexs of all the cases where a least one observation was made'''
    idxs_x, idxs_y = np.where(arr!=0)
    return np.vstack((idxs_x, idxs_y)).T

#####
# inspired from https://stackoverflow.com/questions/13652518/efficiently-find-points-inside-a-circle-sector

def is_clockwise(dp1, dp2):
    return -dp1[0]*dp2[1] + dp1[1]*dp2[0] > 0

def is_counter_clockwise(dp1, dp2):
    return -dp1[0]*dp2[1] + dp1[1]*dp2[0] < 0

def is_in_sector(start, end, dp):
    return is_clockwise(end, dp) and is_counter_clockwise(start, dp)

#####

def normalize(x):
    total = np.sum(x)
    if total != 0:
        return 1/total * x
    else:
        return x

def select_sector(shape, angle, angle_dif, position):
    '''
    For a 2d array of a given size,  
    return the indexs of the value that are in the sector,  
    the sector is specified by the start angle (angle), the difference between the start an the end (dif_angle) and it's center (position)
    '''
    start = [
        np.cos(angle),
        np.sin(angle)
    ]

    end = [
        np.cos(angle + angle_dif),
        np.sin(angle + angle_dif)
    ]
        
    idxs = []
    
    for x, y in itertools.product(range(shape[0]), range(shape[1])):
        dp = (x - position[0], y - position[1])
        if is_in_sector(start, end, dp):
            idxs.append((x,y))

    return np.array(idxs)

def dist(dp0, dp1):
    '''Return the distance between two datapoints'''
    return np.sqrt((abs(dp0[0] - dp1[0]))**2 + (abs(dp0[1] - dp1[1]))**2)

from specifications import Specifications as Spec

def get_cummulated_values(grid, idxs, position):
    '''Sum the selected values of the grid while taking account the distance'''
    

    z_grid = np.zeros(Spec.GRID_SHAPE)

    # set grid values of idxs in z_grid
    z_grid[idxs[:,0], idxs[:,1]] = grid[idxs[:,0], idxs[:,1]]

    # get idxs of none zeros value
    x, y = np.where(z_grid > 0)

    dp_idxs = np.vstack([x,y]).T

    # check if there is a least one none zeros value -> otherwise: no observation
    if dp_idxs.size > 0:
        # get center of none zeros values
        center = np.mean(dp_idxs, 0)

        # get the total value of the sector
        total = np.sum(grid[idxs[:,0], idxs[:,1]])
        
        # check that the observation sector is not on the robot position
        if (center != position).any():
            distance = dist(center, position)
            return total / distance
        else:
            return total
    return 0

