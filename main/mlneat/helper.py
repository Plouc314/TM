from scipy.spatial.distance import cdist
import numpy as np
import pandas as pd

def closest_point(point, points):
    """ Find closest point from a list of points. """
    index = cdist([point], points).argmin()
    return points.iloc[index,:], points.iloc[[index,index-1],:].index[0]

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
    slines = []
    for dp in dps.iterrows():
        idx, dp = dp
        # find closest dps
        cdp1, cdp2 = find_closest_dps(idx, dps)
        slines.append((dp,cdp1))
        slines.append((dp,cdp2))
    slines = drop_same_lines(slines, dps)
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

def get_dps(df):
    size = df.shape[0]
    idx = df.replace(0,np.NaN).notna()
    dps = []
    for i in range(size):
        s = df[i][idx[i]]
        if s.size != 0:
            for e in s.index.values:
                dp = [i, size-e]
                dps.append(dp)
    dps = np.array(dps)
    return dps

def get_max_multiple(multiple, n):
    i = 0
    while i * multiple <= n:
        i += 1
    
    return i-1
