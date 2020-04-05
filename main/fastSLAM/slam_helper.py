import numpy as np
import math
import random
from scipy import linalg

from time import time

def timer(func):
    def inner(*args, **kwargs):
        start = time()
        result = func(*args, **kwargs)
        print('{}: {:.2f}s'.format(func.__name__, time()-start))
        return result
    return inner

WINDOWWIDTH, WINDOWHEIGHT = 1600, 1600

def gauss_noise(mu, sig):
    return random.gauss(mu, sig)

def euclidean_distance(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def cal_direction(a, b):
    """Calculate the angle of the vector a to b"""
    return math.atan2(b[1]-a[1], b[0]-a[0])

def multi_normal(x, mean, cov):
    """Calculate the density for a multinormal distribution"""
    den = 2 * math.pi * math.sqrt(linalg.det(cov))
    num = np.exp(-0.5*np.transpose((x - mean)).dot(linalg.inv(cov)).dot(x - mean))
    result = num/den
    return result[0][0]

def sense_direction(robot_pos, landmark, noise):
    """Measures the direction of the landmark with respect to robot. Add noise"""
    direction = cal_direction(robot_pos, (landmark[0], landmark[1]))
    angle_noise = gauss_noise(0, noise*math.pi/180)
    if direction + angle_noise > math.pi:
        result = direction + angle_noise - 2*math.pi
    elif direction + angle_noise < - math.pi:
        result = direction + angle_noise + 2*math.pi
    else:
        result = direction + angle_noise
    return result

# imported from https://pythonrobotics.readthedocs.io/en/latest/modules/slam.html

def normalize_weight(particles, particles_size):

    sumw = sum([p.weight for p in particles])

    try:
        for i in range(particles_size):
            particles[i].weight /= sumw
    except ZeroDivisionError:
        for i in range(particles_size):
            particles[i].weight = 1.0 / particles_size

        return particles

    return particles

def resampling(particles, particles_size):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles, particles_size)

    pw = []
    for i in range(particles_size):
        pw.append(particles[i].weight)

    pw = np.array(pw)

    Neff = 1.0 / (pw @ pw.T)  # Effective particle number
    

    #if Neff < particles_size/1.5:  # resampling
    if random.random() < .1: # do it 10% of the time because of for some reasons(?) the other test is always False
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / particles_size) - 1 / particles_size
        resampleid = base + np.random.rand(base.shape[0]) / particles_size

        inds = []
        ind = 0
        for ip in range(particles_size):
            while ((ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind])):
                ind += 1
            inds.append(ind)

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i].pos_x = tparticles[inds[i]].pos_x
            particles[i].pos_y = tparticles[inds[i]].pos_y
            #particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].weight = 1.0 / particles_size

    
    return particles


