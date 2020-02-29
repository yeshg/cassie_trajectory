"""
Script for generating a center of mass position and velocity trajectory to follow. This is to be used as a reward signal for a policy that already knows how to walk and match speeds.

Strategy for doing this is continuing to update a state variable (position, heading, instantaneous velocity) according to the past states' velocities, headings. 
"""

import math
import time
import array
import numpy as np
import pickle
import os

import matplotlib.pyplot as plt
from matplotlib import animation

if __name__ == "__main__":

    # total number of steps
    max_traj_len = 800

    # frequency of position 
    frequency = 30
    dt = 1 / frequency

    # exponential moving average factor (like mass in momentum)
    momentum = 0.9

    # Velocity Change
    min_speed = 0.0
    max_speed = 2.0

    # Orientation Change
    max_orient_deviation = math.pi
    # d_orient = 0.1 # maybe use this? maybe moving average is enough

    # set up initial state
    poses = [[0.0,0.0]]
    headings = [0.0]
    velociites = [0.0]  # "forward" velocity
    current = (0,0,0,0) # (x,y,theta,|v|)

    for i in range(max_traj_len):

        # extract current xy position from current state.
        origin = [current[0],current[1]]
        heading = current[2]
        # use this to get current total displacement
        h = np.sqrt(origin[0] ** 2 + origin[1] ** 2)

        # choose a random speed, orientation within the ranges
        speed = np.random.uniform(low=min_speed, high=max_speed)
        orient = np.random.uniform(low=-max_orient_deviation, high=max_orient_deviation)

        # calculate displacement from current position by integrating velocity
        dh = speed * dt
        # use this to calculate commanded vel vector for this step
        x = (h + dh) * np.cos(heading) - origin[0]
        y = (h + dh) * np.sin(heading) - origin[1]

        # update current state
        current = (x, y, heading, speed)

        # plot commanded forward velocity and orientation from current position/orientation
        plt.quiver(origin[0], origin[1], x, y)



    plt.show()