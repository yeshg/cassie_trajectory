import numpy as np
from numpy import genfromtxt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.interpolate import CubicSpline

from os import listdir
from os.path import isfile, join

import time
import array
import pickle
import os

# user can change step height
def process_data(filename, speed, step_height):
    data = genfromtxt(filename,delimiter=',')
    # print(data[0,:].shape) # get length of dat
    

    # LEFT FOOT: find range, count of nans
    l_idx_nan = np.where(np.isnan(data[4,:]))[0][0] # first nan value index
    l_idx_after_nan = np.where(np.isnan(data[4,:]))[0][-1] + 1 # index right after last nan
    left_nans = np.count_nonzero(np.isnan(data[5,:])) # count nans in left foot traj

    # LEFT FOOT: touchdown points from data
    left_initial = [ data[4, l_idx_nan - 1], data[5, l_idx_nan - 1], data[6, l_idx_nan - 1] ]
    left_final = [ data[4,l_idx_after_nan], data[5,l_idx_after_nan], data[6,l_idx_after_nan] ]

    # LEFT FOOT: peak between touchdowns
    left_peak = [(left_initial[0] + left_final[0]) / 2, (left_initial[1] + left_final[1]) / 2, step_height]

    ## Cubic Spline Interpolation

    step_length = data[4,-1] - data[4,0]
    # swing_steplen = step_length / left_nans # this is the number of samples we need to fill in the nans
    swing_steplen = left_nans

    # LEFT FOOT: points to interpolate over
    l_x = [left_initial[0], left_peak[0], left_final[0]]
    l_y = [left_initial[1], left_peak[1], left_final[1]]
    l_z = [left_initial[2], left_peak[2], left_final[2]]

    # generate cubic spline (only using left foot for simplicity and to enforce symmetry)
    l_cs = CubicSpline(l_x, [l_y, l_z], axis=1)

    # indices for cubic spline 
    l_xs = np.linspace(l_x[0], l_x[2], num=swing_steplen)

    # RIGHT FOOT: find range, count of nans
    r_idx_nan = np.where(np.isnan(data[8,:]))[0][0] # first nan value index
    r_idx_after_nan = np.where(np.isnan(data[8,:]))[0][-1] + 1 # index right after last nan
    right_nans = np.count_nonzero(np.isnan(data[8,:])) # count nans in right foot traj

    # RIGHT FOOT: invisible touchdown point before data
    right_initial = [ data[7,r_idx_after_nan] - step_length, data[8,r_idx_after_nan], data[9,r_idx_after_nan] ]

    # RIGHT FOOT: touchdown point from data
    right_final = [ data[7,r_idx_after_nan], data[8,r_idx_after_nan], data[9,r_idx_after_nan] ]

    # RIGHT FOOT: peak before touch down
    right_peak_1 = [(right_final[0] - right_initial[0]) / 2, right_final[1], step_height]

    ## Cubic Spline Interpolation

    # RIGHT FOOT: points to interpolate over
    r_x = [right_initial[0], right_peak_1[0], right_final[0]]
    r_y = [right_initial[1], right_peak_1[1], right_final[1]]
    r_z = [right_initial[2], right_peak_1[2], right_final[2]]

    left_spline = np.array([l_xs, l_cs(l_xs)[0], l_cs(l_xs)[1]])
    spline_x_shift = step_length / 2
    spline_y_shift = right_final[1] - left_final[1]
    right_spline = np.array([left_spline[0] - spline_x_shift, left_spline[1] + spline_y_shift, left_spline[2]])

    ## Store data together

    # COM: store straight from original data
    com_data = np.array([data[1], data[2], data[3]])

    # LEFT FOOT: first from data, then from spline, then from data again
    left_data = data[4:7, 0:l_idx_nan]
    left_data = np.hstack((left_data, left_spline) )
    left_data = np.hstack((left_data, data[4:7, l_idx_after_nan:]))

    # RIGHT FOOT: first from spline, then all from data
    right_data = right_spline # same as left but shifted over half a period
    right_data = np.hstack((right_data, data[7:, r_idx_after_nan:]) )

    ## Store data with added offset
    # COM:
    new_com_x = np.hstack((com_data[0], com_data[0] + step_length))
    new_com_y = np.hstack((com_data[1], com_data[1]))
    new_com_z = np.hstack((com_data[2], com_data[2]))
    new_com = np.vstack((new_com_x, new_com_y, new_com_z))

    # LEFT FOOT:
    new_left_x = np.hstack((left_data[0], left_data[0] + step_length))
    new_left_y = np.hstack((left_data[1], left_data[1]))
    new_left_z = np.hstack((left_data[2], left_data[2]))
    new_left = np.vstack((new_left_x, new_left_y, new_left_z))

    # RIGHT FOOT:
    new_right_x = np.hstack((right_data[0], right_data[0] + step_length))
    new_right_y = np.hstack((right_data[1], right_data[1]))
    new_right_z = np.hstack((right_data[2], right_data[2]))
    new_right = np.vstack((new_right_x, new_right_y, new_right_z))

    # plot of multiple cycles stiched together
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(new_com[0], new_com[1], new_com[2], label='pelvis')
    ax.plot(new_left[0], new_left[1], new_left[2], label='left')
    ax.plot(new_right[0], new_right[1], new_right[2], label='right')
    ax.set_title('2 cycles of walkCycle at {0} m/s, step_height = {1:.2f}'.format(speed,step_height))
    ax.legend()
    ax.axis('equal')
    plt.savefig('plots/walkCycle_{}.png'.format(speed))

    # Stack data together (last element is the time)

    new_time = np.hstack((data[0,:], data[0,:]))

    # print(new_right.shape)
    # print(new_left.shape)
    # print(new_com.shape)
    # print(new_time.shape)

    output = np.vstack((new_right, new_left, new_com, new_time))
    output.shape

    # write to output file
    np.save('./rom_processed/rom_traj_data_{}.npy'.format(speed),output)

if __name__ == "__main__":


    onlyfiles = [f for f in listdir("walkCycles_2") if isfile(join("walkCycles_2", f))]

    speeds = [x / 10 for x in range(0, 31)]
    max_step_height = 0.15
    min_step_height = 0.1
    step_heights = [x * ((max_step_height - min_step_height) / 30) + 0.1 for x in range(0, 31)]

    print(speeds)
    print(step_heights)

    for i, speed in enumerate(speeds):
        print("speed = {0}\tstep height = {1:.2f}".format(speed, step_heights[i]))
        process_data("./walkCycles_2/walkCycle_{}.csv".format(speed), speed, step_heights[i])

