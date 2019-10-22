import numpy as np
from numpy import genfromtxt

from scipy.interpolate import CubicSpline

def process_data(filename, speed):
    data = genfromtxt(filename,delimiter=',')
    # print(data[0,:].shape) # get length of dat


    # LEFT FOOT: find range, count of nans
    l_idx_nan = np.where(np.isnan(data[4,:]))[0][0]
    l_idx_after_nan = np.where(np.isnan(data[4,:]))[0][-1] + 1
    left_nans = np.count_nonzero(np.isnan(data[5,:])) # count nans in right foot traj

    # RIGHT FOOT: find range, count of nans
    r_idx_nan = np.where(np.isnan(data[8,:]))[0][0]
    r_idx_after_nan = np.where(np.isnan(data[8,:]))[0][-1] + 1
    right_nans = np.count_nonzero(np.isnan(data[8,:])) # count nans in right foot traj

    # print("Left nan range:  ({}, {}). Count: {}".format(l_idx_nan, l_idx_after_nan, left_nans))
    # print("Right nan range: ({}, {}). Count: {}".format(r_idx_nan, r_idx_after_nan, right_nans))

    ## Cubic Spline Interpolation

    step_length = data[4,-1] - data[4,0]
    swing_steplen = step_length / left_nans # this is the number of samples we need to fill in the nans
    step_height = 0.1 # user can change this


    # LEFT FOOT: touchdown points from data
    left_initial = [ data[4, l_idx_nan - 1], data[5, l_idx_nan - 1], data[6, l_idx_nan - 1] ]
    left_final = [ data[4,l_idx_after_nan], data[5,l_idx_after_nan], data[6,l_idx_after_nan] ]

    # LEFT FOOT: peak between touchdowns
    left_peak = [(left_initial[0] + left_final[0]) / 2, (left_initial[1] + left_final[1]) / 2, step_height]

    # RIGHT FOOT: invisible touchdown point before data
    right_initial = [ data[7,r_idx_after_nan] - step_length, data[8,r_idx_after_nan], data[9,r_idx_after_nan] ]

    # RIGHT FOOT: touchdown point from data
    right_final = [ data[7,r_idx_after_nan], data[8,r_idx_after_nan], data[9,r_idx_after_nan] ]

    # RIGHT FOOT: peak before touch down
    right_peak_1 = [(right_final[0] - right_initial[0]) / 2, right_final[1], step_height]


    # LEFT FOOT: points to interpolate over
    l_x = [left_initial[0], left_peak[0], left_final[0]]
    l_y = [left_initial[1], left_peak[1], left_final[1]]
    l_z = [left_initial[2], left_peak[2], left_final[2]]

    # RIGHT FOOT: points to interpolate over
    r_x = [right_initial[0], right_peak_1[0], right_final[0]]
    r_y = [right_initial[1], right_peak_1[1], right_final[1]]
    r_z = [right_initial[2], right_peak_1[2], right_final[2]]

    # print(r_x)

    # generate cubic spline (only using left foot for simplicity and to enforce symmetry)
    l_cs = CubicSpline(l_x, [l_y, l_z], axis=1)
    r_cs = CubicSpline(r_x, [r_y, r_z], axis=1)

    # indices for cubic spline 
    l_xs   = np.arange(l_x[0], l_x[2], swing_steplen)
    r_xs   = np.arange(r_x[0], r_x[2], swing_steplen)
    # print(r_xs.shape)



    ## Store data together

    # COM: store straight from original data
    com_data = np.array([data[1], data[2], data[3]])

    # LEFT FOOT: first from data, then from spline, then from data again
    left_data = data[4:7, 0:l_idx_nan]
    # print(left_data.shape)
    left_data = np.hstack((left_data, np.array([l_xs, l_cs(l_xs)[0], l_cs(l_xs)[1]])) )
    # print(left_data.shape)
    left_data = np.hstack((left_data, data[4:7, l_idx_after_nan:]))
    # print(left_data.shape)

    # RIGHT FOOT: first from spline, then all from data
    right_data = np.array([r_xs, r_cs(r_xs)[0], r_cs(r_xs)[1]]) # same as left but shifted over half a period
    # print(right_data.shape)
    right_data = np.hstack((right_data, data[7:, r_idx_after_nan:]) )
    # print(right_data.shape)


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

    # Check for stepping motion
    # print(new_com.shape)
    # print(new_left.shape)
    # print(new_right.shape)


    # Stack data together (last element is the time)

    new_time = np.hstack((data[0,:], data[0,:]))
    sin = np.sin(new_time * 2 * np.pi)
    cos = np.cos(new_time * 2 * np.pi)

    output = np.vstack((new_right, new_left, new_com, new_time))
    output.shape

    # write to output file
    np.save('./rom_processed/rom_traj_data_{}.npy'.format(speed),output)

from os import listdir
from os.path import isfile, join

onlyfiles = [f for f in listdir("walkCycles") if isfile(join("walkCycles", f))]

ignore_files = ['walkCycle_0.6']

speeds = np.arange(0.0, 3.0, 0.1)
print(speeds)
for i in range(len(onlyfiles)):
    print(speeds[i])
    process_data("./walkCycles/{}".format(onlyfiles[i]), speeds[i])

