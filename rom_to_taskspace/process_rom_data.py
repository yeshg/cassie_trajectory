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
def process_data(filename, speed, step_height, useMinJerk = True, td_vel = -0.1):
    print("File name: " + filename)
    data = genfromtxt(filename,delimiter=',',skip_header=1)
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
    swing_duration = data[0, l_idx_after_nan-1] - data[0, l_idx_nan]

    if useMinJerk:
        left_spline_full = use_min_jerk(left_initial, left_peak, left_final, swing_steplen, td_vel)
        left_spline = left_spline_full[0:3]
        left_spline_vel = left_spline_full[3:6]/(swing_duration)
    else:
        left_spline = use_cubic_spline(left_initial, left_peak, left_final, swing_steplen)
        


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

    spline_x_shift = step_length / 2
    spline_y_shift = right_final[1] - left_final[1]
    right_spline = np.array([left_spline[0] - spline_x_shift, left_spline[1] + spline_y_shift, left_spline[2]])
    right_spline_vel = left_spline_vel
    ## Store data together

    # COM: store straight from original data
    com_data = np.array([data[1], data[2], data[3]])

    com_data_vel = np.array([data[10], data[11], data[12]])


    # LEFT FOOT: first from data, then from spline, then from data again
    left_data = data[4:7, 0:l_idx_nan]
    left_data = np.hstack((left_data, left_spline) )
    left_data = np.hstack((left_data, data[4:7, l_idx_after_nan:]))

    left_data_vel = 0*data[4:7, 0:l_idx_nan]
    left_data_vel = np.hstack((left_data_vel, left_spline_vel) )
    left_data_vel = np.hstack((left_data_vel, 0*data[4:7, l_idx_after_nan:]))

    # RIGHT FOOT: first from spline, then all from data
    right_data = right_spline # same as left but shifted over half a period
    right_data = np.hstack((right_data, data[7:10, r_idx_after_nan:]) )

    right_data_vel = right_spline_vel
    right_data_vel = np.hstack((right_data_vel, 0*data[7:10, r_idx_after_nan:]))

    ## Store data with added offset
    # COM:
    new_com_x = np.hstack((com_data[0], com_data[0] + step_length))
    new_com_y = np.hstack((com_data[1], com_data[1]))
    new_com_z = np.hstack((com_data[2], com_data[2]))
    new_com = np.vstack((new_com_x, new_com_y, new_com_z))

    new_com_vel = np.hstack((com_data_vel, com_data_vel))

    # LEFT FOOT:
    new_left_x = np.hstack((left_data[0], left_data[0] + step_length))
    new_left_y = np.hstack((left_data[1], left_data[1]))
    new_left_z = np.hstack((left_data[2], left_data[2]))
    new_left = np.vstack((new_left_x, new_left_y, new_left_z))

    new_left_vel = np.hstack((left_data_vel, left_data_vel))

    # RIGHT FOOT:
    new_right_x = np.hstack((right_data[0], right_data[0] + step_length))
    new_right_y = np.hstack((right_data[1], right_data[1]))
    new_right_z = np.hstack((right_data[2], right_data[2]))
    new_right = np.vstack((new_right_x, new_right_y, new_right_z))

    new_right_vel = np.hstack((right_data_vel, right_data_vel))


    # concatenate the times together
    new_time = np.hstack((data[0,:], data[0,-1] + data[0,:]))

    # plot of multiple cycles stiched together
    fig = plt.figure(figsize=(10,20))
    ax = fig.add_subplot(311, projection='3d')

    ax.plot(new_com[0], new_com[1], new_com[2], label='pelvis')
    ax.plot(new_left[0], new_left[1], new_left[2], label='left')
    ax.plot(new_right[0], new_right[1], new_right[2], label='right')
    ax.set_title('2 cycles of walkCycle at {0} m/s, step_height = {1:.2f}'.format(speed,step_height))
    ax.legend()
    ax.axis('equal')

    ax2 = fig.add_subplot(312)
    ax2.plot(new_time, new_com[0])
    ax2.plot(new_time, new_com[1])
    ax2.plot(new_time, new_com[2])
    ax2.plot(new_time, new_right[0])
    ax2.plot(new_time, new_right[1])
    ax2.plot(new_time, new_right[2])
    ax2.plot(new_time, new_left[0])
    ax2.plot(new_time, new_left[1])
    ax2.plot(new_time, new_left[2])

    ax2 = fig.add_subplot(313)
    ax2.plot(new_time, new_com_vel[0])
    ax2.plot(new_time, new_com_vel[1])
    ax2.plot(new_time, new_com_vel[2])
    ax2.plot(new_time, new_right_vel[0])
    ax2.plot(new_time, new_right_vel[1])
    ax2.plot(new_time, new_right_vel[2])
    ax2.plot(new_time, new_left_vel[0])
    ax2.plot(new_time, new_left_vel[1])
    ax2.plot(new_time, new_left_vel[2])

    plt.savefig('plots/walkCycle_{}.png'.format(speed))

    # Stack data together (last element is the time)


    # print(new_right.shape)
    # print(new_left.shape)
    # print(new_com.shape)
    # print(new_time.shape)

    # output = np.vstack((new_time, new_com, new_com_vel, new_right, new_right_vel, new_left, new_left_vel))
    output = np.vstack((new_time, new_right, new_left, new_com, new_right_vel, new_left_vel, new_com_vel))
    print("output shape: {}".format(output.shape))

    # write to output file

    np.savetxt('./animationROMData/rom_traj_data_{}.csv'.format(speed), output, delimiter=',')
    np.save('./rom_processed/rom_traj_data_{}.npy'.format(speed),output)


def use_cubic_spline(pos_initial, pos_peak, pos_final, n_points):
    
    # LEFT FOOT: points to interpolate over
    l_x = [pos_initial[0], pos_peak[0], pos_final[0]]
    l_y = [pos_initial[1], pos_peak[1], pos_final[1]]
    l_z = [pos_initial[2], pos_peak[2], pos_final[2]]

    # generate cubic spline (only using left foot for simplicity and to enforce symmetry)
    l_cs = CubicSpline(l_x, [l_y, l_z], axis=1)

    #get cubic spline derivative, if needed finish implementing this
    dl_cs = l_cs.derivative()

    # indices for cubic spline 
    l_xs = np.linspace(l_x[0], l_x[2], num=n_points)

    left_spline = np.array([l_xs, l_cs(l_xs)[0], l_cs(l_xs)[1]])

    # import ipdb; ipdb.set_trace()
    return left_spline

def use_min_jerk(init_point, mid_point, final_point, n_points, td_vel):
    from minJerkFootTrajectories import min_jerk_foot_trajectories

    data = min_jerk_foot_trajectories( init_point, final_point, n_points, mid_point[2], td_vel)

    # import ipdb; ipdb.set_trace()
    return np.transpose(data[:,1:])

if __name__ == "__main__":


    onlyfiles = [f for f in listdir("ImprovedCost") if isfile(join("ImprovedCost", f))]

    speeds = [x / 10 for x in range(0, 21)]
    max_step_height = 0.2
    min_step_height = 0.2
    step_heights = [x * ((max_step_height - min_step_height) / 30) + min_step_height for x in range(0, 31)]

    print(speeds)
    print(step_heights)

    for i, speed in enumerate(speeds):
        print("speed = {0}\tstep height = {1:.2f}".format(speed, step_heights[i]))
        process_data("./ImprovedCost/walkCycle_{}.csv".format(speed), speed, step_heights[i], useMinJerk = True, td_vel = -0.3)

