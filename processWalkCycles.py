from cassiemujocoik_ctypes import *
import time
import array
import numpy as np
import pickle
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CassieIK(object):
    def __init__(self, sim_steps=1, render_sim=True):
        self.sim = mujSimulation(render_sim)
        self.sim_steps = sim_steps
        self.render_sim = render_sim
        # qpos we want as output from ik
        self.qpos = array.array('d', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.qpos = (ctypes.c_double * 35) (*self.qpos)

    def single_pos_ik(self, traj_pos):
        # foot position we want to go to
        traj_pos_arr = array.array('d', traj_pos)
        # convert foot positions to c array
        traj_pos_arr_c = (ctypes.c_double * 9) (*traj_pos_arr)

        # use wrapper to get ik
        qpos = self.sim.fetch_cassie_ik(traj_pos_arr_c, steps=self.sim_steps)

        return np.array(qpos[:35])
    
    # version of above function for trajectories from aslip rom
    def rom_trajectory_ik_interpolate(self, spline_params, time_spline, end_time, time_new_points, speed, frequency=30): # frequency is the Hz we should sample from


        N = int(np.round(end_time*frequency))
        time_points = np.linspace(0,end_time,N+1)[0:-1]
        # get times for subsampling at specified frequency
        # time_points = np.arange(0, end_time, step=(1/frequency))
        
        # Total length of 30 Hz trajectory
        # N = time_points.shape[0]
        

        print(N)
        # input()

        # initialize traj_qpos, taskspace_points
        traj_qpos = np.zeros((N, 35))
        
        i_list = []

        for idx in range(N):
            # # get index by finding closest matching time in time_new_points
            # i = (np.abs(time_new_points - time_points[idx])).argmin()

            # get index by evaluating time spline at current time and converting to integer index
            # print(splev(time_points[idx], time_spline))
            i = int(splev(time_points[idx], time_spline)[1])

            i_list.append(i)
            # print("time: {}  idx: {}  i: {}".format(time_points[idx], idx, i))

        # points for splev
        points = np.linspace(0, 1, np.max(i_list)+1)

        # use spline to get taskspace trajectory vector for this index, pass through IK
        taskspace_points = np.transpose(np.array(splev(points, spline_params)))
        # print(taskspace_points.shape)

        # print(max(i_list))

        # Step through time at specified frequency and call IK to fill up traj_qpos
        idx= 0
        for i in i_list:
            traj_qpos[idx] += self.single_pos_ik(taskspace_points[i])
            idx += 1

        # for i_cycles in range(9):
        #     for i in i_list:
        #         self.single_pos_ik(taskspace_points[i,:] + (i_cycles+1)*(taskspace_points[-1,:] - taskspace_points[0,:]))

        
        # fig = plt.figure(figsize=(10,10))
        # ax = fig.add_subplot(111)
        # ax.plot(traj_qpos)
        # plt.savefig('foo.png')
        # plot of multiple cycles stiched together
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(taskspace_points[i_list,0], taskspace_points[i_list,1], taskspace_points[i_list,2], 'o-', label='right')
        ax.plot(taskspace_points[i_list,3], taskspace_points[i_list,4], taskspace_points[i_list,5], 'o-', label='left')
        ax.plot(taskspace_points[i_list,6], taskspace_points[i_list,7], taskspace_points[i_list,8], 'o-', label='pelvis')
        ax.set_title('1 cycles of walkCycle at {0} m/s'.format(speed))
        ax.legend()
        ax.axis('equal')
        plt.savefig('downsampledPlots/walkCycle_{}.png'.format(speed))

        # DON"T USE THE QVEL INFO THAT MUCH ITS NOT THAT ACCURATE
        # Now we gotta estimate the qvels using finite difference estimates of d qpos / dt
        # only do this for the motor positions
        motor_indices = [7, 8, 9, 14, 20, 21, 22, 23, 28, 34]
        traj_qvel = np.zeros((N, len(motor_indices)))
        for i in range(len(traj_qpos)):
            traj_qvel[i] += np.take((traj_qpos[i] - traj_qpos[i - 1]) / (1 / frequency), motor_indices)

            
        # print("COM Z", taskspace_points[:,2])
        # print("left Z", taskspace_points[:-1])
        # calculate distance between feet and center of mass, append to trajectory info
        right_foot = taskspace_points[:,3:6] - taskspace_points[:,6:9]
        left_foot = taskspace_points[:,0:3] - taskspace_points[:,6:9]
            
        return traj_qpos, traj_qvel, right_foot, left_foot, (2*np.pi / N)

if __name__ == "__main__":

    frequency = 30

    speeds = [x / 10 for x in range(1, 31)]
    max_step_height = 0.2
    min_step_height = 0.2
    step_heights = [x * ((max_step_height - min_step_height) / 30) + min_step_height for x in range(0, 31)]

    clock_incs = []

    print(speeds)
    print(step_heights)
    g = input("Enter when ready")

    for i, speed in enumerate(speeds):
        # if speed == 1.4 or speed == 1.8 or speed == 1.9 or speed == 2.4 or speed == 2.5 or speed == 2.6 or speed == 3.0:
            # continue
        print("speed = {0}\tstep height = {1:.2f}".format(speed, step_heights[i]))
        rom_trajectory = np.load("./rom_to_taskspace/rom_processed/rom_traj_data_{}.npy".format(speed))

        # # get first half of rom_trajectory (one cycle and not two)
        rom_trajectory = rom_trajectory[:,0:int( rom_trajectory.shape[1]/2)]

        task_trajectory = rom_trajectory[0:9]

        # get relationship between time data and linear baseline for resampling
        linear_baseline = np.arange(0, rom_trajectory[-1].shape[0])
        time_trajectory = np.vstack((rom_trajectory[-1], linear_baseline))

        # generate splines out of task trajecotry so when we sub-sample to 30 Hz we don't land in between data points
        from scipy.interpolate import splprep, splev

        tck, u = splprep(task_trajectory, s=0)
        new_points = splev(u, tck)

        time_tck, time_u = splprep(time_trajectory, s=0)
        time_new_points = splev(time_u, time_tck)

        # # plot of time
        # fig = plt.figure(figsize=(10,10))
        # ax = fig.add_subplot(111)
        # ax.plot(linear_baseline, rom_trajectory[-1])
        # ax.plot(time_new_points[1], time_new_points[0])
        # ax.legend()
        # plt.show()

        print('Showing Trajectory')
        # g = input("Enter when ready : ")
        cassie = CassieIK(sim_steps=1, render_sim=True)
        traj_qpos, traj_qvel, right_foot, left_foot, clock_inc = cassie.rom_trajectory_ik_interpolate(tck, time_tck, rom_trajectory[-1][-1], time_new_points[0], speed, frequency=frequency)

        clock_incs.append(clock_inc)

        full_trajectory = {"qpos": traj_qpos
                        , "qvel": traj_qvel
                        , "rfoot": right_foot
                        , "lfoot": left_foot
                        , "clock_inc": clock_inc}
        with open("trajectory/aslipTrajsSweep/walkCycle_{}.pkl".format(speed), "wb") as f:
            pickle.dump(full_trajectory, f)
            print("wrote pickle file")

    print(clock_incs)