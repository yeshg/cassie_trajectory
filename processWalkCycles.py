from cassiemujocoik_ctypes import *
import time
import array
import numpy as np
import pickle
import os


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
    def rom_trajectory_ik_interpolate(self, spline_params, step_size=0.001, speedup = 3):
        
        # calculate length based on step_size
        # points = np.arange(0, 1, step_size * speedup)
        points = np.linspace(0, 1, 1682 * 2)
        new_points = np.transpose(np.array(splev(points, spline_params)))
        print("New points shape = {}".format(new_points.shape))
        length = points.shape[0]
        
        traj_qpos = np.zeros((length, 35))
        
        # variables to hold current phase, time, integer index counter
        time = 0
        
        for i in range(length):    
            # pass traj data from spline into ik
            traj_qpos[i] += self.single_pos_ik(new_points[i])
            # increment time
            time += step_size * speedup
            # debug
            print("time: {}  idx: {}".format(time, i))
            
        # Now we gotta estimate the qvels using finite difference estimates of d qpos / dt
        # only do this for the motor positions
        motor_indices = [7, 8, 9, 14, 20, 21, 22, 23, 28, 34]
        traj_qvel = np.zeros((length, len(motor_indices)))
        for i in range(len(traj_qpos)):
            traj_qvel[i] += np.take((traj_qpos[i] - traj_qpos[i - 1]) / (step_size * speedup), motor_indices)

            
        print("COM Z", new_points[:,2])
        print("left Z", new_points[:-1])
        # calculate distance between feet and center of mass, append to trajectory info
        right_foot = new_points[:,3:6] - new_points[:,6:9]
        left_foot = new_points[:,0:3] - new_points[:,6:9]
            
        return traj_qpos, traj_qvel, right_foot, left_foot

if __name__ == "__main__":

    speeds = [x / 10 for x in range(0, 21)]
    max_step_height = 0.15
    min_step_height = 0.1
    step_heights = [x * ((max_step_height - min_step_height) / 30) + 0.1 for x in range(0, 31)]

    print(speeds)
    print(step_heights)

    for i, speed in enumerate(speeds):
        # if speed == 1.4 or speed == 1.8 or speed == 1.9 or speed == 2.4 or speed == 2.5 or speed == 2.6 or speed == 3.0:
            # continue
        print("speed = {0}\tstep height = {1:.2f}".format(speed, step_heights[i]))
        rom_trajectory = np.load("./rom_to_taskspace/rom_processed/rom_traj_data_{}.npy".format(speed))
        task_trajectory = rom_trajectory[0:9]

        # generate splines out of task trajecotry
        from scipy.interpolate import splprep, splev

        tck, u = splprep(task_trajectory, s=0)
        new_points = splev(u, tck)

        task_trajectory_time = rom_trajectory[9:]
        print('Showing Trajectory')
        g = input("Enter when ready : ") 
        cassie = CassieIK(sim_steps=1, render_sim=True)
        traj_qpos, traj_qvel, right_foot, left_foot = cassie.rom_trajectory_ik_interpolate(tck, step_size=0.0001, speedup=3)

        full_trajectory = {"qpos": traj_qpos[(traj_qpos.shape[0] // 2):] \
                        , "qvel": traj_qvel[(traj_qvel.shape[0] // 2):] \
                        , "rfoot": right_foot[(right_foot.shape[0] // 2):] \
                        , "lfoot": left_foot[(left_foot.shape[0] // 2):]}
        with open("trajectory/aslipTrajsTaller/walkCycle_{}.pkl".format(speed), "wb") as f:
            pickle.dump(full_trajectory, f)
            print("wrote pickle file")
