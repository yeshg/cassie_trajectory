from cassiemujocoik_ctypes import *

import time
import array

def plot_joint_pos(qpos_arr):
    plt.plot(qpos_arr)

class CassieIK(object):
    def __init__(self, sim_steps=30, render_sim=True):
        self.sim = mujSimulation()
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
        qpos = self.sim.fetch_cassie_ik(traj_pos_arr_c, steps=self.sim_steps, render=self.render_sim)

        return qpos[:35]

    def trajectory_ik(self, trajectory):
        qpos_arr = []
        for i in range(len(trajectory)):
            qpos_arr.append(single_pos_ik(trajectory[i]))
        return qpos_arr


trajectory = [[0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.02, 0.0, 1.0],
            [0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.04, 0.0, 1.0],
            [0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.06, 0.0, 1.0],
            [0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.08, 0.0, 1.0],
            [0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.1, 0.0, 1.0],
            [0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.12, 0.0, 1.0],
            [0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.14, 0.0, 1.0],
            [0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.16, 0.0, 1.0],
            [0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.18, 0.0, 1.0],
            [0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.2, 0.0, 1.0],
            [0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.22, 0.0, 1.0],
            [0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.24, 0.0, 1.0],
            [0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.26, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.28, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.3, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.32, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.34, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.36, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.38, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.4, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.42, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.44, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.46, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.48, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.5, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.52, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.54, 0.0, 1.0],
            [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.56, 0.0, 1.0]]

cassie = CassieIK()

print(trajectory[0])

plot_joint_pos(cassie.trajectory_ik(trajectory))