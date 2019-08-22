import ctypes
import os
import time
import numpy as np


_dir_path = os.path.dirname(os.path.realpath(__file__))

_libraries = {}
_libraries['./libcassieik.so'] = ctypes.CDLL(_dir_path + '/libcassieik.so')

# python wrapper for mujSimulation class
class mujSimulation(object):
    def __init__(self):
        _libraries['./libcassieik.so'].mujSimulation_new.argtypes = None
        _libraries['./libcassieik.so'].mujSimulation_new.restype = ctypes.c_void_p
        _libraries['./libcassieik.so'].fetch_cassie_ik.argtypes = [ctypes.c_void_p, ctypes.c_double*9 , ctypes.c_int, ctypes.c_bool]
        _libraries['./libcassieik.so'].fetch_cassie_ik.restype = ctypes.POINTER(ctypes.c_double)
        self.obj = _libraries['./libcassieik.so'].mujSimulation_new()
    def fetch_cassie_ik(self, traj_pos, steps=30, render=True):
        return _libraries['./libcassieik.so'].fetch_cassie_ik(self.obj, traj_pos, steps, render)


# # Create mujSimulation object
# sim = mujSimulation()

# # TESTING: run simulation of sample foot position and fetch output of cassie_ik into numpy array

# trajectory = [[0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.02, 0.0, 1.0],
#             [0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.04, 0.0, 1.0],
#             [0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.06, 0.0, 1.0],
#             [0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.08, 0.0, 1.0],
#             [0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.1, 0.0, 1.0],
#             [0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.12, 0.0, 1.0],
#             [0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.14, 0.0, 1.0],
#             [0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.16, 0.0, 1.0],
#             [0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.18, 0.0, 1.0],
#             [0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.2, 0.0, 1.0],
#             [0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.22, 0.0, 1.0],
#             [0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.24, 0.0, 1.0],
#             [0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.26, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.28, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.3, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.32, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.34, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.36, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.38, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.4, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.42, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.44, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.46, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.48, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.5, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.52, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.54, 0.0, 1.0],
#             [0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.56, 0.0, 1.0]]
# phase_len = 28
# wait_time = 100

# for i in range(phase_len):
#     start = time.time()

#     # qpos we want as output from ik
#     qpos = array.array('d', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
#     qpos = (ctypes.c_double * 35) (*qpos)

#     # foot positions we want to go to
#     traj_pos = array.array('d', trajectory[i])
#     # convert foot positions to c array
#     traj_pos_c = (ctypes.c_double * 9) (*traj_pos)

#     # use wrapper to get ik
#     qpos = sim.fetch_cassie_ik(traj_pos_c)

#     print(qpos[:35])