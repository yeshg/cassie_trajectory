import ctypes
import os
import time
import numpy as np
import array

_dir_path = os.path.dirname(os.path.realpath(__file__))

_libraries = {}
_libraries['./libmujsimulation.so'] = ctypes.CDLL(_dir_path + '/libmujsimulation.so')

# python wrapper for mujSimulation class
class mujSimulation(object):
    def __init__(self):
        _libraries['./libmujsimulation.so'].mujSimulation_new.argtypes = None
        _libraries['./libmujsimulation.so'].mujSimulation_new.restype = ctypes.c_void_p
        _libraries['./libmujsimulation.so'].fetch_cassie_ik.argtypes = [ctypes.c_void_p, ctypes.c_double*9 ]
        _libraries['./libmujsimulation.so'].fetch_cassie_ik.restype = ctypes.POINTER(ctypes.c_double)
        self.obj = _libraries['./libmujsimulation.so'].mujSimulation_new()
    def fetch_cassie_ik(self, traj_pos):
        return _libraries['./libmujsimulation.so'].fetch_cassie_ik(self.obj, traj_pos)


# Create mujSimulation object
sim = mujSimulation()

# TESTING: run simulation of sample foot position and fetch output of cassie_ik into numpy array
for i in range(1000):
    start = time.time()

    # qpos we want as output from ik
    qpos = array.array('d', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    qpos = (ctypes.c_double * 35) (*qpos)

    # foot positions we want to go to
    traj_pos = array.array('d', [0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.38, 0.0, 1.0])
    # convert foot positions to c array
    traj_pos_c = (ctypes.c_double * 9) (*traj_pos)

    # use wrapper to get ik
    qpos = sim.fetch_cassie_ik(traj_pos_c)

    print(qpos)