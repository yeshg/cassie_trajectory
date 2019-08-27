import numpy as np
import random
import os

class CassieTrajectory:
    def __init__(self, filepath):
        n = 1 + 35 + 32 + 10 + 10 + 10
        data = np.fromfile(filepath, dtype=np.double).reshape((-1, n))

        dirname = os.path.dirname(__file__)
        foot_pos = np.load(os.path.join(dirname, "ref_foot_pos.npy"))
        foot_vel = np.load(os.path.join(dirname, "ref_foot_vel.npy"))

        # states
        self.time = data[:, 0]
        self.qpos = data[:, 1:36]
        self.qvel = data[:, 36:68]

        self.foot_pos = foot_pos
        self.foot_vel = foot_vel

        # actions
        self.torque = data[:, 68:78]
        self.mpos = data[:, 78:88]
        self.mvel = data[:, 88:98]

    def state(self, t):
        tmax = self.time[-1]

        i = int((t % tmax) / tmax * len(self.time))

        return (self.qpos[i], self.qvel[i])

    def action(self, t):
        tmax = self.time[-1]
        i = int((t % tmax) / tmax * len(self.time))
        return (self.mpos[i], self.mvel[i], self.torque[i])

    def sample(self):
        i = random.randrange(len(self.time))
        return (self.time[i], self.qpos[i], self.qvel[i])

    def __len__(self):
        return len(self.time)