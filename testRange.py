from cassiemujocoik_ctypes import *
import time
import array
import numpy as np
import pickle
import os
from processWalkCycles import *

import sys
import select
import tty
import termios

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# old_settings = termios.tcgetattr(sys.stdin)
# try:
#     tty.setcbreak(sys.stdin.fileno())

#     i = 0
#     while 1:
#         print(i)
#         i += 1

#         if isData():
#             c = sys.stdin.read(1)
#             if c == '\x1b':         # x1b is ESC
#                 break

# finally:
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


cassie = CassieIK(sim_steps=1, render_sim=True)

cassie.single_pos_ik([0,0.1,0,   0,-0.1,0,   0,0,1.0])
# input("bleh")
pos = [0,0.1,0,   0,-0.1,0,   0,0,1.0]


old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())
    while True:
        if isData():
            c = sys.stdin.read(1)
            print('INPUT: ' + str(c)+ '\n')
            if c == 'w':         # w up
                pos[8] += 0.05
            if c == 's':         # s up
                pos[8] -= 0.05
            if c == 'a':         # a left
                pos[7] += 0.05
            if c == 'd':         # d right
                pos[7] -= 0.05
            if c == 'q':         # q quit
                break
            print("New Pos: ")
            print(pos)
        
        cassie.single_pos_ik(pos)
        # time.sleep(1/120.0)
        
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)