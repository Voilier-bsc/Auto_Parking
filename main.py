import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import Vehicle
import Environment as Env
# import Global_path_planning_1 as GL1
# import Global_path_planning_2 as GL2
import Global_path_planning_h_connect as GLHC
CAR_INIT_X = 3
CAR_INIT_Y = 3
CAR_INIT_YAW = 90 #deg


# while True:
# plt.clf()
env = Env.Environment_1(Env.XMIN,Env.YMIN,Env.XMAX,Env.YMAX)
env.create_world_1()
env.plot_world()
veh = Vehicle.Vehicle(x = CAR_INIT_X, y =CAR_INIT_Y, yaw=np.deg2rad(CAR_INIT_YAW), v=0.0)
veh.plot_car(0)

obs_list = env.obstacle_list


## for target yaw
if env.target_y > 15:
    env.target_yaw = np.deg2rad(90)
    
else:
    env.target_yaw = np.deg2rad(-90)


start_pos = [veh.x, veh.y, veh.yaw]
target_pos = [env.target_x, env.target_y, env.target_yaw]

print("start : ", start_pos)
print("target : ", target_pos)

global_path=GLHC.Global_Hybrid_A_star_Planning(start_pos, target_pos, obs_list)
if global_path != None:
    print("find path!")
        # break
        

veh.plot_car(0)
env.plot_world(global_path=global_path)
print("done")
plt.pause(0.01)
plt.show()