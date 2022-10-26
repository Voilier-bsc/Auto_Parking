import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


import Vehicle
import Environment as Env
import Global_path_planning_1 as GL1
# import Global_path_planning_2 as GL2
import Global_path_planning_h_connect as GLHC
import time
CAR_INIT_X = 3
CAR_INIT_Y = 3
CAR_INIT_YAW = 90 #deg

global_path_animation = True

# while True:
# plt.clf()
fig, ax = plt.subplots()
ax.set_xlim((0,Env.XMAX))
ax.set_ylim((0,Env.YMAX))
env = Env.Environment_1(Env.XMIN,Env.YMIN,Env.XMAX,Env.YMAX)
env.create_world(1,parking_dir='back')
env.plot_world()
# veh = Vehicle.Vehicle(x = CAR_INIT_X, y =CAR_INIT_Y, yaw=np.deg2rad(CAR_INIT_YAW), v=0.0)

# # circle = plt.Circle((veh.front_x, veh.front_y), Vehicle.WB/math.tan(Vehicle.MAX_STEER))
# circle_1 = plt.Circle((veh.front_x, veh.front_y), Vehicle.WB/math.tan(Vehicle.MAX_STEER/2)*2, color='b', fill=False)
# circle_2 = plt.Circle((veh.front_x, veh.front_y), Vehicle.WB/math.tan(Vehicle.MAX_STEER)*2 , color='r', fill=False)

# ax.add_patch(circle_1)
# ax.add_patch(circle_2)
# veh.plot_car(0)



obs_list = env.obstacle_list
plt.pause(0.1)


start_pos = [env.vehicle_list[0].front_x, env.vehicle_list[0].front_y, env.vehicle_list[0].yaw]
target_pos = [env.vehicle_list[0].global_target_x, env.vehicle_list[0].global_target_y, env.vehicle_list[0].global_target_yaw]

# print("start : ", start_pos)
# print("target : ", target_pos)
start_time = time.time()
global_path=GLHC.Global_Hybrid_A_star_Planning(start_pos, target_pos, obs_list,global_path_animation)
end_time = time.time()
print(end_time-start_time)

# # global_path=GL1.Global_Hybrid_A_star_Planning(start_pos, target_pos, obs_list)
# if global_path != None:
#     print("find path!")
#         # break

# ax.add_patch(circle_1)
# ax.add_patch(circle_2)
plt.clf()
# veh.plot_car(0)
for global_path_x, global_path_y, global_path_yaw in zip(global_path.x_list, global_path.y_list, global_path.yaw_list):
    plt.clf()
    env.vehicle_list[0].x = global_path_x - Vehicle.WB*math.cos(global_path_yaw)
    env.vehicle_list[0].y = global_path_y - Vehicle.WB*math.sin(global_path_yaw)
    env.vehicle_list[0].yaw = global_path_yaw
    env.plot_world(global_path=global_path)
    plt.pause(0.1)
plt.show()