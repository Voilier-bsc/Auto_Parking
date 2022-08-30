import heapq
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import Reeds_shepp_path as rs
import Vehicle

class Global_Node:
    def __init__(self, x, y, yaw, direction,
                 x_list, y_list, yaw_list, directions,
                 steer=0.0, parent_node=None, cost=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.directions = directions
        
        self.steer = steer
        self.parent_node = parent_node
        self.cost = cost

class Global_Path:
    def __init__(self, x_list, y_list, yaw_list, direction_list, cost):
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.direction_list = direction_list
        self.cost = cost
        
        
def calc_distance_heuristic(goal_x, goal_y, obstacle_list, veh_rad):
    return 0

def Global_Hybrid_A_Star_Planning(start_pos, target_pos, obstacle_list):
    obstacle_kd_tree = cKDTree(np.array(obstacle_list))
    
    start_node = Global_Node(start_pos[0], start_pos[1], start_pos[2], True,
                             [start_pos[0]], [start_pos[1]], [start_pos[2]], [True], cost = 0)
    target_node = Global_Node(target_pos[0], target_pos[1], target_pos[2], True,
                             [target_pos[0]], [target_pos[1]], [target_pos[2]], [True])
    
    OpenList, ClosedList = {}, {}
    
    h_dp = calc_distance_heuristic(target_node.x_list[-1], target_node.y_list[-1],
                                obstacle_list, Vehicle.VEHICLE_RAD)
    
    return 0

