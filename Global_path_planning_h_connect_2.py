import heapq
import math
import os
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from math import cos, sin, tan, pi

from sklearn import neighbors
from sqlalchemy import false, true

import Vehicle
import Environment as Env

N_STEER = 20
SB_COST = 0  # switch back penalty cost
BACK_COST = 1  # backward penalty cost
STEER_CHANGE_COST = 0.1 # steer angle change penalty cost
# global STEER_COST=5  # steer angle change penalty cost

D_WEIGHT = 1

NODE_DIST_AB = 1 #m
NODE_ANGLE_AB = 2 #deg

class Global_Node:
    def __init__(self, x_ind, y_ind, yaw_ind, direction,
                 x, y, yaw, steering_angle=0.0, parent_node=None, cost=None):
        self.x_ind = x_ind
        self.y_ind = y_ind
        self.yaw_ind = yaw_ind
        self.direction = direction
        
        self.x = x
        self.y = y
        self.yaw = yaw
        
        self.steering_angle = steering_angle
        self.parent_node = parent_node
        self.cost = cost
        
class Global_Path:
    def __init__(self, x_list, y_list, yaw_list, direction_list, cost):
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.direction_list = direction_list
        self.cost = cost
        
### H_Connect
def Global_Hybrid_A_star_Planning(start_pos, target_pos, obstacle_list):
    obstacle_kd_tree = cKDTree(np.array(obstacle_list))
    
    start_node = Global_Node(round(start_pos[0]), round(start_pos[1]), round(start_pos[2] / Env.YAW_RES), True, start_pos[0], start_pos[1], start_pos[2], cost = 0)
    target_node = Global_Node(round(target_pos[0]), round(target_pos[1]), round(target_pos[2] / Env.YAW_RES), True, target_pos[0], target_pos[1], target_pos[2], cost = 0)
    
    OpenList_A, ClosedList_A = {}, {}
    OpenList_B, ClosedList_B = {}, {}
    
    priority_q_A = []
    priority_q_B = []
    
    connect_node, AB = None, None
    
    
    start_ind = (start_node.x_ind, start_node.y_ind, start_node.yaw_ind)
    target_ind = (target_node.x_ind, target_node.y_ind, target_node.yaw_ind)
    
    
    OpenList_A[start_ind] = start_node
    OpenList_B[target_ind] = target_node
    
    heapq.heappush(priority_q_A, (calc_cost(start_node,target_node), start_ind))
    heapq.heappush(priority_q_B, (calc_cost(target_node,start_node), target_ind))
    
    Flag = False
    
    Near_node_A, Near_node_B = None, None
    global STEER_COST
    STEER_COST = 0.5
    while True:

        if not OpenList_A or not OpenList_B:
            print("cannot find global path")
            return None
        _, current_ind_A = heapq.heappop(priority_q_A)
        _, current_ind_B = heapq.heappop(priority_q_B)
        
        
        if current_ind_A in OpenList_A and current_ind_B in OpenList_B:
            current_node_A = OpenList_A.pop(current_ind_A)
            ClosedList_A[current_ind_A] = current_node_A
            
            current_node_B = OpenList_B.pop(current_ind_B)
            ClosedList_B[current_ind_B] = current_node_B
            
        
        else:
            continue
        
        # print(target_node.cost)


        # plt.plot(current_node_A.x, current_node_A.y, "xc")
        
        plt.arrow(current_node_A.x, current_node_A.y, 0.5*cos(current_node_A.yaw),0.5*sin(current_node_A.yaw), head_width=0.5)
        plt.arrow(current_node_B.x, current_node_B.y, 0.5*cos(current_node_B.yaw),0.5*sin(current_node_B.yaw), head_width=0.5 )
        # plt.plot(current_node_B.x, current_node_B.y, "xc")
        # veh = Vehicle.Vehicle(x = current_node.x, y =current_node.y, yaw=current_node.yaw, v=0.0)
        # veh.plot_car(0)
        
        plt.pause(0.001)
        

        #### break if 문 설정####
        is_end, connect_node, AB = calc_connect_node(ClosedList_A, ClosedList_B, current_node_A, current_node_B)
        
        if is_end:
            break
        # if calc_connect_node(ClosedList_A, ClosedList_B, current_node_A, current_node_B)==true:
        #     break
        # if calc_heuristic_cost(current_node, target_node) <= 0.5:
        #     if calc_angle_dist(current_node, target_node) <= 5:
        #         target_node.parent_node = current_node
        #         target_node.cost = current_node.cost
        #         break
        # if calc_cost(current_node_A, start_node) >= 5:
            # STEER_COST = 0
        if calc_cost(current_node_B, target_node) >= 5:
            STEER_COST = 0
        

        for neighbor in get_neighbors(current_node_A, obstacle_list, obstacle_kd_tree, "A"):
            neighbor_ind = (neighbor.x_ind, neighbor.y_ind, neighbor.yaw_ind)
            # plt.plot(neighbor.x, neighbor.y, "xc")
            # plt.pause(0.0001)
            if neighbor_ind in ClosedList_A:
                continue
            
            if neighbor not in OpenList_A or OpenList_A[neighbor_ind].cost > neighbor.cost:
                
                if(calc_heuristic_cost(neighbor, current_node_B) < 10 and Flag==False):
                    Flag = True
                    Near_node_B = current_node_B
                    Near_node_A = current_node_A
                    # STEER_COST = 0
                    
                if (Flag):
                    # print("test")
                    heapq.heappush(priority_q_A, (calc_cost(neighbor, Near_node_B), neighbor_ind))
                else:
                    heapq.heappush(priority_q_A, (calc_cost(neighbor, current_node_B), neighbor_ind))
                OpenList_A[neighbor_ind] = neighbor
        
        for neighbor in get_neighbors(current_node_B, obstacle_list, obstacle_kd_tree, "B"):
            neighbor_ind = (neighbor.x_ind, neighbor.y_ind, neighbor.yaw_ind)
            # plt.plot(neighbor.x, neighbor.y, "xc")
            # plt.pause(0.0001)
            if neighbor_ind in ClosedList_B:
                continue
            
            if neighbor not in OpenList_B or OpenList_B[neighbor_ind].cost > neighbor.cost:
                # heapq.heappush(priority_q_B, (calc_cost(neighbor, start_node), neighbor_ind))
                if(calc_heuristic_cost(neighbor, current_node_A) < 10 and Flag==False):
                    Flag = True
                    Near_node_B = current_node_B
                    Near_node_A = current_node_A
                    # STEER_COST = 0
                    
                if (Flag):
                    heapq.heappush(priority_q_B, (calc_cost(neighbor, Near_node_A), neighbor_ind))
                else:
                    heapq.heappush(priority_q_B, (calc_cost(neighbor, current_node_A), neighbor_ind))
                OpenList_B[neighbor_ind] = neighbor
            
    
    path = get_final_path(ClosedList_A, ClosedList_B, current_node_A, current_node_B, connect_node, AB)
    
    return path

def calc_connect_node(ClosedList_A, ClosedList_B, current_node_A, current_node_B):
    for node_A in ClosedList_A.values():
        # plt.plot(node_A.x, node_A.y, "+k")
        if calc_heuristic_cost(node_A, current_node_B) <= NODE_DIST_AB:
            if calc_angle_dist(node_A, current_node_B) <= NODE_ANGLE_AB:
                
                return True, node_A, "A"
            
    for node_B in ClosedList_B.values():
        if calc_heuristic_cost(node_B, current_node_A) <= NODE_DIST_AB:
            if calc_angle_dist(node_B, current_node_A) <= NODE_ANGLE_AB:
                return True, node_B, "B"
    
    # for node_A in ClosedList_A.values():
    #     if calc_heuristic_cost(node_A, current_node_B) <= NODE_DIST_AB:
    #         if calc_angle_dist(node_A, current_node_B) <= NODE_ANGLE_AB:
    #             return true
            
    # for node_B in ClosedList_B.values():
    #     if calc_heuristic_cost(node_B, current_node_A) <= NODE_DIST_AB:
    #         if calc_angle_dist(node_B, current_node_A) <= NODE_ANGLE_AB:
    #             return true
    # if calc_heuristic_cost(current_node_A, current_node_B) <= NODE_DIST_AB:
    #     if calc_angle_dist(current_node_A, current_node_B) <= NODE_ANGLE_AB:
    #         return true
        
    return False, None, None

def get_final_path(ClosedList_A, ClosedList_B, current_node_A, current_node_B, connected_node, AB):
    if AB=="A":
        node_ind_A = (connected_node.x_ind, connected_node.y_ind, connected_node.yaw_ind)
        node_ind_B = (current_node_B.parent_node.x_ind, current_node_B.parent_node.y_ind, current_node_B.parent_node.yaw_ind)
    
    else:
        node_ind_A = (current_node_A.parent_node.x_ind, current_node_A.parent_node.y_ind, current_node_A.parent_node.yaw_ind)
        node_ind_B = (connected_node.x_ind, connected_node.y_ind, connected_node.yaw_ind)
    final_cost = current_node_A.cost + current_node_B.cost
    
    final_A_x, final_A_y, final_A_yaw, final_A_dir = [], [], [], []
    final_B_x, final_B_y, final_B_yaw, final_B_dir = [], [], [], []
    
    while node_ind_A:
        node = ClosedList_A[node_ind_A]
        final_A_x.append(node.x)
        final_A_y.append(node.y)
        final_A_yaw.append(node.yaw)
        final_A_dir.append(node.direction)
        
        if(node.parent_node == None):
            break
        node_ind_A = (node.parent_node.x_ind, node.parent_node.y_ind, node.parent_node.yaw_ind)
        
    while node_ind_B:
        node = ClosedList_B[node_ind_B]
        final_B_x.append(node.x)
        final_B_y.append(node.y)
        final_B_yaw.append(node.yaw)
        final_B_dir.append(node.direction)
        
        if(node.parent_node == None):
            break
        node_ind_B = (node.parent_node.x_ind, node.parent_node.y_ind, node.parent_node.yaw_ind)
        
    final_x = list(reversed(final_A_x)) + list(final_B_x)
    final_y = list(reversed(final_A_y)) + list(final_B_y)
    final_yaw = list(reversed(final_A_yaw)) + list(final_B_yaw)
    final_direction = list(reversed(final_A_dir)) + list(final_B_dir)
    
    final_direction[0] = final_direction[1]
    path = Global_Path(final_x,final_y,final_yaw,final_direction,final_cost)
    
    
    return path
    

def get_neighbors(current_node, obstacle_list, kd_tree, AB):
    node_list = []
    steer_list = np.concatenate((np.linspace(-Vehicle.MAX_STEER, Vehicle.MAX_STEER, N_STEER), [0.0]))
    d_list = [-1, 1]
    
    for steer in steer_list:
        # print(steer_list)
        for d in d_list:
            
            new_node = calc_next_node(current_node, d, steer, obstacle_list, kd_tree, AB)
            
            if new_node and verify_node(new_node):
                
                # yield new_node
                node_list.append(new_node)
                
    return node_list

def calc_next_node(current_node, d, steer, obstacle_list, kd_tree, AB):
    x, y, yaw = current_node.x, current_node.y, current_node.yaw
    # print("yaw ",yaw)
    
    # if(AB=="B"):
    #     if(steer==0.0):
            # print(x, y, yaw)
            # print(Vehicle.check_vehicle_collision(x,y,yaw,obstacle_list,kd_tree))
    x, y, yaw = kinematic_move(x, y, yaw, d, steer)
    # plt.plot(x, y, "xb")
    
    # print("after_yaw", yaw)
    # if(AB=="B"):
    #     if(steer==0.0):
            # print("after : ", x, y, yaw)
            # print(Vehicle.check_vehicle_collision(x,y,yaw,obstacle_list,kd_tree))

    if not Vehicle.check_vehicle_collision(x - Vehicle.WB*cos(yaw),y - Vehicle.WB*sin(yaw),yaw,obstacle_list,kd_tree,AB):
        
        return None
    
    # print("x")
   
    direction = d == 1
    
    x_ind = round(x)
    y_ind = round(y)
    yaw_ind = round(yaw/Env.YAW_RES)
    
    overall_cost = 0.0
    
    if(AB=="A"):
        if(d == -1):
            overall_cost += BACK_COST
            
    if(AB=="B"):
        if(d == 1):
            overall_cost += BACK_COST
    
    if d != current_node.direction:
        overall_cost += SB_COST
    
    overall_cost += STEER_COST * abs(steer)

    overall_cost += STEER_CHANGE_COST * abs(current_node.steering_angle - steer)
    
    cost = current_node.cost + overall_cost
    
    new_node = Global_Node(x_ind, y_ind, yaw_ind, direction, x, y, yaw, steering_angle=steer, parent_node=current_node, cost = cost)
    
    return new_node
    

def kinematic_move(x, y, yaw, d, steer):
    # print(steer)
    yaw += pi_2_pi(D_WEIGHT * d * tan(steer) / Vehicle.WB)
    x += D_WEIGHT * d * cos(yaw)
    y += D_WEIGHT * d * sin(yaw)
    
    # print(x, y, yaw)
    return x, y, yaw

def calc_cost(start_node, target_node, flag=False):
    h_cost = calc_heuristic_cost(start_node,target_node,flag)
    return start_node.cost + h_cost

def calc_angle_dist(start_node, target_node):
    return abs(np.rad2deg(start_node.yaw) - np.rad2deg(target_node.yaw))

def calc_heuristic_cost(start_node, target_node, flag=False):
    x_dt = target_node.x - start_node.x
    y_dt = target_node.y - start_node.y
    yaw_dt = target_node.yaw - start_node.yaw
    
    if flag==False:
        return math.sqrt(pow(x_dt,2) + pow(y_dt,2))
        # print(math.sqrt(pow(x_dt,2) + pow(y_dt,2)), pow(yaw_dt,2))
    return math.sqrt(pow(x_dt,2) + pow(y_dt,2) + 5*pow(yaw_dt,2))    
    
    # start = np.array((start_node.x, start_node.y))
    # target = np.array((target_node.x, target_node.y))
    # return np.linalg.norm(target - start)

def verify_node(node):

    if Env.XMIN <= node.x_ind <= Env.XMAX and Env.YMIN <= node.y_ind <= Env.YMAX:
        return True
    return False


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi


# CAR_INIT_X = 3
# CAR_INIT_Y = 3
# CAR_INIT_YAW = 90 #deg

# env = Env.Environment_1(Env.XMIN,Env.YMIN,Env.XMAX,Env.YMAX)
# env.create_world_1()
# env.plot_world()
# veh = Vehicle.Vehicle(x = CAR_INIT_X, y =CAR_INIT_Y, yaw=np.deg2rad(CAR_INIT_YAW), v=0.0)
# veh.plot_car(0)

# obs_list = env.obstacle_list

# start_pos = [veh.x, veh.y, veh.yaw]
# target_pos = [env.target_x, env.target_y, np.deg2rad(-90.0)]

# print("start : ", start_pos)
# print("target : ", target_pos)

# global_path=Global_Hybrid_A_star_Planning(start_pos, target_pos, obs_list)
# env.plot_world(global_path=global_path)
# veh.plot_car(0)
# plt.pause(100)
