import heapq
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from math import cos, sin, tan, pi

import Vehicle
import Environment as Env
import Reeds_shepp_path as rs

N_STEER = 20
SB_COST = 1.0  # switch back penalty cost
BACK_COST = 0.1  # backward penalty cost
STEER_CHANGE_COST = 0.03  # steer angle change penalty cost
STEER_COST = 0.01  # steer angle change penalty cost

D_WEIGHT = 1

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
        
        
def Global_Hybrid_A_star_Planning(start_pos, target_pos, obstacle_list):
    obstacle_kd_tree = cKDTree(np.array(obstacle_list))
    
    start_node = Global_Node(round(start_pos[0]), round(start_pos[1]), round(start_pos[2] / Env.YAW_RES), True, start_pos[0], start_pos[1], start_pos[2], cost = 0)
    target_node = Global_Node(round(target_pos[0]), round(target_pos[1]), round(target_pos[2] / Env.YAW_RES), True, target_pos[0], target_pos[1], target_pos[2], cost = 0)
    
    OpenList, ClosedList = {}, {}
    
    priority_q = []
    
    start_ind = (start_node.x_ind, start_node.y_ind, start_node.yaw_ind)
    OpenList[start_ind] = start_node
    
    heapq.heappush(priority_q, (calc_cost(start_node,target_node), start_ind))
    rs_path = None
    current_node = None
    
    while True:
        if not OpenList:
            print("cannot find global path")
            return [], [], []
        
        _, current_ind = heapq.heappop(priority_q)
        
        
        if current_ind in OpenList:
            current_node = OpenList.pop(current_ind)
            ClosedList[current_ind] = current_node
        
        else:
            continue

        # plt.plot(current_node.x, current_node.y, "xc")
        # veh = Vehicle.Vehicle(x = current_node.x, y =current_node.y, yaw=current_node.yaw, v=0.0)
        # veh.plot_car(0)
        
        # plt.pause(0.001)
        
        is_updated, rs_path = update_node_with_analytic_expansion(current_node, target_node, obstacle_list, obstacle_kd_tree)
        
        if is_updated:
            print("rs path found")
            break

        
        for neighbor in get_neighbors(current_node, obstacle_list, obstacle_kd_tree):
            neighbor_ind = (neighbor.x_ind, neighbor.y_ind, neighbor.yaw_ind)
            # plt.plot(neighbor.x, neighbor.y, "xc")
            # plt.pause(0.0001)
            if neighbor_ind in ClosedList:
                continue
            
            if neighbor not in OpenList or OpenList[neighbor_ind].cost > neighbor.cost:
                heapq.heappush(priority_q, (calc_cost(neighbor, target_node), neighbor_ind))
                OpenList[neighbor_ind] = neighbor
                
            
    
    path = get_final_path(ClosedList, current_node, rs_path)
    
    return path

def update_node_with_analytic_expansion(current_node, target_node,
                                        obstacle_list, obstacle_kd_tree):
    rs_path = analytic_expansion(current_node, target_node, obstacle_list, obstacle_kd_tree)

    if rs_path:
        return True, rs_path

    return False, None

def analytic_expansion(current_node, target_node, obstacle_list, obstacle_kd_tree):
    
    max_curvature = math.tan(Vehicle.MAX_STEER) / Vehicle.WB
    paths = rs.calc_paths(current_node.x, current_node.y, current_node.yaw,
                          target_node.x, target_node.y, target_node.yaw,
                          max_curvature, step_size=Env.STEP_SIZE)

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        print(path.x)
        if Vehicle.check_vehicle_collision(path.x, path.y, path.yaw, obstacle_list, obstacle_kd_tree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path

    return best_path

def calc_rs_path_cost(reed_shepp_path):
    cost = 0.0
    for length in reed_shepp_path.lengths:
        if length >= 0:  # forward
            cost += length
        else:  # back
            cost += abs(length) * BACK_COST

    # switch back penalty
    for i in range(len(reed_shepp_path.lengths) - 1):
        # switch back
        if reed_shepp_path.lengths[i] * reed_shepp_path.lengths[i + 1] < 0.0:
            cost += SB_COST

    # steer penalty
    for course_type in reed_shepp_path.ctypes:
        if course_type != "S":  # curve
            cost += STEER_COST * abs(Vehicle.MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    n_ctypes = len(reed_shepp_path.ctypes)
    u_list = [0.0] * n_ctypes
    for i in range(n_ctypes):
        if reed_shepp_path.ctypes[i] == "R":
            u_list[i] = - Vehicle.MAX_STEER
        elif reed_shepp_path.ctypes[i] == "L":
            u_list[i] = Vehicle.MAX_STEER

    for i in range(len(reed_shepp_path.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(u_list[i + 1] - u_list[i])

    return cost

def get_final_path(ClosedList, target_node, rs_path):
    
    final_x = []
    final_y = []
    final_yaw = []
    final_direction = []
    final_cost = 0
        
    if target_node.parent_node is not None:
        node_ind = (target_node.parent_node.x_ind, target_node.parent_node.y_ind, target_node.parent_node.yaw_ind)
        final_cost = target_node.cost
        
        while node_ind:
            node = ClosedList[node_ind]
            final_x.append(node.x)
            final_y.append(node.y)
            final_yaw.append(node.yaw)
            final_direction.append(node.direction)
            veh = Vehicle.Vehicle(x = node.x, y =node.y, yaw=node.yaw, v=0.0)
            # veh.plot_car(0)
            # plt.pause(0.001)
            if(node.parent_node == None):
                break
            node_ind = (node.parent_node.x_ind, node.parent_node.y_ind, node.parent_node.yaw_ind)
            
        final_x = list(reversed(final_x))
        final_y = list(reversed(final_y))
        final_yaw = list(reversed(final_yaw))
        final_direction = list(reversed(final_direction))
        
    if(len(final_direction)  >= 2):
        final_direction[0] = final_direction[1]
    
    ### append rs_path
    final_x = final_x + rs_path.x[1:]
    final_y = final_y + rs_path.y[1:]
    final_yaw = final_yaw + rs_path.yaw[1:]
    

    fd = []
    for d in rs_path.directions[1:]:
        fd.append(d >= 0)
        
    final_direction = final_direction + fd
    
    path = Global_Path(final_x,final_y,final_yaw,final_direction,final_cost)
    
    return path
    
def get_neighbors(current_node, obstacle_list, kd_tree):
    node_list = []
    steer_list = np.concatenate((np.linspace(-Vehicle.MAX_STEER, Vehicle.MAX_STEER, N_STEER), [0.0]))
    d_list = [-1, 1]
    
    for steer in steer_list:
        for d in d_list:
            new_node = calc_next_node(current_node, d, steer, obstacle_list, kd_tree)
            if new_node and verify_node(new_node):
                # yield new_node
                node_list.append(new_node)
                
    return node_list

def calc_next_node(current_node, d, steer, obstacle_list, kd_tree):
    x, y, yaw = current_node.x, current_node.y, current_node.yaw
    x, y, yaw = kinematic_move(x, y, yaw, d, steer)
    if not Vehicle.check_vehicle_collision(x,y,yaw,obstacle_list,kd_tree):
        return None
    
    direction = d == 1
    
    x_ind = round(x)
    y_ind = round(y)
    yaw_ind = round(yaw/Env.YAW_RES)
    
    overall_cost = 0.0
    
    
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

def calc_cost(start_node, target_node):
    h_cost = calc_heuristic_cost(start_node,target_node)
    return start_node.cost + h_cost

def calc_heuristic_cost(start_node, target_node):
    x_dt = target_node.x - start_node.x
    y_dt = target_node.y - start_node.y
    
    return math.sqrt(pow(x_dt,2) + pow(y_dt,2))    

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
# # plt.show()

# obs_list = env.obstacle_list

# start_pos = [veh.x, veh.y, veh.yaw]
# target_pos = [env.target_x, env.target_y, np.deg2rad(90.0)]

# print("start : ", start_pos)
# print("target : ", target_pos)

# global_path=Global_Hybrid_A_star_Planning(start_pos, target_pos, obs_list)
# env.plot_world(global_path=global_path)
# veh.plot_car(0)
# plt.show()

# plt.pause(4)
