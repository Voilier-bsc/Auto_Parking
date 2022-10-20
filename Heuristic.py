import heapq
import math
from ssl import VerifyMode
import Environment as Env
import numpy as np

DISCRETE_COST = 1

discrete_motion =   [[1, 0, DISCRETE_COST],
                    [0, 1, DISCRETE_COST],
                    [-1, 0, DISCRETE_COST],
                    [0, -1, DISCRETE_COST],
                    [-1, -1, DISCRETE_COST * math.sqrt(2)],
                    [-1, 1, DISCRETE_COST * math.sqrt(2)],
                    [1, -1, DISCRETE_COST * math.sqrt(2)],
                    [1, 1, DISCRETE_COST * math.sqrt(2)]]


class D_Node:
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost


def create_heuristic_dict(goal_x, goal_y, obstacle_list):
    goal_node = D_Node(round(goal_x), round(goal_y), 0.0)
    
    obstacle_map = create_obstacle_map(obstacle_list)
    
    open_set, heurisitc_dict = dict(), dict()
    
    goal_ind = calc_index(goal_node)
    open_set[goal_ind] = goal_node
    
    ## cost, ind
    priority_q = [(0, goal_ind)]

    while True:
        if not priority_q:
            break
        
        cost, current_ind = heapq.heappop(priority_q)
        
        if current_ind in open_set:
            current_node = open_set[current_ind]
            heurisitc_dict[current_ind] = current_node
            open_set.pop(current_ind)
            
        else:
            continue
        
        for i, _ in enumerate(discrete_motion):
            new_node = D_Node(current_node.x + discrete_motion[i][0], current_node.y + discrete_motion[i][1], current_node.cost + discrete_motion[i][2])
            new_ind = calc_index(new_node)
            
            if new_ind in heurisitc_dict:
                continue
            
            if not check_discrete_collision(new_node, obstacle_map):
                continue
            
            if new_ind not in open_set:
                open_set[new_ind] = new_node
                heapq.heappush(priority_q, (new_node.cost, calc_index(new_node)))
                
            else:
                if open_set[new_ind].cost >= new_node.cost:
                    open_set[new_ind] = new_node
                    heapq.heappush(priority_q, (new_node.cost, calc_index(new_node)))
        
    return heurisitc_dict

def create_obstacle_map(obstacle_list):
    obstacle_map = [[False for _ in range(Env.YWID+1)] for _ in range(Env.XWID+1)]
    for i in range(Env.XWID+1):
        x = i + Env.XMIN
        for j in range(Env.YWID+1):
            y = j + Env.XMIN
            for ox, oy in obstacle_list:
                if [x, y] == [round(ox), round(oy)]:
                    obstacle_map[i][j] = True
                    break
    return obstacle_map

def calc_index(node):
    return (node.y - Env.YMIN) * Env.XWID + (node.x - Env.XMIN)

def check_discrete_collision(node, obstacle_map):
    if node.x < Env.XMIN:
            return False
    elif node.y < Env.YMIN:
        return False
    elif node.x >= Env.XMAX:
        return False
    elif node.y >= Env.YMAX:
        return False

    if obstacle_map[node.x][node.y]:
        return False
    
    return True