import matplotlib.pyplot as plt
import numpy as np
import math
from math import cos, sin, tan, pi
from scipy.spatial.transform import Rotation as Rot

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

DT = 0.1

LF = LENGTH - BACKTOWHEEL
LB = BACKTOWHEEL

RearToCen = (LF - LB) / 2.0
VEHICLE_RAD = np.hypot((LF + LB) / 2.0, WIDTH / 2.0)

MAX_CURVATURE = math.tan(MAX_STEER) / WB

VRX = [LF, LF, -LB, -LB, LF]
VRY = [WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2, WIDTH / 2]

class Vehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw ## rad
        self.v = v
        self.predelta = None
        
        self.front_x = self.x + WB*cos(self.yaw)
        self.front_y = self.y + WB*sin(self.yaw)
        
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        
        self.global_target_x = None
        self.global_target_y = None
        self.global_target_yaw = None
        
        
    def plot_car(self, steer=0.0, truckcolor="-k"):
        
        
        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                                [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(self.yaw), math.sin(self.yaw)],
                            [-math.sin(self.yaw), math.cos(self.yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                            [-math.sin(steer), math.cos(steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += self.x
        outline[1, :] += self.y
        fr_wheel[0, :] += self.x
        fr_wheel[1, :] += self.y
        rr_wheel[0, :] += self.x
        rr_wheel[1, :] += self.y
        fl_wheel[0, :] += self.x
        fl_wheel[1, :] += self.y
        rl_wheel[0, :] += self.x
        rl_wheel[1, :] += self.y

        plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                    np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                    np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                    np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                    np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(self.x, self.y, "*")
        
        plt.plot(self.global_target_x, self.global_target_y, '-o')
    
    def update_state(self, a, delta):
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER

        self.x = self.x + self.v * math.cos(self.yaw) * DT
        self.y = self.y + self.v * math.sin(self.yaw) * DT
        self.yaw = self.yaw + self.v / WB * math.tan(delta) * DT
        self.v = self.v + a * DT
        
        self.front_x = self.x + WB*cos(self.yaw)
        self.fornt_y = self.y + WB*sin(self.yaw)

        if self.v > MAX_SPEED:
            self.v = MAX_SPEED
        elif self.v < MIN_SPEED:
            self.v = MIN_SPEED
            
## 뒷바퀴 기준 collision check 진행
def check_vehicle_collision(x, y, yaw, obstacle_list, kd_tree, AB=None):
    if isinstance(x, float):
        cx = x + RearToCen * cos(yaw)
        cy = y + RearToCen * sin(yaw)
        # plt.plot(cx, cy, "xb")
        # print(cx, cy)
        ids = kd_tree.query_ball_point([cx, cy], VEHICLE_RAD)
        if not ids:
            return True  # no collision
        
        ox = np.array(obstacle_list)[:,0]
        oy = np.array(obstacle_list)[:,1]
            
        if not rectangle_check(x, y, yaw, [ox[i] for i in ids], [oy[i] for i in ids]):
            # print("coll")
            return False # collision
        
        return True
    
    else:
        for i_x, i_y, i_yaw in zip(x, y, yaw):
            cx = i_x + RearToCen * cos(i_yaw)
            cy = i_y + RearToCen * sin(i_yaw)
            # print(cx, cy)
            ids = kd_tree.query_ball_point([cx, cy], VEHICLE_RAD)
            if not ids:
                return True  # no collision
            
            ox = np.array(obstacle_list)[:,0]
            oy = np.array(obstacle_list)[:,1]
            
            if not rectangle_check(i_x, i_y, i_yaw, [ox[i] for i in ids], [oy[i] for i in ids]):
                print("coll")
                return False # collision
        
        return True
    
def check_vehicle_narrow_collision(x, y, yaw, obstacle_list, kd_tree):
    if isinstance(x, float):
        cx = x + RearToCen * cos(yaw)
        cy = y + RearToCen * sin(yaw)
        # print(cx, cy)
        ids = kd_tree.query_ball_point([cx, cy], VEHICLE_RAD)
        if not ids:
            return True  # no collision
        
        
        ox = np.array(obstacle_list)[:,0]
        oy = np.array(obstacle_list)[:,1]
        
        
        if not rectangle_check(x, y, yaw, [ox[i] for i in ids], [oy[i] for i in ids]):
            return False # collision
        
        return True
    
    else:
        for i_x, i_y, i_yaw in zip(x, y, yaw):
            cx = i_x + RearToCen * cos(i_yaw)
            cy = i_y + RearToCen * sin(i_yaw)
            # print(cx, cy)
            ids = kd_tree.query_ball_point([cx, cy], VEHICLE_RAD)
            if not ids:
                return True  # no collision
            
            ox = np.array(obstacle_list)[:,0]
            oy = np.array(obstacle_list)[:,1]
            
            if not rectangle_check(i_x, i_y, i_yaw, [ox[i] for i in ids], [oy[i] for i in ids]):
                print("coll")
                return False # collision
        
        return True
    
def rectangle_check(x, y, yaw, ox, oy):
    # transform obstacles to base link frame
    rot = rot_mat_2d(yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        converted_xy = np.stack([tx, ty]).T @ rot
        rx, ry = converted_xy[0], converted_xy[1]

        if not (rx > LF or rx < -LB or ry > WIDTH / 2.0 + 0.4 or ry < -WIDTH / 2.0 - 0.4):
            return False  # no collision

    return True  # collision
            
def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]