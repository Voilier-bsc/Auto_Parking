import matplotlib.pyplot as plt
import numpy as np
import math
import Vehicle
import random

PARKINGLOT_H = 5.2
PARKINGLOT_L = 2.6
PARKINGLOT_AISLE = 6

XMIN = 0
YMIN = 0
XMAX = 2*PARKINGLOT_AISLE + 6*PARKINGLOT_L
YMAX = 3*PARKINGLOT_AISLE + 2*PARKINGLOT_H

class parking_slot:
    def __init__(self, slot_id, start_x, start_y):
        self.id = slot_id
        self.start_x = start_x
        self.start_y = start_y
        
        self.center_x = start_x + PARKINGLOT_L/2
        self.center_y = start_y + PARKINGLOT_H/2
        
        self.slot_list = []
        self.obs_list = []
        
        self.full = False
        iter_i = np.arange(self.start_x, start_x+PARKINGLOT_L, 0.1)
        iter_j = np.arange(self.start_y, start_y+PARKINGLOT_H, 0.1)
        for i in iter_i:
            self.slot_list.append([i,self.start_y])
            self.slot_list.append([i,self.start_y+PARKINGLOT_H])
            
        for j in iter_j:
            self.slot_list.append([self.start_x,j])
            self.slot_list.append([self.start_x+PARKINGLOT_L,j])

    def append_car(self):
        iter_i = np.arange(self.start_x + 0.3, self.start_x + Vehicle.WIDTH + 0.3, 0.1)
        iter_j = np.arange(self.start_y + 0.3, self.start_y + Vehicle.LENGTH + 0.3, 0.1)
        for i in iter_i:
            for j in iter_j:
                self.obs_list.append([i,j])
    

class Environment_1:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.obstacle_list = []
        self.parking_slot_list = []
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.target_x = 0
        self.target_y = 0
        
    def append_wall(self, start_x, start_y, width, height):
        iter_i = np.arange(start_x, start_x+width, 0.1)
        iter_j = np.arange(start_y, start_y+height, 0.1)
        
        for i in iter_i:
            self.obstacle_list.append([i,start_y])
            self.obstacle_list.append([i,start_y+height])
            
        for j in iter_j:
            self.obstacle_list.append([start_x,j])
            self.obstacle_list.append([start_x+width,j])
                
    def create_parking_slot(self, num_slot, start_x, start_y, start_slot=0):
        for n in range(num_slot):
            new_slot = parking_slot(n+start_slot, start_x, start_y)
            if(random.randint(0,1)==1):
                new_slot.append_car()
                new_slot.full=True
                self.obstacle_list = self.obstacle_list + new_slot.obs_list
            self.parking_slot_list.append(new_slot)
            
            start_x = start_x + PARKINGLOT_L
            
    def create_world_1(self):
        self.append_wall(self.xmin,self.ymin,self.xmax,self.ymax)
        self.create_parking_slot(num_slot=6, start_x=PARKINGLOT_AISLE, start_y=PARKINGLOT_AISLE)
        self.create_parking_slot(num_slot=6, start_x=PARKINGLOT_AISLE, start_y=PARKINGLOT_AISLE*2 + PARKINGLOT_H, start_slot=6)
        while(True):
            rand_num = random.randint(0,len(self.parking_slot_list)-1)
            if(self.parking_slot_list[rand_num].full == False):
                self.target_x = self.parking_slot_list[rand_num].center_x
                self.target_y = self.parking_slot_list[rand_num].center_y
                break
        
        
    def plot_world(self):
        plt.plot(np.array(self.obstacle_list)[:,0], np.array(self.obstacle_list)[:,1], '.k')
        for parking_slot in self.parking_slot_list:
            plt.plot(np.array(parking_slot.slot_list)[:,0], np.array(parking_slot.slot_list)[:,1], '.g')
        plt.plot(self.target_x, self.target_y, '-o')
        

env_1 = Environment_1(XMIN,YMIN,XMAX,YMAX)
env_1.create_world_1()
plt.clf()
env_1.plot_world()
veh_1 = Vehicle.Vehicle(x=3, y=3, yaw=math.radians(90), v=0.0)
veh_1.plot_car(0)
plt.grid()
plt.show()


