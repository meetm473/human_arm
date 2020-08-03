#!/usr/bin/env python3

import threading
import math
from queue import Queue
import time

class Rpy:
    rotateX = 0
    rotateY = 0
    rotateZ = 0
    
    lock = threading.Condition()
    
    def __init__(self):
        pass
    

class Fusion(threading.Thread):
        
    def onlyAccelerometer(self, restrict_roll):
        acc = [float(i) for i in self.rawData.get()[:3]]
        for j in range(0,3):
            acc[j] -= self.bias[j]
        Rpy.lock.acquire()
        if(restrict_roll):
            deno = math.sqrt(acc[1]**2 + acc[2]**2)
            if(deno!=0):
                Rpy.rotateX = math.atan2(acc[1],acc[2])
                Rpy.rotateY = math.atan(-acc[0]/(math.sqrt(acc[1]**2 + acc[2]**2)))                     
        else:
            deno = math.sqrt(acc[0]**2 + acc[2]**2)
            if(deno!=0):
                Rpy.rotateX = math.atan(acc[1]/(math.sqrt(acc[0]**2 + acc[2]**2)))
                Rpy.rotateY = math.atan2(-acc[0],acc[2])
        
        try:
            Rpy.lock.notify()
        finally:
            Rpy.lock.release()
        
    def onlyGyroscope(self):
        gyro = [float(i) for i in self.rawData.get()[3:]]
        for j in range(0,3):
            gyro[j] -= self.bias[j+3]
        Rpy.lock.acquire()
        cur_time = time.time()
        self.prev_time -= cur_time
        Rpy.rotateX = (Rpy.rotateX - gyro[0]*self.prev_time)
        Rpy.rotateY = (Rpy.rotateY - gyro[1]*self.prev_time)
        Rpy.rotateZ = (Rpy.rotateZ - gyro[2]*self.prev_time)
       
        self.prev_time = cur_time;        
        try:
            Rpy.lock.notify()
        finally:
            Rpy.lock.release()
        
    def acclAndGyro(self, restrict_roll):
        data = [float(i) for i in self.rawData.get()[0:6]]
        for j in range(0,6):
            data[j] -= self.bias[j]
        
        Rpy.lock.acquire()
        
        # Estimating roll and pitch
        acc_roll = 0;
        acc_pitch = 0;
        if(restrict_roll):
            deno = math.sqrt(data[1]**2 + data[2]**2)
            if(deno!=0):
                Rpy.rotateX = -math.atan2(data[1],data[2])
                Rpy.rotateY = math.atan(-data[0]/(math.sqrt(data[1]**2 + data[2]**2)))         
        else:
            deno = math.sqrt(data[0]**2 + data[2]**2)
            if(deno!=0):
                Rpy.rotateX = math.atan(data[1]/(math.sqrt(data[0]**2 + data[2]**2)))
                Rpy.rotateY = math.atan2(-data[0],data[2])
        
        # Estimating yaw
        cur_time = time.time()
        self.prev_time -= cur_time
        Rpy.rotateZ = (Rpy.rotateZ - data[5]*self.prev_time)
        self.prev_time = cur_time;
        
        try:
            Rpy.lock.notify()
        finally:
            Rpy.lock.release()

    def __init__(self):
        threading.Thread.__init__(self)
        self.rawData = Queue()
        self.show_fused_data = False
        self.stop_fusion = False
        self.initial_read = False
        self.bias = [0,0,0,0,0,0]     # accl(3), gryo(3)
        self.prev_time = 0

    def run(self):
        counter = 0;
        while not self.stop_fusion:
            if not self.rawData.empty():
                if self.initial_read:
                    #self.onlyAccelerometer(True)
                    #self.onlyGyroscope()
                    self.acclAndGyro(True)
                else:
                    self.bias += [float(i) for i in self.rawData.get()]
                    counter = counter + 1
                    self.prev_time = time.time()
                    print('Calculating initial bias')
                    if counter == 50:
                        self.initial_read = True
                        self.bias = [i/50 for i in self.bias]
                        print('Calculated the bias')
            
            if(self.show_fused_data):
                print([Rpy.rotateX, Rpy.rotateY, Rpy.rotateZ])
        
        # Resetting Rpy values to mark end of fusion
        Rpy.lock.acquire()
        Rpy.rotateX = 0
        Rpy.rotateY = 0
        Rpy.rotateZ = 0
        try:
            Rpy.lock.notify()
        finally:
            Rpy.lock.release()
        print('Stopped fusion')    

if __name__ == '__main__':
	print('Please run the script a_imu_control.py')
