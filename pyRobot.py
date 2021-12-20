import numpy as np
import random

class pyRobot(object):
    # constructor
    def __init__(self, x0, y0, th0, real_env, p_fault_odometry, p_fault_sensor):
        
        #initiate robot
        self.__p_fault_odom = p_fault_odometry
        self.__p_fault_sens = p_fault_sensor
        self.__x = x0
        self.__y = y0
        self.__th = th0
        self.real_env = real_env
        
        print("Robot initiated at x0:{}, y0:{}, th0:{}".format(x0, y0, th0))

    def print_robot_position(self):
        print("Robot position (x:{}, y:{}, th:{})".format(self.__x, self.__y, self.__th))

    def move_forward(self):
        print("Moving forward")
        #determine if faulty odometry
        dx=0
        if(random.random()>self.__p_fault_odom):
            dx=1
        else: print("'I slipped!'")

        if self.__th == 0:
            newcoord = (self.__y, self.__x+dx)   #right
        elif self.__th == 90:
            newcoord = (self.__y-dx, self.__x)     #up
        elif self.__th == 180:
            newcoord = (self.__y, self.__x-dx)     #left
        elif self.__th == 270:
            newcoord = (self.__y+dx, self.__x)     #down

        if self.real_env[newcoord] != 0:
            self.__y, self.__x = newcoord

    def turn_right(self):
        print("Turning right:")
        #determine if faulty odometry
        dth=0
        if(random.random()>self.__p_fault_odom):
            dth=90
        else: print("'I slipped!'")
        
        self.__th = self.__th-dth
        if(self.__th<0):
            self.__th = 270

    def turn_left(self):
        #determine if faulty odometry
        print("Turning left")
        dth=0
        if(random.random()>self.__p_fault_odom):
            dth=90
        else: print("'I slipped!'")
        
        self.__th = self.__th+dth
        if(self.__th>270):
            self.__th = 0

    def get_robot_postion(self):
        return (self.__x, self.__y, self.__th)

    def sense(self):
        print("Sensing")
        
        reading=np.zeros((3,3),dtype=int)
        if(random.random()>self.__p_fault_sens):
            x = self.__x
            y = self.__y        
            reading = self.real_env[y-1:y+2,x-1:x+2]
            #print("reading\n", reading)
        else:
            while(reading[1,1]==0):
                x = random.randint(1,11)
                y = random.randint(1,11)
                reading = self.real_env[y-1:y+2,x-1:x+2]        
                #print("reading\n", reading)
        return reading

