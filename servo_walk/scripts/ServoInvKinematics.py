#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@author: Arnaud Villeneuve

This file contains the main program to run the Spotmicro Controller

"""

from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import math
import numpy as np

import Spotmicro_Inverse_Kinematics_and_Position_Library_v01
Spot = Spotmicro_Inverse_Kinematics_and_Position_Library_v01.Spot()
import Spotmicro_Gravity_Center_Library_v01
SpotCG = Spotmicro_Gravity_Center_Library_v01.SpotCG()

class ServoInvKinematicsEngine:
    def __init__(self):
        """ Walking parameters """
        self.b_height = 200
        self.h_amp = 100# horizontal step length
        self.v_amp = 40 #vertical step length
        self.track = 58.09
        self.x_offset = 0 #body offset in x direction 
        self.ra_longi = 30# body distance 
        self.ra_lat = 30#20
        self.steering =200 #Initial steering radius (arbitrary)
        self.walking_direction = 90/180*pi #Initial steering angle (arbitrary)
        self.stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2

        self.Angle = [0, 0]

        self.center_x = self.steering * cos(self.walking_direction) #steering center x relative to body center 
        self.center_y = self.steering * sin(self.walking_direction) #steering center y relative to body center

        self.pos_frontrear = 4
        self.pos_leftright = 3
        self.pos_turn = 0    

        self.joypos = [0.01959229, -0.01177979, -0.00393677,  0.06665039, -1, -1]

        self.t = 0 #Initializing timing/clock
        self.tstart = 1 #End of start sequence
        self.tstop = 1000 #Start of stop sequence by default
        self.tstep = 0.01 #Timing/clock step
        self.tstep1 = self.tstep

        self.distance =[] #distance to support polygon edge
        self.balance =[] #balance status (True or False)

        self.cw = 1
        self.walking_speed = 0
        self.walking_direction = 0
        self.module = 0

        self.Tcomp = 0.02

        self.x_spot = [0, self.x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        self.y_spot = [0,0,Spot.ylf+self.track, Spot.yrf-self.track, Spot.yrr-self.track, Spot.ylr+self.track,0,0,0]
        self.z_spot = [0,self.b_height,0,0,0,0,0,0,0]
        self.theta_spot = [0,0,0,0,0,0]

        self.stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted

        #theta xyz of ground then theta xyz of frame/body
        self.pos_init = [-self.x_offset, self.track, -self.b_height, 
                         -self.x_offset, -self.track, -self.b_height,
                         -self.x_offset, -self.track, -self.b_height,
                         -self.x_offset, self.track, -self.b_height]

        self.thetaLF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos_init[0], self.pos_init[1], self.pos_init[2], 1)[0]
        self.thetaRF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos_init[3], self.pos_init[4], self.pos_init[5], -1)[0]
        self.thetaRR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos_init[6], self.pos_init[7], self.pos_init[8], -1)[0]
        self.thetaLR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos_init[9], self.pos_init[10], self.pos_init[11], 1)[0]

        self.CG = SpotCG.CG_calculation (self.thetaLF, self.thetaRF, self.thetaRR, self.thetaLR)
        #Calculation of CG absolute position
        self.M = Spot.xyz_rotation_matrix(self.theta_spot[0],self.theta_spot[1],self.theta_spot[2],False)
        self.CGabs = Spot.new_coordinates(self.M,self.CG[0],self.CG[1],self.CG[2],self.x_spot[1],self.y_spot[1],self.z_spot[1])
        self.dCG = SpotCG.CG_distance(self.x_spot[2:6],self.y_spot[2:6],self.z_spot[2:6],self.CGabs[0],self.CGabs[1],self.stance)

        self.x_spot = [0, self.x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,self.CG[0],self.CGabs[0],self.dCG[1]]
        self.y_spot = [0,0,Spot.ylf+self.track, Spot.yrf-self.track, Spot.yrr-self.track, Spot.ylr+self.track,self.CG[1],self.CGabs[1],self.dCG[2]]
        self.z_spot = [0,self.b_height,0,0,0,0,self.CG[2],self.CGabs[2],self.dCG[3]]

        self.pos = [-self.x_offset,self.track,-self.b_height,-self.x_offset,-self.track,-self.b_height,-self.x_offset,-self.track,-self.b_height,-self.x_offset,self.track,-self.b_height,self.theta_spot,self.x_spot,self.y_spot,self.z_spot]
        
        self.walking = True
        self.stop = False
        self.Free = False
        self.t=0
        self.tstart = 1
        self.tstop = 1000
        self.lock = True
        self.trec = int(self.t)   

    def convertThetas(self, thetaRF, thetaRR, thetaLR, thetaLF):
        # Need to inverse various theta arrays since servo config is lower, mid, shoulder on robot.
        thetaArr = [thetaRF, thetaRR, thetaLR, thetaLF]
        # In the form of (m (slope), b (intercept))
        thetaSlopesAndIntercepts = [
            [(-31.25, 346), (-82.5, 330), (-56.42857143, 325)],
            [(-31.25, 286), (-96.25, 310), (-92.85714286, 275)],
            [(-31.25, 307), (75, 321), (87.14285714, 318)],
            [(31.25, 346), (75, 321), (78.57142857, 306)]
        ]

        positions = []
        for i in range(0, len(thetaArr)):
            leg_positions = []
            for j in range(0, len(thetaArr[i])):
                theta = thetaArr[i][j]
                slope = thetaSlopesAndIntercepts[i][j][0]
                intercept = thetaSlopesAndIntercepts[i][j][1]

                servoPosition = theta * slope + intercept # y = mx + b
                leg_positions.append(servoPosition)
            for item in leg_positions[::-1]:
                positions.append(item)
        return positions

    def validateThetas(self, thetaRF, thetaRR, thetaLR, thetaLF):
        # Servo Order: Shoulder, Mid, Lower (Note!! Diff from spreadsheet)
        # See https://docs.google.com/spreadsheets/d/1XUiQSWsIxxkEMCcOQQ--JhUazW5b6SP89guxUswyA4M/edit?usp=sharing
        thetaBoundaries = [
            [(-1.92, 0.96), (-1.563636364, 1.709090909), (-3.189873418, 1.594936709)],
            [(-1.28, 1.6), (-1.381818182, 1.423376623), (-2.153846154, 0.7538461538)],
            [(-1.92, 0.97), (-1.733333333, 1.866666667), (-2.547540984, 0.5508196721)],
            [(-2.56, 0.32), (-1.733333333, 1.866666667), (-2.672727273, 0.7636363636)]
        ]

        thetaValues = [thetaRF, thetaRR, thetaLR, thetaLF]

        for i in range(0, len(thetaValues)):
            row = thetaValues[i]
            for j in range(0, len(row)):
                val = row[j]
                minVal = thetaBoundaries[i][j][0]
                maxVal = thetaBoundaries[i][j][1]
                if not (minVal < val and val < maxVal):
                    title = ""
                    leg = ""
                    if i == 0:
                        title = "Right Front "
                    elif i == 1:
                        title = "Right Rear "
                    elif i == 2:
                        title = "Left Rear "
                    elif i == 3:
                        title = "Left Front "
                    if j == 0:
                        leg = "Shoulder"
                    if j == 1:
                        leg = "Mid: "
                    if j == 2:
                        leg = "Lower: "

                    print(title + leg + str(val))
                    return False
        return True

    def generate_next_step(self):
        coef = 1.2
                            
        if (abs(self.joypos[self.pos_leftright])>0.2)|(abs(self.joypos[self.pos_frontrear])>0.2)|(self.stop == True):                
            self.t=self.t+self.tstep
            trec = int(self.t)+1
            
            module_old = self.module
            walking_direction_old = self.walking_direction
            steering_old = self.steering
            
            x_old = module_old*cos(walking_direction_old)
            y_old = module_old*sin(walking_direction_old)
            
            #update request
            module = sqrt(self.joypos[self.pos_leftright]**2 + self.joypos[self.pos_frontrear]**2)
            self.walking_direction = (atan2(-self.joypos[self.pos_leftright],-self.joypos[self.pos_frontrear])%(2*pi)+pi/2)%(2*pi)
            
            x_new = module*cos(self.walking_direction)
            y_new = module*sin(self.walking_direction)
                            
            #steering update                
            if (abs(self.joypos[self.pos_turn]) < 0.2):
                self.cw = 1
                if (self.steering<2000):
                    self.steering = min(1e6,steering_old*coef) 
                else:
                    self.steering = 1e6
            else:
                self.steering = 2000-(abs(self.joypos[0])-0.2)*2000/0.8+0.001
                if ((self.steering/steering_old)>coef):                       
                    self.steering = steering_old*coef
                if ((steering_old/self.steering)>coef):                       
                    self.steering = steering_old/coef   
                    if (self.steering <0.001):
                        self.steering = 0.001
                self.cw = -np.sign(self.joypos[0])
            
            
            gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
            
            if (gap>0.01):
                x_new = x_old+ (x_new-x_old)/gap*0.01
                y_new = y_old+ (y_new-y_old)/gap*0.01
                module = sqrt(x_new**2+y_new**2)
                self.walking_direction = atan2(y_new,x_new)
                                                    
            #reduces speed sideways and backwards  
            min_h_amp = self.h_amp*(1/2e6*self.steering+1/2)               
            xa = 1+cos(self.walking_direction-pi/2) 
            walking_speed = min (1, module) * min(self.h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)                
            
            
        if ((abs(self.joypos[self.pos_leftright])<0.2)&(abs(self.joypos[self.pos_frontrear])<0.2))&(self.stop == False):  
            self.t = self.t + self.tstep                
            self.module = max (0, module-0.01)
            self.walking_speed = self.module* self.h_amp * ((1+cos(self.walking_direction-pi/2))/2*0.75+0.25)
            if (self.steering<2000):
                self.steering = min(1e6,self.steering*coef) 
            else:
                self.steering = 1e6
            self.cw=1    
            if (t>trec): 
                t=trec

        """ 
        If you have an IMU that measures Angle[0] and Angle [1] 
        values can be transferred to theta_spot
        """                
        self.theta_spot[3] = self.Angle [0] # angle around x axis
        self.theta_spot[4] = self.Angle [1] # angle around y axis
        self.theta_spot[0] = self.Angle [0] # angle around x axis
        self.theta_spot[1] = self.Angle [1] # angle around y axis
                    
        if (self.t< self.tstart):           
            self.pos = Spot.start_walk_stop (self.track,self.x_offset,self.steering,self.walking_direction,self.cw,walking_speed,self.v_amp,self.b_height,self.stepl,self.t,self.tstep,self.theta_spot,self.x_spot,self.y_spot,self.z_spot,'start')
        else:
            if (self.t<self.tstop):
                self.pos = Spot.start_walk_stop (self.track,self.x_offset,self.steering,self.walking_direction,self.cw,walking_speed,self.v_amp,self.b_height,self.stepl,self.t,self.tstep,self.theta_spot,self.x_spot,self.y_spot,self.z_spot,'walk')
            else:
                self.pos = Spot.start_walk_stop (self.track,self.x_offset,self.steering,self.walking_direction,self.cw,walking_speed,self.v_amp,self.b_height,self.stepl,self.t,self.tstep,self.theta_spot,self.x_spot,self.y_spot,self.z_spot,'stop')    
                        
        self.theta_spot = self.pos[12]
        self.x_spot = self.pos[13]
        self.y_spot = self.pos[14]                 
        self.z_spot = self.pos[15]
        
        if (self.t>(self.tstop+1-self.tstep)):
            self.stop = False
            self.walking = False
            self.Free = True
        
        self.xc = self.steering* cos(self.walking_direction)
        self.yc = self.steering* sin(self.walking_direction)
            
        center_x = self.x_spot[0]+(self.xc*cos(self.theta_spot[2])-self.yc*sin(self.theta_spot[2])) #absolute center x position
        center_y = self.y_spot[0]+(self.xc*sin(self.theta_spot[2])+self.yc*cos(self.theta_spot[2])) #absolute center y position


        self.thetaLF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos[0], self.pos[1], self.pos[2], 1)[0]
        self.thetaRF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos[3], self.pos[4], self.pos[5], -1)[0]
        self.thetaRR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos[6], self.pos[7], self.pos[8], -1)[0]
        self.thetaLR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, self.pos[9], self.pos[10], self.pos[11], 1)[0]
                
        stance = [False, False, False, False]
        if (self.pos[15][2] < 0.01):            
            stance[0] = True
        if (self.pos[15][3] < 0.01):           
            stance[1] = True
        if (self.pos[15][4] < 0.01):           
            stance[2] = True
        if (self.pos[15][5] < 0.01):              
            stance[3] = True

        if (self.Free == True):
            sleep(0.1)
        
        """ CG update """
        self.CG = SpotCG.CG_calculation (self.thetaLF,self.thetaRF,self.thetaRR,self.thetaLR)
        #Calculation of CG absolute position
        self.M = Spot.xyz_rotation_matrix(self.theta_spot[0],self.theta_spot[1],self.theta_spot[2],False)
        self.CGabs = Spot.new_coordinates(self.M,self.CG[0],self.CG[1],self.CG[2],self.x_spot[1],self.y_spot[1],self.z_spot[1])
        self.dCG = SpotCG.CG_distance(self.x_spot[2:6],self.y_spot[2:6],self.z_spot[2:6],self.CGabs[0],self.CGabs[1],stance)
        
        self.pos[13][6] = self.CG[0] #x
        self.pos[14][6] = self.CG[1] #y
        self.pos[15][6] = self.CG[2] #z
        
        self.pos[13][7] = self.CGabs[0] #x
        self.pos[14][7] = self.CGabs[1] #y
        self.pos[15][7] = self.CGabs[2] #z
        
        self.pos[13][8] = self.dCG[1] #xint
        self.pos[14][8] = self.dCG[2] #yint
        self.pos[15][8] = self.dCG[3] #balance
        
        self.distance.append(self.dCG[0])

        if self.validateThetas(self.thetaRF, self.thetaRR, self.thetaLR, self.thetaLF):
            positions = self.convertThetas(self.thetaRF, self.thetaRR, self.thetaLR, self.thetaLF)
            return positions
