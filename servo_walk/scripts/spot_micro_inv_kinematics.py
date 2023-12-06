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

class ServoInvKinematics:
    def __init__(self):
        self.mainLoop()

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

    def mainLoop(self):
        """ Walking parameters """
        b_height = 200
        h_amp = 100# horizontal step length
        v_amp = 40 #vertical step length
        track = 58.09
        x_offset = 0 #body offset in x direction 
        ra_longi = 30# body distance 
        ra_lat = 30#20
        steering =200 #Initial steering radius (arbitrary)
        walking_direction = 90/180*pi #Initial steering angle (arbitrary)
        stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2

        Angle = [0, 0]

        center_x = steering*cos(walking_direction) #steering center x relative to body center 
        center_y = steering*sin(walking_direction) #steering center y relative to body center
        cw =1

        """ Joystick Init """

        """ XBOX One controller settings """
        """ use Essai_Joystick_01.py utility to find out the right parameters """

        but_move = 4

        pos_frontrear = 4
        pos_leftright = 3
        pos_turn = 0    

        joypos = [0.01959229, -0.01177979, -0.00393677,  0.06665039, -1, -1]
        joybut = np.zeros(10) #xbox one controller has 10 buttons

        t = 0 #Initializing timing/clock
        tstart = 1 #End of start sequence
        tstop = 1000 #Start of stop sequence by default
        tstep = 0.01 #Timing/clock step
        tstep1 = tstep

        distance =[] #distance to support polygon edge
        balance =[] #balance status (True or False)

        cw = 1
        walking_speed = 0
        walking_direction = 0
        module = 0

        Tcomp = 0.02

        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]

        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted

        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]

        thetaLF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetaRF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetaRR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetaLR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]

        CG = SpotCG.CG_calculation (thetaLF,thetaRF,thetaRR,thetaLR)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)

        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]

        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]
        
        walking = True
        stop = False
        Free = False
        t=0
        tstart = 1
        tstop = 1000
        lock = True
        trec = int(t)   

        while (True):  
            coef = 1.2
            
            if (joybut[but_move] == True)&(tstep > 0)&(lock == False):
                tstep = 0
                lock = True
              
            if (joybut[but_move] == True)&(tstep == 0)&(lock == False):
                tstep = tstep1
                lock = True    
                 
            if (abs(joypos[pos_leftright])>0.2)|(abs(joypos[pos_frontrear])>0.2)|(stop == True):                
                t=t+tstep
                trec = int(t)+1
                
                module_old = module
                walking_direction_old = walking_direction
                steering_old = steering
                
                x_old = module_old*cos(walking_direction_old)
                y_old = module_old*sin(walking_direction_old)
                
                #update request
                module = sqrt(joypos[pos_leftright]**2 + joypos[pos_frontrear]**2)
                walking_direction = (atan2(-joypos[pos_leftright],-joypos[pos_frontrear])%(2*pi)+pi/2)%(2*pi)
                
                x_new = module*cos(walking_direction)
                y_new = module*sin(walking_direction)
                                
                #steering update                
                if (abs(joypos[pos_turn]) < 0.2):
                    cw = 1
                    if (steering<2000):
                        steering = min(1e6,steering_old*coef) 
                    else:
                        steering = 1e6
                else:
                    steering = 2000-(abs(joypos[0])-0.2)*2000/0.8+0.001
                    if ((steering/steering_old)>coef):                       
                        steering = steering_old*coef
                    if ((steering_old/steering)>coef):                       
                        steering = steering_old/coef   
                        if (steering <0.001):
                            steering = 0.001
                    cw = -np.sign(joypos[0])
                
                
                gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
                
                if (gap>0.01):
                    x_new = x_old+ (x_new-x_old)/gap*0.01
                    y_new = y_old+ (y_new-y_old)/gap*0.01
                    module = sqrt(x_new**2+y_new**2)
                    walking_direction = atan2(y_new,x_new)
                                                       
                #reduces speed sideways and backwards  
                min_h_amp = h_amp*(1/2e6*steering+1/2)               
                xa = 1+cos(walking_direction-pi/2) 
                walking_speed = min (1, module) * min(h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)                
                
                
            if ((abs(joypos[pos_leftright])<0.2)&(abs(joypos[pos_frontrear])<0.2))&(stop == False):  
                t=t+tstep                
                module = max (0, module-0.01)
                walking_speed = module* h_amp * ((1+cos(walking_direction-pi/2))/2*0.75+0.25)
                if (steering<2000):
                    steering = min(1e6,steering*coef) 
                else:
                    steering = 1e6
                cw=1    
                if (t>trec): 
                    t=trec

            """ 
            If you have an IMU that measures Angle[0] and Angle [1] 
            values can be transferred to theta_spot
            """                
            theta_spot[3] = Angle [0] # angle around x axis
            theta_spot[4] = Angle [1] # angle around y axis
            theta_spot[0] = Angle [0] # angle around x axis
            theta_spot[1] = Angle [1] # angle around y axis
                        
            if (t< tstart):           
                pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'start')
            else:
                if (t<tstop):
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'walk')
                else:
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'stop')    
                           
            theta_spot = pos[12]
            x_spot = pos[13]
            y_spot = pos[14]                 
            z_spot = pos[15]
            
            if (t>(tstop+1-tstep)):
                stop = False
                walking = False
                Free = True
            
            xc = steering* cos(walking_direction)
            yc = steering* sin(walking_direction)
                
            center_x = x_spot[0]+(xc*cos(theta_spot[2])-yc*sin(theta_spot[2])) #absolute center x position
            center_y = y_spot[0]+(xc*sin(theta_spot[2])+yc*cos(theta_spot[2])) #absolute center y position


            thetaLF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
            thetaRF = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
            thetaRR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
            thetaLR = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]

            if self.validateThetas(thetaRF, thetaRR, thetaLR, thetaLF):
                positions = self.convertThetas(thetaRF, thetaRR, thetaLR, thetaLF)
                print("Positions: ", positions)
                    
            stance = [False, False, False, False]
            if (pos[15][2] < 0.01):            
                stance[0] = True
            if (pos[15][3] < 0.01):           
                stance[1] = True
            if (pos[15][4] < 0.01):           
                stance[2] = True
            if (pos[15][5] < 0.01):              
                stance[3] = True

            if (Free == True):
                sleep(0.1)
            
            """ CG update """
            CG = SpotCG.CG_calculation (thetaLF,thetaRF,thetaRR,thetaLR)
            #Calculation of CG absolute position
            M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
            CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
            dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
            
            
            pos[13][6] = CG[0] #x
            pos[14][6] = CG[1] #y
            pos[15][6] = CG[2] #z
            
            pos[13][7] = CGabs[0] #x
            pos[14][7] = CGabs[1] #y
            pos[15][7] = CGabs[2] #z
            
            pos[13][8] = dCG[1] #xint
            pos[14][8] = dCG[2] #yint
            pos[15][8] = dCG[3] #balance
            
            distance.append(dCG[0])

ServoInvKinematics()