"""
File: moco.py
Author: Thomas Woodruff
Date: 4/15/49
Description: Wrapper for actuators.py. Handles saturation condition,
             converts input to writable signal and passes to actuator
             control.
"""

import time
import busio
import board
import adafruit_pca9685 as pca
from adafruit_servokit import ServoKit
import numpy as np

class __Actuate__:
    def __init__(self, freq = 100):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = pca.PCA9685(self.i2c)
        self.kit = ServoKit(channels = 16)
        self.hat.frequency = freq
        
        self.throttle = 90
        self.dir = 90


    def startup(self, mchannel, schannel):
        
        self.motorCh = mchannel
        self.steerCh = schannel
        
        self.kit.servo[self.motorCh].angle = 90
        time.sleep(1)
        self.kit.servo[self.steerCh].angle = 90
        time.sleep(1)
        
        print("Car Initialized...")


    def Drive(self, throttle):
        self.throttle = throttle
        self.kit.servo[self.motorCh].angle = throttle
        
    def Steer(self, direction):
        self.dir = direction
        self.kit.servo[self.steerCh].angle = self.dir
        
        
    def getThrottle(self):
        return(self.throttle)
    
    def getDir(self):
        return(self.dir)
        
        
    def shutdown(self):
        self.kit.servo[self.motorCh].angle = 90
        self.kit.servo[self.steerCh].angle = 90
        print("System Shutdown...")



class MotorController:
    def __init__(self):
        self.car = __Actuate__()
        self.car.startup(3,0) #not a good place for this
        
        self.steer = self.car.getDir() #use getters from actuators
        self.speed = self.car.getThrottle()
        
        self.maxAngle = 135
        self.minAngle = 45
        self.maxSpeed = 115 #true max is higher
        self.minSpeed = 65 #true min is lower
        
        self.FWD = 1
        self.BACK = -1
        
    
    def setSteer(self, angleIN):
        """
        positive: right?
        negative: left?
        """
        self.steer = angleIN + 90
        return float(self.steer)
    
        
    def setDrive(self, speedIN):
        """input from -1 <-> 1"""
        slope = (self.maxSpeed-self.minSpeed)/2
        self.speed = (speedIN-self.BACK)*slope + self.minSpeed #maps from input to angle
        return float(self.speed)
    
    def getDrive(self):
        return(self.speed)
    
    def getSteer(self):
        return(self.steer)
    
    def run(self):
        """
        Checks saturation condition and writes to actuators
        Called whenever state is updated
        """
        #steering condition
        if (self.steer > self.maxAngle):
            self.steer = self.maxAngle
        elif (self.steer < self.minAngle):
            self.steer = self.minAngle
            
        self.car.Steer(self.steer)
        #print("Steering: ", self.steer)
        
        #throttle condition
        if (self.speed > self.maxSpeed):
            self.speed = self.maxSpeed
        elif (self.speed < self.minSpeed):
            self.speed = self.minSpeed
            
        self.car.Drive(self.speed)
        #print("Throttle: ", self.speed)
        
        
    def shutdown(self):
        self.car.shutdown()
        
        
        
class Accel:
    def __init__(self, mc):
        self.mc = mc
    
    
    def rampSpd(self, start, stop=0):
        """
        controls the acceleration of the car
        """
        if start>=stop:
            for spd in np.arange(start, stop, -0.07):
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                print(curr_spd)
                time.sleep(0.45)
        elif start<stop:
            for spd in np.arange(start, stop, 0.07):
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                print(curr_spd)
                time.sleep(0.45)
        else: #change to elsif that checks 'stop' var
            self.mc.shutdown()
        
        
    def rampDir(self, start, stop=0):
        """
        controls responsiveness of steering
        """
        if start>=0: #right
            for dir in np.arange(start, stop, 2):
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                print(curr_dir)
                time.sleep(0.09)
        elif start<0: #left
            for dir in np.arange(start, stop, -2):
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                print(curr_dir)
                time.sleep(0.09)
        else:
            self.mc.shutdown()
          
          
    def stop(self):
        curr_spd = self.mc.setDrive(0)
        curr_dir = self.mc.setSteer(0)
        self.mc.run()
        print("Stop ", curr_spd, curr_dir)
        
        
##    def setAccel(self, acc):
##        
##    def setResp(self, res):