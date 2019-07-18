"""
File: actuators.py
Author: Thomas Woodruff
Date: 4/15/19
Description: Direct control with ESC and Servo on the car.
             Includes startup, control, and shutdown methods.
"""

import time
import busio
import board
import adafruit_pca9685 as pca
from adafruit_servokit import ServoKit

class actuate:
    
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
        
        