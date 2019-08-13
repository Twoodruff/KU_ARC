"""
File: moco.py
Author: Thomas Woodruff
Date: 8/07/19
Revision: 0.1.1
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

#This class acts directly with the servo shield to control the servo and motor
class __Actuate__:
    def __init__(self, freq = 100):
        """sets up I2C comms with the PCA9685 board"""
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = pca.PCA9685(self.i2c)
        self.kit = ServoKit(channels = 16)
        self.hat.frequency = freq

        self.throttle = 90
        self.dir = 90


    def startup(self, mchannel, schannel):
        'initializes the steering and motor to their neutral positions'
        self.motorCh = mchannel
        self.steerCh = schannel

        self.kit.servo[self.motorCh].angle = 90
        time.sleep(1)
        self.kit.servo[self.steerCh].angle = 90
        time.sleep(1)

        print("Car Initialized...")


    def Drive(self, throttle):
        'applies the throttle value to the motor'
        self.throttle = throttle
        self.kit.servo[self.motorCh].angle = throttle

    def Steer(self, direction):
        'applies the direction value to the steering servo'
        self.dir = direction
        self.kit.servo[self.steerCh].angle = self.dir


    def getThrottle(self):
        return(self.throttle)

    def getDir(self):
        return(self.dir)


    def shutdown(self):
        'sets steering and motor back to their neutral positions'
        self.kit.servo[self.motorCh].angle = 90
        self.kit.servo[self.steerCh].angle = 90
        print("System Shutdown...")


#This class is an interface between the PCA board and the outside world
class MotorController:
    def __init__(self):
        'create a car instance and setup initial values'
        self.car = __Actuate__()
        self.car.startup(3,0)                                                   #not a good place for this

        self.steer = self.car.getDir()                                          #use getters from actuators
        self.speed = self.car.getThrottle()

        self.maxAngle = 135
        self.minAngle = 45
        self.maxSpeed = 115                                                     #true max is higher
        self.minSpeed = 65                                                      #true min is lower

        self.FWD = 1
        self.BACK = -1


    def setSteer(self, angleIN):
        'converts angleIN to readable value for __Actuate__'
        'positive => right  negative => left'
        self.steer = angleIN + 90
        return float(self.steer)


    def setDrive(self, speedIN):
        'maps speedIN from [-1,1] to angle value for __Actuate__'
        slope = (self.maxSpeed-self.minSpeed)/2
        self.speed = (speedIN-self.BACK)*slope + self.minSpeed                  #maps from input to angle
        return float(self.speed)

    def getDrive(self):
        return(self.speed)

    def getSteer(self):
        return(self.steer)

    def run(self):
        """
        Checks saturation condition and writes to actuators
        Called when state is updated
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
        #need a way to prevent more commands from being written after shutdown


#This class is an open-loop control on the acceleration and steering sensitivity of a car
class Accel:
    def __init__(self, mc):
        'define a MotorController object to act on'
        self.mc = mc


    def rampSpd(self, start, stop=0):
        """
        controls the acceleration of the car
        specify start and stop speed in range of [-1,1]
        """
        if start>=stop:
            for spd in np.arange(start, stop, -0.07):                           #step value chosen experimentally
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                print(curr_spd)
                time.sleep(0.45)                                                #delay value chosen experimentally
        elif start<stop:
            for spd in np.arange(start, stop, 0.07):
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                print(curr_spd)
                time.sleep(0.45)
        else:                                                                   #change to elif that checks 'stop' var
            self.mc.shutdown()


    def rampDir(self, start, stop=0):
        """
        controls responsiveness of steering
        """
        if start>=0: #right
            for dir in np.arange(start, stop, 2):                               #step value chosen experimentally
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                print(curr_dir)
                time.sleep(0.09)                                                #delay value chosen experimentally
        elif start<0: #left
            for dir in np.arange(start, stop, -2):
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                print(curr_dir)
                time.sleep(0.09)
        else:
            self.mc.shutdown()                                                  #replace shutdown with stop condition


    def stop(self):
        curr_spd = self.mc.setDrive(0)
        curr_dir = self.mc.setSteer(0)
        self.mc.run()
        print("Stop ", curr_spd, curr_dir)


##    def setAccel(self, acc):                                                  #future setter that would allow for step and delay to chosen by user
##                                                                              #...possibly
##    def setResp(self, res):
