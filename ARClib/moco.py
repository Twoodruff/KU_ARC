"""
File: moco.py
Author: Thomas Woodruff
Date: 10/18/19
Revision: 0.1
Description: Handles all motion control for the car
"""

import time
import busio
import board
import adafruit_pca9685 as pca
from adafruit_servokit import ServoKit
import numpy as np


class __Actuate__:
    '''
    Class acts directly with the servo shield to control the servo and motor
    '''
    def __init__(self, freq = 100):
        '''
        Inputs:
            freq : frequency for the I2C comms

        Function: sets up I2C comms with the PCA9685 board
        '''
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = pca.PCA9685(self.i2c)
        self.kit = ServoKit(channels = 16)
        self.hat.frequency = freq

        self.throttle = 90
        self.dir = 90

        self.running = True


    def startup(self, mchannel, schannel):
        '''
        Inputs:
            mchannel : channel on PCA9685 connected to motor
            schannel : channel on PCA9685 connected to servo

        Function: initializes the steering and motor to their neutral positions
        '''
        self.motorCh = mchannel
        self.steerCh = schannel

        self.kit.servo[self.motorCh].angle = 90
        time.sleep(1)
        self.kit.servo[self.steerCh].angle = 90
        time.sleep(1)

        print("Car Initialized...")


    def Drive(self, throttle):
        '''
        Inputs:
            throttle : PWM angle that represents the desired speed

        Function: applies the throttle value to the motor
        '''
        self.throttle = throttle
        self.kit.servo[self.motorCh].angle = throttle

    def Steer(self, direction):
        '''
        Inputs:
            direction : PWM angle that represents the desired steering angle

        Function: applies the direction value to the steering servo
        '''
        self.dir = direction
        self.kit.servo[self.steerCh].angle = self.dir


    def getThrottle(self):
        return(self.throttle)

    def getDir(self):
        return(self.dir)


    def shutdown(self):
        '''
        Function: stops car by setting steering and motor
                  back to their neutral positions
        '''
        self.kit.servo[self.motorCh].angle = 90
        self.kit.servo[self.steerCh].angle = 90
        print("System Shutdown...")



class MotorController:
    '''
    Class is an interface between the PCA board and the outside world
    Wrapper class for _Actuate_
    '''
    maxAngle = 135
    minAngle = 45
    maxSpeed = 115  #true max is higher
    minSpeed = 65   #true min is lower

    FWD = 1
    BACK = -1

    def __init__(self):
        '''
        create an __Actuate__ instance and setup initial values
        '''
        self.car = __Actuate__()
        self.car.startup(mchannel = 3, schannel = 0)

        self.steer = self.car.getDir()
        self.speed = self.car.getThrottle()

        self.running = True


    def run(self):
        '''
        Function: Checks saturation condition and sets desired values
        '''
        if self.running:
            # STEERING CONDITION
            if (self.steer > MotorController.maxAngle):
                self.steer = MotorController.maxAngle
            elif (self.steer < MotorController.minAngle):
                self.steer = MotorController.minAngle
            else:
                pass

            # THROTTLE CONDITION
            if (self.speed > MotorController.maxSpeed):
                self.speed = MotorController.maxSpeed
            elif (self.speed < MotorController.minSpeed):
                self.speed = MotorController.minSpeed
            else:
                pass


    def update(self):
        '''
        Function: writes to actuators
        '''
        self.run()

        self.car.Steer(self.steer)
        #print("Steering: ", self.steer)

        self.car.Drive(self.speed)
        #print("Throttle: ", self.speed)


    def shutdown(self):
        self.running = False
        self.car.shutdown()


    def setSteer(self, angleIN):
        '''
        Inputs:
            angleIN : desired steering angle
                      positive => right / negative => left

        Function: converts angleIN to readable value for __Actuate__
        '''
        self.steer = angleIN + 90 + 8  # trim = 8
        return float(self.steer-8)


    def setDrive(self, speedIN):
        '''
        Inputs:
            speedIN : desired speed from [-1,1]
                      pos => forward / neg => backward

        Function: maps speedIN to angle value for __Actuate__
        '''
        slope = (MotorController.maxSpeed-MotorController.minSpeed)/2
        self.speed = (speedIN-MotorController.BACK)*slope + MotorController.minSpeed
        return float(self.speed)

    def getDrive(self):
        return(self.speed)

    def getSteer(self):
        return(self.steer)


class Accel:
    '''
    This class is an open-loop control on the acceleration and steering sensitivity of a car
    '''
    def __init__(self, mc):
        '''
        Inputs:
            mc = MotorController object
        '''
        self.mc = mc


    def rampSpd(self, start, stop=0):
        '''
        Inputs:
            start : start speed of accel/deccel in range [-1,1]
            stop  : stop speed of accel/deccel in range [-1,1]

        Function: controls the acceleration of the car
        '''
        # ACCELERATION
        if start>=stop:
            for spd in np.arange(start, stop, -0.07):   #step value chosen experimentally
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                self.mc.update()
                print(curr_spd)
                time.sleep(0.45)    #delay value chosen experimentally
        # DECCELERATION
        elif start<stop:
            for spd in np.arange(start, stop, 0.07):
                curr_spd = self.mc.setDrive(spd)
                self.mc.run()
                self.mc.update()
                print(curr_spd)
                time.sleep(0.45)
        # CONSTANT SPEED
        elif start == stop:
            curr_spd = self.mc.setDrive(start)
            self.mc.run()
            self.mc.update()
            print(curr_spd)

        else:
            self.mc.stop()


    def rampDir(self, start, stop=0):
        '''
        Inputs:
            start : start direction of turn in range [-1,1]
            stop  : stop directin of turn in range [-1,1]

        Function: controls responsiveness of steering
        '''
        # TURN RIGHT
        if start>=0:
            for dir in np.arange(start, stop, 2):   #step value chosen experimentally
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                self.mc.update()
                print(curr_dir)
                time.sleep(0.09)    #delay value chosen experimentally
        # TURN LEFT
        elif start<0:
            for dir in np.arange(start, stop, -2):
                curr_dir = self.mc.setSteer(dir)
                self.mc.run()
                self.mc.update()
                print(curr_dir)
                time.sleep(0.09)
        # NO TURN
        elif start == stop:
            curr_dir = self.mc.setSteer(start)
            self.mc.run()
            self.mc.update()
            print(curr_dir)

        else:
            self.mc.stop()


    def stop(self):
        '''
        Function: stops car without shutting down
        '''
        curr_spd = self.mc.setDrive(0)
        curr_dir = self.mc.setSteer(0)
        self.mc.run()
        self.mc.update()
        print("Stop ", curr_spd, curr_dir)


if __name__ == "__main__":
    import sys

    car = MotorController()
    car.setDrive(0.3)
    car.setSteer(sys.argv[1])

    try:
        while True:
            car.update()

    except KeyboardInterrupt:
        car.shutdown()
