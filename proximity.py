'''
File: UltrasonicSensor.py
Author: Ryan Strong
Date: 11/4/2019
Revision: 0.1
Description: Ultrasonic sensor thread, used to measure distance by sending 8 40kHz signals out and measuring the
             time taken to receive a signal back. Using this time and speed of sound, distance is found.
             Module must include __init__, run, update, shutdown
'''
# IMPORTS
import time
from gpiozero import DistanceSensor
from time import sleep

class distance:

    def __init__(self):
        self.sensor = DistanceSensor(23,24)
       # INITIALIZE VARS
        self.running = True

    def run(self):
        if self.running:
            while True:
                print('Distance to the nearest object is', self.sensor.distance,'m')
                self.OUT = self.sensor.distance
                time.sleep(1)

    def update(self):
        return self.OUT


    def shutdown(self):
        self.running = False
