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

class Distance:

    def __init__(self):
        self.sensor = DistanceSensor(23,24)
        self.OUT = 0
        self.running = True

    def run(self):
        if self.running:
            while True:
                print('Distance to the nearest object is',self.sensor.distance,'m')
                self.OUT = self.sensor.distance
                time.sleep(1)
               
    def update(self):
        return self.OUT

    def shutdown(self):
        self.running = False
       
if __name__ == "__main__":
    testing_dis = Distance()
    loop = 0
    try:
        while True:
            output = testing_dis.run()
            output = testing_dis.update()
            print("\nloop: ", loop)
            print("Distance:",output, "m")
            loop+=1
    except KeyboardInterrupt:
        testing_dis.shutdown()





