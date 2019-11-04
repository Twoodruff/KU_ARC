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
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

class distance:

   '''
   HC-SR04 proximity sensor sends out ultrasonic waves and records time is takes for the
   waves to return to the sensor. Knowing the speed of sound at sea level is 343 m/s we can
   find distance from sensor/car at any time
   '''
    def __init__(self,inputs):
        self.TRIG=23
        self.ECHO=24
        GPIO.setup(self.TRIG,GPIO.OUT)
        GPIO.setup(self.ECHO,GPIO.IN)
        GPIO.output(self.TRIG,False)
        time.sleep(.02)
       # INITIALIZE VARS
        self.running = True

    def run(self, inputs):
        while self.running:
            GPIO.output(self.TRIG,True)
            time.sleep(.00001)
            GPIO.output(self.TRIG,False)

            while GPIO.input(self.ECHO)==0:
                pulse_start = time.time()

            while GPIO.input(self.ECHO)==1:
                pulse_end = time.time()

            total_pulse_time = (pulse_end - pulse_start)*.5
            distance = 343*total_pulse_time #distance is in meters
            self.OUT = distance

    def update(self):
        return self.OUT


    def shutdown(self):
        GPIO.cleanup()
        self.running = False
