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

class Distance:

    '''
    HC-SR04 proximity sensor sends out ultrasonic waves and records time is takes for the
    waves to return to the sensor. Knowing the speed of sound at sea level is 343 m/s we can
    find distance from sensor/car at any time
    '''

    def __init__(self):
        self.TRIG=23
        self.ECHO=24
        GPIO.setup(self.TRIG,GPIO.OUT)
        GPIO.setup(self.ECHO,GPIO.IN)
        GPIO.output(self.TRIG, False)
        time.sleep(2)
        print("settling sensor")
        self.running = True

    def run(self):
        if self.running:
            new_reading = False
            counter = 0
            #send trigger pulse
            GPIO.output(self.TRIG,True)
            time.sleep(.00001)
            GPIO.output(self.TRIG,False)
            
            while GPIO.input(self.ECHO)==0:
                pass
                print("getting start time")
                counter+=1
                if counter>15:
                    new_reading = True
                    break
            pulse_start = time.time()
        
            if new_reading:
                return False
            
            while GPIO.input(self.ECHO)==1:
                pass
                print("getting stop time")
            pulse_end = time.time()

            total_pulse_time = (pulse_end - pulse_start)*.5
            self.OUT = 343*total_pulse_time #distance is in meters
               
    def update(self):
        if self.OUT<4:
            return self.OUT
        else:
            return False

    def shutdown(self):
        self.running = False
        GPIO.cleanup()
       
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





