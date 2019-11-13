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

   def __init__(self,trig,echo):
        self.TRIG=trig
        self.ECHO=echo
        #self.OUT=0
        GPIO.setup(self.TRIG,GPIO.OUT)
        GPIO.setup(self.ECHO,GPIO.IN)
        GPIO.setup(4,GPIO.OUT)
        GPIO.setup(17,GPIO.OUT)
        GPIO.output(17,False)
        GPIO.output(4,True)

        self.running = True

   def run(self):
       self.pulse_end=0
       if self.running:
           GPIO.output(self.TRIG,True)
           time.sleep(.00001)
           GPIO.output(self.TRIG,False)

            while GPIO.input(self.ECHO)==0:
                pulse_start = time.time()

           while GPIO.input(self.ECHO)==1:
               self.pulse_end = time.time()
            
           if self.pulse_end==0:
               total_pulse_time = .0117
           else:
               total_pulse_time = (self.pulse_end - pulse_start)*.5
        
           self.OUT = 343*total_pulse_time #distance is in meters
               

    def update(self):
        return self.OUT


   def shutdown(self):
       self.running = False
       GPIO.cleanup()
       
if __name__ == "__main__":
    testing_dis = Distance(trig=23,echo=24)
    loop=0
    try:
        while True:
            testing_dis.run()
            output=testing_dis.update()
            print("\nloop: ", loop)
            print(output)
            loop+=1
    except KeyboardInterrupt:
        testing_dis.shutdown()





