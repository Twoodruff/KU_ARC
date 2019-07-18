"""
File: keyboard_OL.py
Author: Thomas Woodruff
Date: 4/22/19
Description: Reads value from keyboard and passes commands
             to the car.
"""

from moco import MotorController, Accel
import sys

class KC:
    
    def __init__(self, car):
        self.mc = car
        self.move = Accel(car)
        self.called = False
    
    def forward(self):
        '''
        ramp function to full speed
        '''
        print('Forward')
        self.move.rampSpd(0, 0.42) #car will run for ~1.5s
        self.move.rampSpd(0.42)
##        count = 0
##        if not self.called:
##            while count<15:
##                if self.called:
##                    break
##                elif count<15:
##                    time.sleep(0.1)
##                    count = count + 1
##                else:
##                    move.stop()
        self.move.stop()
        
    def backward(self):
        '''
        ramp function down to 0
        '''
        print("Backward")
        self.move.rampSpd(0, -0.56) #car will run for ~1.5s
        self.move.rampSpd(-0.56)
        self.move.stop()
        
    def left(self):
        print("Left")
        self.mc.setSteer(-15)
        self.move.rampSpd(0, 0.42)
        self.move.rampSpd(-0.42)
        self.move.stop()
        
    def right(self):
        print("Right")
        self.mc.setSteer(15)
        self.move.rampSpd(0, 0.42)
        self.move.rampSpd(-0.42)
        self.move.stop()
        
    def quit(self):
        self.mc.shutdown()
        sys.exit()
    
    direction = {
        'w': forward,
        's': backward,
        'a': left,
        'd': right,
        'q': quit
    }
    
    def keyControl(self, key):
        func = self.direction.get(chr(key), lambda: "Not valid") #fix lambda
        func(self)
        

    #need getter for current speed var