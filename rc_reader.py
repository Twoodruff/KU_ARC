"""
File: rc_reader.py
Author: Thomas Woodruff
Date: 4/19/19
Description: Reads value from car receiver as pulse time &
             converts it to angle for actuators.
"""

import time
#import machine -> must download using pip
from motor_controller import MotorController

class reader:
    
    def __init__(self, pin1, pin2):
        """
        need definitions for declaring GPIO on Pi
        use circuitpython most likely
        """
        
    def calcSignal(self, pin):
        if(pin is high):
            start = time.time()
        else:
            stop = time.time()
            pulse = stop - start