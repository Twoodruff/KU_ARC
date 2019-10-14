"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 10/10/19
Revision: 0.1
Description: Test code for main with lane keeping.
"""

from ARClib.moco import MotorController
from ARClib.cam import camera
from ARClib.LaneKeep import LaneKeep
from ARClib.PID import PID

import time
import sys
import threading
import queue
import cv2
import numpy as np

#global variables
curr_spd = 0.0
exit_flag = 0
CAM_PORT = 0
timeout = 0.5

# Create part objects
car = MotorController()
cam = camera(CAM_PORT)
control = LaneKeep(cam)
pid = PID()
input_queue = queue.Queue()

#subclass for Thread that calls the movement functions
class move_op(threading.Thread):
    def __init__(self,queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.x = 0

    def run(self):
        while not exit_flag:
            try:
                #move control
                self.x += 1
            except queue.Empty:
                # if no more input, exit
                break

#create camera Thread
cam_thread = threading.Thread(target = cam.run)
cam_thread.start()

#create control thread
control_thread = threading.Thread(target = control.run)
control_thread.start()

#create motion control Thread
move_thread = move_op(input_queue)
move_thread.start()

while not exit_flag:
    try:
        #put the input in the queue
        input_queue.put()

    #if Ctrl-C is pressed, end everything
    except KeyboardInterrupt:
        exit_flag = 1
        cam_thread.join()
        move_thread.join()
        input_queue.join()
        control.keyControl(ord('q'))
        #cap.release()
        #cv2.destroyAllWindows()
        pass    #redundant?

sys.exit(1)
