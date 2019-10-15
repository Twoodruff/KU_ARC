"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 10/14/19
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

# GLOBAL VARIABLES
drivefreq = 10 #Hz
curr_spd = 0.0 #[-1,1]
curr_dir = 0
dt = 1 / drivefreq
exit_flag = 0
CAM_PORT = 0
timeout = 0.5  #seconds

# PART OBJECTS
car = MotorController()
cam = camera(CAM_PORT)
control = LaneKeep(cam)
pid = PID(0.5,0,0)
input_queue = queue.Queue()

# CAMERA THREAD
cam_thread = threading.Thread(target = cam.run)
cam_thread.start()

# #create control thread
# control_thread = threading.Thread(target = control.run)
# control_thread.start()

# MOTION CONTROL THREAD
move_thread = threading.Thread(target = car.run)
move_thread.start()

while not exit_flag:
    try:
        # START LOOP TIMER
        start_loop = time.time_ns()

        # GET CAMERA INPUT
        frame = cam.update()

        # COMPUTE SETPOINT HEADING ANGLE
        control.run(frame)
        heading = control.update()
        pid.setSP(heading)

        # PID LOOP
        while abs(pid.getErr()) > 0.1:
            print('Loop #: ', n)
            print('Speed: ', curr_dir)
            new_dir = pid.update(curr_dir, dt)
            #move.rampDir(curr_dir, new_dir)
            car.setSteer(new_dir)
            curr_dir = new_dir

        # APPLY NEW DIRECTION
        car.update()

        # END LOOP AND WAIT
        loop_time = time.time_ns() - start_loop
        time.sleep(dt - loop_time/1e9)

    #if Ctrl-C is pressed, end everything
    except KeyboardInterrupt:
        exit_flag = 1
        car.shutdown()
        cam.shutdown()
        control.shutdown()
        cam_thread.join()
        move_thread.join()
        pass    #redundant?

sys.exit(1)
