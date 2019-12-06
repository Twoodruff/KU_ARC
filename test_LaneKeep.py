"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 11/14/19
Revision: 0.1
Description: Test code for main with lane keeping.
"""

from ARClib.moco import MotorController
from ARClib.cam import camera
from ARClib.LaneKeep import LaneKeep
from ARClib.tools import median, memory

import time
import sys
from pathlib import Path

# GLOBAL VARIABLES
drivefreq = 9  # Hz
dt = 1 / drivefreq  # sec

curr_spd = 0.3  # [-1,1]
curr_dir = 0

exit_flag = 0
CAM_PORT = 0
filter_size = 3

#filepath = Path("C:/Users/jazzy/Documents/KU_ARC/") #personal/testing
filepath = Path("/home/pi/Documents/KU_ARC/") #RPi

# PART OBJECTS
car = MotorController()
cam = camera(CAM_PORT)
control = LaneKeep()
medFilter = median(filter_size)
mem = memory(filepath)

# LOOP INITIALIZATIONS
heading = 0
prev_head = heading
loop = 1
car.setDrive(curr_spd)

# DRIVE LOOP
while not exit_flag:
    try:
        # START LOOP TIMER
        start_loop = time.time_ns()

        # GET CAMERA INPUT
        cam.run()
        frame = cam.update()

        # COMPUTE SETPOINT HEADING ANGLE
        control.run(frame)
        heading = control.update()
        #heading = medFilter.run(headingi)

        # PREVENT OVERSTEERING
        if (heading-prev_head) > 10:
            head = car.setSteer(prev_head + 10)
        elif (heading-prev_head) < -10:
            head = car.setSteer(prev_head - 10)
        else:
            head = car.setSteer(heading)

        # APPLY CONTROL INPUTS
        car.update()

        # SHOW LANES
        #control.showHeading(cam, head-90)

        # SAVE IMAGE WITH HEADING FOR TROUBLESHOOTING
        mem.saveImage((control.showHeading(cam, head-90), head-90, loop))

        # END LOOP AND WAIT
        prev_head = head - 90
        loop += 1

        loop_time = time.time_ns() - start_loop
        extra_time = dt-loop_time/1e9
        if extra_time >= 0:
            time.sleep(extra_time)
        else:
            print("loop time: ",loop_time/1e6-dt*1e3)
            #time.sleep(dt-extra_time)

    #if Ctrl-C is pressed, end everything
    except KeyboardInterrupt:
        exit_flag = 1
        car.shutdown()
        cam.shutdown()
        control.shutdown()
        print(loop)
        pass

sys.exit(1)
