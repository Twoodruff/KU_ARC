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
from ARClib.tools import median

import time
import sys

# GLOBAL VARIABLES
drivefreq = 10  # Hz
dt = 1 / drivefreq  # sec

curr_spd = 0.3  # [-1,1]
curr_dir = 0

exit_flag = 0
CAM_PORT = 0
filter_size = 3

# PART OBJECTS
car = MotorController()
cam = camera(CAM_PORT)
control = LaneKeep()
medFilter = median(filter_size)

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
        headingi = control.update()
        heading = medFilter.run(headingi)

        # PREVENT OVERSTEERING
        if (heading-prev_head) > 10:
            head = car.setSteer(prev_head + 10)
        elif (heading-prev_head) < -10:
            head = car.setSteer(prev_head - 10)
        else:
            head = car.setSteer(heading)

        # APPLY CONTROL INPUTS
        car.update()

        prev_head = head - 90
        loop += 1

        # SHOW LANES
        #control.showHeading(cam, head-90)

        # END LOOP AND WAIT
        loop_time = time.time_ns() - start_loop
        extra_time = dt-loop_time/1e9
        if extra_time >= 0:
            time.sleep(extra_time)
        else:
            time.sleep(dt-extra_time)

    #if Ctrl-C is pressed, end everything
    except KeyboardInterrupt:
        exit_flag = 1
        car.shutdown()
        cam.shutdown()
        control.shutdown()
        print(loop)
        pass

sys.exit(1)
