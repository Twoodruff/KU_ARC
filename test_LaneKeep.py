"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 2/6/20
Revision: 0.1
Description: Test code for main with lane keeping.
"""

from ARClib.moco import MotorController
from ARClib.cam import camera
from ARClib.image_process import ImageProcess
from ARClib.lane_control import OneLine
from ARClib.tools import median, memory

import time
import sys
from pathlib import Path
import threading
import queue

# GLOBAL VARIABLES
drivefreq = 8  # Hz
dt = 1 / drivefreq  # sec

curr_spd = 0.32  # [-1,1]
curr_dir = 0

exit_flag = 0
CAM_PORT = 0
filter_size = 3

#filepath = Path("C:/Users/jazzy/Documents/KU_ARC/") #personal/testing
filepath = Path("/home/pi/Documents/KU_ARC/") #RPi

# PART OBJECTS
car = MotorController()
cam = camera(CAM_PORT)
control = ImageProcess(P=185, I=45, D=25)
medFilter = median(filter_size)
mem = memory(filepath)

# THREADS
image_queue = queue.Queue()

def memory_op():
    while True:
        try:
            image = image_queue.get()
            mem.saveImage(image)
            image_queue.task_done()
        except queue.Empty:
            break

mem_thread = threading.Thread(target = memory_op)
mem_thread.start()


# LOOP INITIALIZATIONS
heading = 0
prev_head = heading
loop = 1
car.setDrive(curr_spd)
car.setSteer(curr_dir)
cam.run() #grab initial frame to reduce capture time on first loop
car.update()

# DRIVE LOOP
while not exit_flag:
    try:
        # START LOOP TIMER
        start_loop = time.time_ns()

        # GET CAMERA INPUT
        start = time.time_ns()
        cam.run()
        frame = cam.update()
        end = time.time_ns()
        cam_time = (end - start)/1e6

        # COMPUTE SETPOINT HEADING ANGLE
        start = time.time_ns()
        control.run(frame)
        heading = control.update()
        end = time.time_ns()
        control_time = (end - start)/1e6
        heading = medFilter.run(heading)

        # PREVENT OVERSTEERING
        if (heading-prev_head) > 20:
            car.setSteer(prev_head + 20)
        elif (heading-prev_head) < -20:
            car.setSteer(prev_head - 20)
        else:
            car.setSteer(heading)

        # APPLY CONTROL INPUTS
        start = time.time_ns()
        car.update()
        head = car.getSteer()
        end = time.time_ns()
        car_time = (end - start)/1e6

        # SHOW LANES
        #control.showHeading(cam, head-90)

        # SAVE IMAGE WITH HEADING FOR TROUBLESHOOTING
        start = time.time_ns()
        image_queue.put((control.showHSV(cam), head-90, loop))
        end = time.time_ns()
        mem_time = (end - start)/1e6
        # #print("queue size: ", image_queue.qsize())

        # END LOOP AND WAIT
        prev_head = head - 90
        loop += 1

        loop_time = (time.time_ns() - start_loop)/1e6
        extra_time = dt-loop_time/1e9
        if extra_time >= 0:
            time.sleep(extra_time)

        print("cam time: {}\ncontrol time: {}\ncar time: {}\nmemory time: {}\nloop time: {}\n".format(cam_time,control_time,car_time,mem_time,loop_time))


    #if Ctrl-C is pressed, end everything
    except KeyboardInterrupt:
        exit_flag = 1
        car.shutdown()
        control.shutdown()
        mem_thread.join(timeout=3)
        image_queue.join()
        print(loop)
        break

sys.exit(0)