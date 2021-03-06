"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 1/2/20
Revision: 0.1
Description: Test code for main with lane keeping.
"""

from ARClib.cam import camera
from ARClib.image_process import ImageProcess
from ARClib.tools import median
from ARClib.logger import ImageLogger

import time
import sys
from pathlib import Path
import threading
import queue
import os

# GLOBAL VARIABLES
drivefreq = 10  # Hz
dt = 1 / drivefreq  # sec

filepath = Path("C:/Users/jazzy/Documents/KU_ARC/") #personal/testing
# filepath = Path("/home/pi/Documents/KU_ARC/") #RPi

exit_flag = 0
filter_size = 3

# PART OBJECTS
cam = camera('testrun2_b.avi')
print("Camera fps: ",cam.fps)
control = ImageProcess()
medFilter = median(filter_size)
mem = ImageLogger(filepath)

# THREADS
image_queue = queue.Queue()

def memory_op():
    while True:
        try:
            image = image_queue.get()
            mem.saveImage(image)
            image_queue.task_done()
        except image_queue.Empty():
            print('Breaking from thread')
            break

mem_thread = threading.Thread(target = memory_op, name='memory')
mem_thread.start()

# LOOP INITIALIZATIONS
heading = 0
prev_head = heading
loop = 1

# DRIVE LOOP
while not exit_flag:
    try:
        # START LOOP TIMER
        start_loop = time.time_ns()

        # GET CAMERA INPUT
        for cam_count in range(3):
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
        heading = medFilter.run(heading)
        control_time = (end - start)/1e6

        # PREVENT OVERSTEERING
        if (heading-prev_head) > 20:
            head = prev_head + 20
        elif (heading-prev_head) < -20:
            head = prev_head - 20
        else:
            head = heading

        # SHOW LANES
        control.showLanes(cam)

        # SAVE IMAGE WITH HEADING FOR TROUBLESHOOTING
        start = time.time_ns()
        image_queue.put((control.showHeading(cam, head), head, loop))
        end = time.time_ns()
        mem_time = (end - start)/1e6
        #print("queue size: ", image_queue.qsize())

        # END LOOP AND WAIT
        prev_head = head
        loop += 1

        loop_time = (time.time_ns() - start_loop)/1e6
        extra_time = dt-loop_time/1e9
        if extra_time >= 0:
            time.sleep(extra_time)

        print("cam time: {}\ncontrol time: {}\nmemory time: {}\nloop time: {}\n".format(cam_time,control_time,mem_time,loop_time))


    #if Ctrl-C is pressed, end everything
    except (KeyboardInterrupt, AttributeError) as err:
        print(err)
        exit_flag = 1
        cam.shutdown()
        control.shutdown()
        image_queue.join()
        mem_thread.join(timeout=3)
        print('Loop count: ', loop)
        break

print('Queue size: ', image_queue.qsize())
print('Threads alive: ', threading.enumerate())
print('System Exiting...')
image_queue = None
mem_thread = None
cam = None
control = None
os._exit(0)
