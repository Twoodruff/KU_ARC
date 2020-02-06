"""
File: test_LaneKeep.py
Author: Thomas Woodruff
Date: 1/2/20
Revision: 0.1
Description: Test code for main with lane keeping.
"""

from ARClib.cam import camera
from ARClib.LaneKeep import LaneKeep
from ARClib.tools import median, memory

import time
import sys
from pathlib import Path
import threading
import queue

# GLOBAL VARIABLES
drivefreq = 10  # Hz
dt = 1 / drivefreq  # sec

filepath = Path("C:/Users/jazzy/Documents/KU_ARC/") #personal/testing
#filepath = Path("/home/pi/Documents/KU_ARC/") #RPi

exit_flag = 0
filter_size = 3

# PART OBJECTS
cam = camera('testrun2.avi')
print("Camera fps: ",cam.fps)
control = LaneKeep()
medFilter = median(filter_size)
mem = memory(filepath)

# THREADS
image_queue = queue.Queue()

def memory_op():
    while not exit_flag:
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
        control_time = (end - start)/1e6
        heading = medFilter.run(heading)

        # PREVENT OVERSTEERING
        if (heading-prev_head) > 20:
            head = prev_head + 20
        elif (heading-prev_head) < -20:
            head = prev_head - 20
        else:
            head = heading

        # SHOW LANES
        # control.showHeading(cam, head)

        # SAVE IMAGE WITH HEADING FOR TROUBLESHOOTING
        start = time.time_ns()
        image_queue.put((control.showHeading(cam, head), head, loop))
        #image_queue.put((control.showHough(cam), head, loop))
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
    except KeyboardInterrupt:
        exit_flag = 1
        cam.shutdown()
        control.shutdown()
        mem_thread.join(timeout=3)
        image_queue.join()
        print(loop)
        sys.exit(0)
        break

sys.exit(0)
