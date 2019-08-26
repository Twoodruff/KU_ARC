"""
File: test_OL.py
Author: Thomas Woodruff
Date: 8/20/19
Revision: 0.9.1
Description: Test code for main with open-loop control.
"""

#from keyboard_OL import KC
import time
#from moco import MotorController, Accel
import sys

import threading
import queue

import cv2
import numpy as np

curr_spd = 0.0
exit_flag = 0
CAM_PORT = 0    #default is 0
timeout = 0.5

#car = MotorController()
#control = KC(car)
#car.setDrive(curr_spd)
#car.run()

cap = cv2.VideoCapture(CAM_PORT,cv2.CAP_DSHOW)                                  #create camera instance
input_queue = queue.Queue()


def camera_op():                                                                #reads and displays video
    #take a snapshot
    while not exit_flag:
        ret, frame = cap.read()
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):                                   #waitKey() statement needed for imshow
            break                                                               #shouldn't need later on
    print('Camera closed')

class move_op(threading.Thread):                                                #subclass for Thread that calls the keyboard control function
    def __init__(self,queue):
        threading.Thread.__init__(self)
        self.queue = queue

    def run(self):
        while not exit_flag:
            try:
                key = self.queue.get()
                #control.keyControl(key)
                print("motion processing: {}".format(key))
            except queue.Empty:
                # if no more input, exit
                break


cam_thread = threading.Thread(target = camera_op)                               #create camera Thread
cam_thread.start()

move_thread = move_op(input_queue)                                              #create motion control Thread
move_thread.start()

while not exit_flag:
    try:
        comm = sys.stdin.readline()                                             #get input from the cmd
        key = ord(comm[0])
        input_queue.put(key)                                                    #put the input in the queue
        #control.keyControl(key)
    except KeyboardInterrupt:                                                   #if Ctrl-C is pressed, end everything
        exit_flag = 1
        cam_thread.join()
        move_thread.join()
        input_queue.join()
        #control.keyControl(ord('q'))
        cap.release()
        cv2.destroyAllWindows()
        pass                   #redundant?

sys.exit(1)
