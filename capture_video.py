#! /usr/bin/env python3

import imutils
from imutils.video import VideoStream
import cv2

import numpy as np
import time
from __future__ import print_function
import sys

vs = VideoStream(src = 1, usePiCamera = False, resolution = (680,420), framerate = 30).start()
fourcc = cv2.VideoWriter_fourcc('MJPG')
frame = vs.read()

(h, w) = frame.shape[:2]
writer = cv2.VideoWriter('testrun_'+sys.argv[1], fourcc, 30, (w, h), True)
zeros = np.zeros((h, w), dtype="uint8")

while True:
    try:
        frame = vs.read()
        writer.write(frame)

    except KeyboardInterrupt:
        break

vs.stop()
writer.release()
