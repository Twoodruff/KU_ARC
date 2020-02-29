#! /usr/bin/env python3
from __future__ import print_function
import numpy as np
import time
import sys
from pathlib import Path

import imutils
from imutils.video import VideoStream
import cv2

print("Creating stream")
vs = VideoStream(src = 0, usePiCamera = False, resolution = (680,420), framerate = 30).start()
fourcc = cv2.VideoWriter_fourcc(*"XVID")
frame = vs.read()

(h, w) = frame.shape[:2]
# filepath = Path("/home/pi/Documents/KU_ARC/testrun.avi")
writer = cv2.VideoWriter("one_line_data.avi", fourcc, 30, (w, h), True)
zeros = np.zeros((h, w), dtype="uint8")

print("Capturing video...")
while True:
    try:
        frame = vs.read()
        writer.write(frame)

    except KeyboardInterrupt:
        break

vs.stop()
writer.release()
print("Video caputured")