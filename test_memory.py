
import cv2
import random
from pathlib import Path
import sys
import os
from datetime import datetime
from ARClib.tools import memory

# now = datetime.now()
# dt_string = now.strftime("%b-%d-%y_%H-%M-%S")
# os.makedirs(dt_string)
filepath = Path("C:/Users/jazzy/Documents/KU_ARC/") #personal/testing
# filepath = Path("~/pi/home/Documents/") #RPi
# filepath = filepath / dt_string

mem = memory(filepath)
cap = cv2.VideoCapture(0)

loop = 0
heading = 90 + random.randint(0, 20)
while cap.isOpened():
    _, frame = cap.read()
    cv2.imshow('frame',frame)
    mem.saveImage((frame, heading, loop))

    loop += 1
    heading = 90 + random.randint(-15, 15)

    if cv2.waitKey(1) == ord('q') & 0xFF:
        cap.release()
        cv2.destroyAllWindows()
