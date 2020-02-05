from ARClib import cam
from ARClib.cam import camera
import cv2
import numpy as np


cam1 = camera('C:/Users/jazzy/Documents/KU_ARC/testrun2.avi')

while True:
    cam1.run()
    proj = cam.projective_warp(cam1.frame)
    rot_proj = cam.rotate(proj, 180)
    rot = cam.rotate(cam1.frame, 180)

    cv2.imshow('Rotated', rot)
    cv2.imshow('Projected', rot_proj)
    cam1.show()
