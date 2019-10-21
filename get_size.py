import numpy as np
import cv2
import sys

cap = cv2.VideoCapture(0)

_,frame = cap.read()
filename = 'pics/pic' + sys.argv[1] + '.jpg'
cv2.imwrite(filename,frame)

dim = frame.shape

height = frame.shape[0]
width = frame.shape[1]
channels = frame.shape[2]

# print('Image Dimension    : ',dim)
# print('Image Height       : ',height) #480
# print('Image Width        : ',width)  #640
# print('Number of Channels : ',channels)

print("image taken")
