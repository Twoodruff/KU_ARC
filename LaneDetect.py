"""
File: LaneDetect.py
Author: Thomas Woodruff
Date: 9/11/19
Revision: 0.1.1
Description: Reads in camera frame from video stream
             and detects lane lines. Return value is
             array with line end points.
"""

import cv2
import numpy as np


class LaneDetect():
    def __init__(self, cam = 0, detect = [255,0,0], color = [0,255,0]):
        self.cap = cv2.VideoCapture(cam)

        _, self.frame = cap.read()
        self.height, self.width, _ = self.frame.shape

        det_color = np.uint8([[detect]])
        self.detect_color = cv2.cvtColor(det_color, cv2.COLOR_BGR2HSV)

        self.line_color = color


    def rotate(image, angle):
        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        (cX, cY) = (w / 2, h / 2)

        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise)
        M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

        # perform the actual rotation and return the image
        return cv2.warpAffine(image, M, (w, h))


    def avg_lines(frame, lines):
        import numpy.polynomial.polynomial as poly
        lane_line = []
        ht, wt, _ = frame.shape

        left_s = []
        right_s = []

        if lines is None:
            print('No lines to detect')
            return lane_line

        for line in lines:
            for x1,y1,x2,y2 in line:
                #find slope using points and categorize left from right
                fit = poly.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[1]
                intercept = fit[0]
                if slope < 0:
                    if x2 < wt/2:
                        left_s.append((slope,intercept))
                elif slope > 0:
                    if x2 > wt/2:
                        right_s.append((slope,intercept))

        # average lines (slope, int)
        avgl = np.mean(left_s, axis = 0)
        #print("avgl", avgl)                         #debug

        avgr = np.mean(right_s, axis = 0)
        #print("avgr", avgr)                         #debug

        #create lane lines based on categorization
        if len(left_s) > 0:
            slopel = avgl[0]
            intl = avgl[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intl)/slopel))
            x2 = max(-wt,min(2*wt,(y2-intl)/slopel))
            lane_line.append([round(x1),y1,round(x2),y2])

        if len(right_s) > 0:
            sloper = avgr[0]
            intr = avgr[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intr)/sloper))
            x2 = max(-wt,min(2*wt,(y2-intr)/sloper))
            lane_line.append([round(x1),y1,round(x2),y2])

        #print("lane lines", lane_line)              #debug
        return lane_line


    def run(self):
        # ROTATE FRAME
        self.rot = rotate(self.frame, 180)

        # CONVERTING COLOR SPACE
        self.hsv = cv2.cvtColor(self.rot, cv2.COLOR_BGR2HSV)

        # COLOR DETECTION
        hue_center = self.detect_color[0][0][0]
        #bounds in hsv color space
        lower = np.array([hue_center-25,30,90])
        upper = np.array([hue_center+25,255,255])
        mask = cv2.inRange(self.hsv, lower, upper)
        self.res = cv2.bitwise_and(self.hsv, self.hsv, mask = mask)

        # CROPPING IMAGE
        crop = np.zeros(self.res.shape, dtype = 'uint8')
        n = 2 #determines how much of the frame is cropped in the vertical direction
        cv2.rectangle(crop, (0, self.height/n), (self.width, self.height), (255, 255, 255), -1)
        self.crop_im = cv2.bitwise_and(src1 = self.res, src2 = crop)

        # EDGE DETECTION
        self.edges = cv2.Canny(self.crop_im ,100,200) #threshold parameters may need tuning for robustness

        # LINE DETECTION
        minLineLength = 100
        maxLineGap = 20
        self.lines = cv2.HoughLinesP(self.edges,1,np.pi/180,50,minLineLength,maxLineGap)

        self.lanes = avg_lines(self.rot, self.lines)

        return self.lanes


<<<<<<< HEAD
    def show():
        k = cv2.waitKey(1)
        if k = ord('q') & 0xFF:
            shutdown()

    def showRot(self):
        cv2.imshow('Rotated', self.rot)
        show()

    def showHSV(self):
        cv2.imshow('Color', self.res)
        show()

    def showEdge(self):
        cv2.imshow('Edges', self.edges)
        show()
=======
    def showRot(self):
        cv2.imshow('Rotated', self.rot)

    def showHSV(self):
        cv2.imshow('Color', self.res)

    def showEdge(self):
        cv2.imshow('Edges', self.edges)
>>>>>>> Issue #7: format test code into class

    def showHough(self):
        new = np.zeros_like(self.rot)
        if self.lines is not None:
            for line in self.lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(new,(x1,y1),(x2,y2),(0,255,0),2)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        cv2.imshow('Hough', new)
<<<<<<< HEAD
        show()
=======
>>>>>>> Issue #7: format test code into class

    def showLanes(self):
        new = np.zeros_like(self.rot)
        if self.lanes is not None:
            for x1,y1,x2,y2 in self.lanes:
                cv2.line(new,(x1,y1),(x2,y2),(0,255,0),10)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        cv2.imshow('Lanes', new)
<<<<<<< HEAD
        show()
=======
>>>>>>> Issue #7: format test code into class

    def shutdown(self):
        cv2.release()
        cv2.destroyAllWindows()
