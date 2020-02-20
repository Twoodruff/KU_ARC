"""
File: LaneDetect.py
Author: Thomas Woodruff
Date: 11/15/19
Revision: 0.1
Description: Reads in camera frame from video stream
             and detects lane lines. Classify lanes by
             slopes and position in frame. Lane hue_center
             found by average of ends of lanes. Return value
             is array with line end points/heading angle in deg.
"""

import cv2
import numpy as np
import math

from . import cam
from .lane_control import TwoLines, MultiLine


class ImageProcess():
    def __init__(self, detect = [255,0,0], color = [0,255,0]):
        '''
        Inputs:
            detect : color of lanes to detect in BGR
                     default is blue
            color : color of display lines  in BGR
                    default is green
        '''
        det_color = np.uint8([[detect]])
        self.detect_color = cv2.cvtColor(det_color, cv2.COLOR_BGR2HSV)
        self.line_color = color
        self.running = True
        self.track  = TwoLines()


    def run(self, frame):
        '''
        Inputs:
            frame : input that needs processing

        Function: main loop that perfroms color filtering,
                  edge, line, and lane detection, and computes
                  desired heading angle

        Outputs:
            heading_deg : heading angle in degrees
        '''
        if self.running:
            # READ IN FRAME
            self.height, self.width, _ = frame.shape

            # ROTATE FRAME
            self.rot = cam.rotate(frame,180)

            # CROPPING IMAGE
            crop = np.zeros(self.rot.shape, dtype = 'uint8')
            n = 0.5   #determines how much of the frame is cropped in the vertical direction, from the top
            self.top = int(self.height*n)
            cv2.rectangle(crop, (0, self.top), (self.width, self.height), (255, 255, 255), -1)
            self.crop_im = cv2.bitwise_and(src1 = self.rot, src2 = crop)

            self.proj = cam.projective_warp(self.crop_im)

            # CONVERTING COLOR SPACE
            hsv = cv2.cvtColor(self.proj, cv2.COLOR_BGR2HSV)

            # COLOR DETECTION
            hue_center = self.detect_color[0][0][0]
            lower = np.array([hue_center-25,30,70]) #bounds in hsv color space
            upper = np.array([hue_center+25,255,255])
            mask = cv2.inRange(hsv, lower, upper)
            self.res = cv2.bitwise_and(hsv, hsv, mask = mask)

            # EDGE DETECTION
            self.edges = cv2.Canny(self.res,95,135)    #threshold parameters may need tuning for robustness

            # LINE DETECTION
            rho_res = 1
            theta_res = np.pi/180
            threshold = 50
            minLineLength = 20
            maxLineGap = 10
            self.lines = cv2.HoughLinesP(self.edges,rho_res,theta_res,threshold,minLineLength,maxLineGap)

            # LANE DETECTION
            self.lanes = self.track.track(self.rot, self.lines)
            self.heading = self.track.control(self.lanes)


    def update(self):
        return self.heading


    def shutdown(self):
        self.running = False


    # FUNCTIONS TO SHOW DIFFERENT IMAGES
    def showRot(self, cam):
        # SHOW IMAGE AFTER ROTATION
        cv2.imshow('Rotated', self.rot)
        cam.show()
        return self.rot

    def showHSV(self, cam):
        # SHOW IMAGE AFTER COLOR FITLERING
        cv2.imshow('Color', self.res)
        cam.show()
        return self.res

    def showEdge(self, cam):
        # SHOW IMAGE AFTER EDGE DETECTION
        cv2.imshow('Edges', self.edges)
        cam.show()
        return self.edges

    def showHough(self, cam):
        # SHOW IMAGE WITH DETECTED LINES
        new = np.zeros_like(self.rot)
        if self.lines is not None:
            for line in self.lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(new,(x1,y1),(x2,y2),(0,255,0),2)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        # cv2.imshow('Hough', new)
        # cam.show()
        return new

    def showLanes(self, cam):
        # SHOW IMAGE WITH DETECTED LANES
        new = np.zeros_like(self.rot)
        if self.lanes is not None:
            for line in self.lanes:
                for x1,y1,x2,y2 in line:
                    cv2.line(new,(x1,y1),(x2,y2),self.line_color,10)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        cv2.imshow('Lanes', new)
        cam.show()
        return new

    def showHeading(self, cam, heading):
        # SHOW IMAGE WITH DETECTED LANES & HEADING DIRECTION
        new = np.zeros_like(self.rot)
        rad = heading/180.0*math.pi

        if self.lanes is not None:
            x1 = int(self.width/2)
            y1 = self.height
            x2 = int(x1 + (self.height/2)*math.tan(rad))
            y2 = self.top
            cv2.line(new,(x1,y1),(x2,y2),[0,0,255],8)
            for line in self.lanes:
                for x1,y1,x2,y2 in line:
                    cv2.line(new,(x1,y1),(x2,y2),self.line_color,10)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        # cv2.imshow('Heading', new)
        # cam.show()
        return new


if __name__ == "__main__":
    import cam
    import time
    from tools import median

    camObj = cam.camera(0)
    LaneDetect = LaneKeep()
    medFilter = median(4)
    while(1):
        camObj.run()
        fix = camObj.update()
        LaneDetect.run(fix)
        steeri = LaneDetect.update()
        steer = medFilter.run(steeri)

        print("steer = ", steer)
        LaneDetect.showLanes(camObj)
        LaneDetect.showHeading(camObj)
        filename = 'C:/Users/jazzy/Documents/Python/pics/heading_' + str(i) + '.jpg'
        cv2.imwrite(filename,LaneDetect.headingcam)

        #time.sleep(0.5)
