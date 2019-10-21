"""
File: LaneDetect.py
Author: Thomas Woodruff
Date: 10/14/19
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


class LaneKeep():
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
            height, width, _ = frame.shape

            # ROTATE FRAME
            self.rot = cam.rotate(frame,180)

            # CONVERTING COLOR SPACE
            hsv = cv2.cvtColor(self.rot, cv2.COLOR_BGR2HSV)

            # COLOR DETECTION
            hue_center = self.detect_color[0][0][0]
            #bounds in hsv color space
            lower = np.array([hue_center-25,30,90])
            upper = np.array([hue_center+25,255,255])
            mask = cv2.inRange(hsv, lower, upper)
            self.res = cv2.bitwise_and(hsv, hsv, mask = mask)

            # CROPPING IMAGE
            crop = np.zeros(self.res.shape, dtype = 'uint8')
            n = 2   #determines how much of the frame is cropped in the vertical direction
            top = int(height/n)
            cv2.rectangle(crop, (0, top), (width, height), (255, 255, 255), -1)
            crop_im = cv2.bitwise_and(src1 = self.res, src2 = crop)

            # EDGE DETECTION
            self.edges = cv2.Canny(crop_im ,100,200)    #threshold parameters may need tuning for robustness

            # LINE DETECTION
            minLineLength = 100
            maxLineGap = 20
            self.lines = cv2.HoughLinesP(self.edges,1,np.pi/180,50,minLineLength,maxLineGap)

            # LANE DETECTION
            self.lanes = self.avg_lines(self.rot, self.lines)

            # COMPUTE HEADING ANGLE
            prevxoff = 0
            if len(self.lanes)>1:
                #if both lanes are detected, find the middle
                _, _, x_left, _ = self.lanes[0][0]  #first row, first column
                _, _, x_right, _ = self.lanes[1][0] #second row, first column
                x_off = (x_left + x_right)/2 - int(width/2)  #offset from frame center
            elif len(self.lanes)>0:
                #if only lane is detected
                x1, _, x2, _ = self.lanes[0][0]
                x_off = x2-x1
            else:
                #if no lanes are detected, use previous heading
                x_off = prevxoff

            self.prevxoff = x_off
            y_off = int(height/2)
            heading_rad = math.atan(x_off/y_off)            #compute heading angle (rad)
            self.heading_deg = int(heading_rad * 180 / math.pi)  #convert to deg


    def update(self):
        return self.heading_deg


    def shutdown(self):
        self.running = False


    def avg_lines(self, frame, lines):
        '''
        Inputs:
            frame : image that needs processing

            lines : set of lines detected form Hough Transform

        Function: uses detected lines to determine the lanes
                  of the track

        Outputs:
            lane_line : left and right lane of the track
        '''
        import numpy.polynomial.polynomial as poly
        # INITIALIZE ARRAYS
        lane_line = []
        ll_s = []
        lr_s = []
        rr_s = []
        rl_s = []

        # GET IMAGE SIZE
        ht, wt, _ = frame.shape

        # RETURN IF NO LINES ARE FOUND
        if lines is None:
            print('No lines to detect')
            return lane_line

        # CLASSIFY LINES TO LEFT OR RIGHT LANE
        for line in lines:
            for x1,y1,x2,y2 in line:
                #find slope using points and categorize left from right
                fit = poly.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[1]
                intercept = fit[0]

                #classification of lines
                if slope < 0:
                    if x2 < int(wt*(2/3)):
                        #neg slope on left side (straight/right turn)
                        ll_s.append((slope,intercept))
                    else:
                        #neg slope on right side (right turn)
                        lr_s.append((slope,intercept))
                elif slope > 0:
                    if x2 >  int(wt*(1/3)):
                        #pos slope on right side (straight/left turn)
                        rr_s.append((slope,intercept))
                    else:
                        #pos slope on left side (left turn)
                        rl_s.append((slope,intercept))
                else:
                    pass

        # FIND AVG OF EACH TYPE OF LINE
        avglr = np.mean(lr_s, axis = 0)
        #print("\nRight", avglr)

        avgll = np.mean(ll_s, axis = 0)
        #print("Strt/Right", avgll)
                              #debug
        avgrr = np.mean(rr_s, axis = 0)
        #print("Strt/Left", avgrr)

        avgrl = np.mean(rl_s, axis = 0)
        #print("Left", avgrl)

        # CREATE LANES BASED ON AVG's
        #left lane
        if len(ll_s) > 0:
            left_lane = self.lane(ht,wt,avgll[0],avgll[1])
            lane_line.append([left_lane])
        elif len(rl_s) > 0:
            left_lane = self.lane(ht,wt,avgrl[0],avgrl[1])
            lane_line.append([left_lane])

        #right lane
        if len(rr_s) > 0:
            right_lane = self.lane(ht,wt,avgrr[0],avgrr[1])
            lane_line.append([right_lane])
        elif len(lr_s) > 0:
            right_lane = self.lane(ht,wt,avglr[0],avglr[1])
            lane_line.append([right_lane])

        #print("lane lines", lane_line)     #debug
        return lane_line


    def lane(self,ht,wt,slope,inter):
        '''
        Inputs:
            ht,wt : size of image frame
            slope : avg slope of lines
            inter : avg intercept of lines

        Function: helper function to compute lanes

        Outputs:
            lane : set of two (x,y) pairs that define
                   a lane line
        '''
        y1 = ht
        y2 = int(y1/2)
        x1 = max(-wt,min(2*wt,(y1-inter)/slope))
        x2 = max(-wt,min(2*wt,(y2-inter)/slope))
        lane = [int(x1),y1,int(x2),y2]
        return lane

    # FUNCTIONS TO SHOW DIFFERENT IMAGES
    def showRot(self, cam):
        # SHOW IMAGE AFTER ROTATION
        cv2.imshow('Rotated', self.rot)
        cam.show()

    def showHSV(self, cam):
        # SHOW IMAGE AFTER COLOR FITLERING
        cv2.imshow('Color', self.res)
        cam.show()

    def showEdge(self, cam):
        # SHOW IMAGE AFTER EDGE DETECTION
        cv2.imshow('Edges', self.edges)
        cam.show()

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
        cv2.imshow('Hough', new)
        cam.show()

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


if __name__ == "__main__":
    import cam
    camObj = cam.camera(0)
    LaneDetect = LaneKeep()

    while(1):
        camObj.run()
        fix = camObj.update()
        LaneDetect.run(fix)
        steer = LaneDetect.update()
        print("steer = ",steer)
        LaneDetect.showLanes(camObj)
