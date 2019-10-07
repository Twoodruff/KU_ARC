"""
File: LaneDetect.py
Author: Thomas Woodruff
Date: 9/11/19
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


class LaneKeep():
    def __init__(self, cap, detect = [255,0,0], color = [0,255,0]):
        det_color = np.uint8([[detect]])
        self.detect_color = cv2.cvtColor(det_color, cv2.COLOR_BGR2HSV)
        self.line_color = color
        self.cap = cap


    def rotate(self, image, angle):
        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        (cX, cY) = (w / 2, h / 2)

        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise)
        M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

        # perform the actual rotation and return the image
        return cv2.warpAffine(image, M, (w, h))


    def avg_lines(self, frame, lines):
        import numpy.polynomial.polynomial as poly
        lane_line = []
        ht, wt, _ = frame.shape

        ll_s = []
        lr_s = []
        rr_s = []
        rl_s = []

        if lines is None:
            print('No lines to detect')
            return lane_line

        #iterate over all lines
        for line in lines:
            for x1,y1,x2,y2 in line:
                #find slope using points and categorize left from right
                fit = poly.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[1]
                intercept = fit[0]
                #classification of lines
                if slope < 0:
                    if x2 < int(wt*(2/3)):
                        ll_s.append((slope,intercept))                          #neg slope on left side (straight/right)
                    else:
                        lr_s.append((slope,intercept))                          #neg slope on right side (turn right)
                elif slope > 0:
                    if x2 >  int(wt*(1/3)):
                        rr_s.append((slope,intercept))                          #pos slope on right side (straight/left)
                    else:
                        rl_s.append((slope,intercept))                          #pos slope on left side (turn left)
                else:
                    pass

        # average lines (slope, int)

        avglr = np.mean(lr_s, axis = 0)
        #print("\nRight", avglr)

        avgll = np.mean(ll_s, axis = 0)
        #print("Strt/Right", avgll)
                              #debug
        avgrr = np.mean(rr_s, axis = 0)
        #print("Strt/Left", avgrr)

        avgrl = np.mean(rl_s, axis = 0)
        #print("Left", avgrl)

        #create lane lines based on categorization

        #left lane
        if len(ll_s) > 0:
            slopel = avgll[0]
            intl = avgll[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intl)/slopel))
            x2 = max(-wt,min(2*wt,(y2-intl)/slopel))
            left_lane = [int(x1),y1,int(x2),y2]
            lane_line.append([left_lane])
        elif len(rl_s) > 0:
            slopel = avgrl[0]
            intl = avgrl[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intl)/slopel))
            x2 = max(-wt,min(2*wt,(y2-intl)/slopel))
            left_lane = [int(x1),y1,int(x2),y2]
            lane_line.append([left_lane])

        #right lane
        if len(rr_s) > 0:
            sloper = avgrr[0]
            intr = avgrr[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intr)/sloper))
            x2 = max(-wt,min(2*wt,(y2-intr)/sloper))
            right_lane = [int(x1),y1,int(x2),y2]
            lane_line.append([right_lane])
        elif len(lr_s) > 0:
            sloper = avglr[0]
            intr = avglr[1]
            y1 = ht
            y2 = int(y1/2)
            x1 = max(-wt,min(2*wt,(y1-intr)/sloper))
            x2 = max(-wt,min(2*wt,(y2-intr)/sloper))
            right_lane = [int(x1),y1,int(x2),y2]
            lane_line.append([right_lane])

        #print("lane lines", lane_line)              #debug
        return lane_line


    def run(self, frame):
        # READ IN FRAME
        self.height, self.width, _ = frame.shape

        # ROTATE FRAME
        self.rot = self.rotate(frame,180)

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
        top = int(self.height/n)
        cv2.rectangle(crop, (0, top), (self.width, self.height), (255, 255, 255), -1)
        self.crop_im = cv2.bitwise_and(src1 = self.res, src2 = crop)

        # EDGE DETECTION
        self.edges = cv2.Canny(self.crop_im ,100,200) #threshold parameters may need tuning for robustness

        # LINE DETECTION
        minLineLength = 100
        maxLineGap = 20
        self.lines = cv2.HoughLinesP(self.edges,1,np.pi/180,50,minLineLength,maxLineGap)

        # LANE DETECTION
        self.lanes = self.avg_lines(self.rot, self.lines)

        #return self.lanes

        # COMPUTE HEADING ANGLE
        if len(self.lanes)>1:
            _, _, x_left, _ = self.lanes[0][0]  #first row, first column
            _, _, x_right, _ = self.lanes[1][0] #second row, first column

            x_off = (x_left + x_right)/2 - int(self.width/2)  #offset from frame center
        else:
            x_off = 0

        y_off = int(self.height/2)
        heading_rad = math.atan(x_off/y_off)            #compute heading angle (rad)
        heading_deg = int(heading_rad * 180 / math.pi)  #convert to deg

        return heading_deg

    # FUNCTIONS TO SHOW DIFFERENT IMAGES
    def show(self):
        k = cv2.waitKey(1)
        if k == ord('q') & 0xFF:
            self.shutdown()

    def showRot(self):
        cv2.imshow('Rotated', self.rot)
        self.show()

    def showHSV(self):
        cv2.imshow('Color', self.res)
        self.show()

    def showEdge(self):
        cv2.imshow('Edges', self.edges)
        self.show()

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
        self.show()

    def showLanes(self):
        new = np.zeros_like(self.rot)
        if self.lanes is not None:
            for line in self.lanes:
                for x1,y1,x2,y2 in line:
                    cv2.line(new,(x1,y1),(x2,y2),self.line_color,10)
        else:
            pass

        new = cv2.addWeighted(self.rot, 1, new, 1, 1)
        cv2.imshow('Lanes', new)
        self.show()


    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    LaneDetect = LaneKeep(cap)

    while(1):
        _, frame = cap.read()
        steer = LaneDetect.run(frame)
        print('\n',steer)
        LaneDetect.showLanes()
