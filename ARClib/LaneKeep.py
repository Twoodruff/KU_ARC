"""
File: LaneDetect.py
Author: Thomas Woodruff
Date: 1/7/2020
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
        self.prevxoff = 0
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
            self.height, self.width, _ = frame.shape

            # ROTATE FRAME
            self.rot = cam.rotate(frame,180)

            # CONVERTING COLOR SPACE
            hsv = cv2.cvtColor(self.rot, cv2.COLOR_BGR2HSV)

            # COLOR DETECTION
            hue_center = self.detect_color[0][0][0]
            lower = np.array([hue_center-25,30,70]) #bounds in hsv color space
            upper = np.array([hue_center+25,255,255])
            mask = cv2.inRange(hsv, lower, upper)
            self.res = cv2.bitwise_and(hsv, hsv, mask = mask)

            # CROPPING IMAGE
            crop = np.zeros(self.res.shape, dtype = 'uint8')
            n = 0.5   #determines how much of the frame is cropped in the vertical direction, from the top
            self.top = int(self.height*n)
            cv2.rectangle(crop, (0, self.top), (self.width, self.height), (255, 255, 255), -1)
            crop_im = cv2.bitwise_and(src1 = self.res, src2 = crop)

            # EDGE DETECTION
            self.edges = cv2.Canny(crop_im ,95,135)    #threshold parameters may need tuning for robustness

            # LINE DETECTION
            rho_res = 1
            theta_res = np.pi/180
            threshold = 50
            minLineLength = 20
            maxLineGap = 10
            self.lines = cv2.HoughLinesP(self.edges,rho_res,theta_res,threshold,minLineLength,maxLineGap)

            # LANE DETECTION
            self.lanes = self.avg_lines(self.rot, self.lines)

            # COMPUTE HEADING ANGLE
            # if len(self.lanes)>1:
            #     #if both lanes are detected, find the middle
            #     _, _, x_left, _ = self.lanes[0][0]  #first row, first column
            #     _, _, x_right, _ = self.lanes[1][0] #second row, first column
            #     x_off = (x_left + x_right)/2 - int(self.width/2)  #offset from frame center
            # elif len(self.lanes)>0:
            #     #if only one lane is detected
            #     x1, _, x2, _ = self.lanes[0][0]
            #     x_off = x2-x1
            # else:
            #     #if no lanes are detected, use previous heading
            #     x_off = self.prevxoff
            #
            # self.prevxoff = x_off
            # y_off = int(self.height/2)
            # self.heading_rad = math.atan(x_off/y_off)            #compute heading angle (rad)
            # self.heading_deg = int(self.heading_rad * 180 / math.pi)  #convert to deg


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
        angle1 = None
        angle2 = None
        angle3 = None
        angle4 = None
        ang1 = []
        ang2 = []
        ang3 = []
        ang4 = []

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
                angle = math.degrees(math.atan((y2-y1)/(x2-x1)))
                print("angle: {}\nslope: {}\n".format(angle,slope))

                # classification by slope of line (using the angle)
                if angle1 is None:
                    angle1 = angle
                    ang1 = [(slope,intercept, x1, y1)]
                elif (math.fabs(angle-angle1)>10):
                    if angle2 is None:
                        angle2 = angle
                        ang2 = [(slope,intercept, x1, y1)]
                    elif (math.fabs(angle-angle2)>10):
                        if angle3 is None:
                            angle3 = angle
                            ang3 = [(slope,intercept, x1, y1)]
                        elif (math.fabs(angle-angle3)>10):
                            if angle4 is None:
                                angle4 = angle
                                ang4 = [(slope,intercept, x1, y1)]
                            else:
                                ang4.append((slope,intercept, x1, y1))
                        else:
                            ang3.append((slope,intercept, x1, y1))
                    else:
                        ang2.append((slope,intercept, x1, y1))
                else:
                    ang1.append((slope,intercept, x1, y1))

        # FIND AVG OF EACH TYPE OF LINE cd Documents/KU_ARC python Documents/KU_ARC/test_LaneKeep_sim.py
        #with warnings.catch_warnings():
        line1 = np.nanmean(ang1, axis = 0)
        print("line1: ", line1)

        line2 = np.nanmean(ang2, axis = 0)
        print("line2: ", line2)

        line3 = np.nanmean(ang3, axis = 0)
        print("line3: ", line3)

        line4 = np.nanmean(ang4, axis = 0)
        print("line4: {}\n".format(line4))

        # CREATE LANES BASED ON AVG's
        if len(ang1) > 0:
            lane1 = self.lane(self.height, self.width, line1[0], line1[1], line1[2], line1[3])
            lane_line.append([lane1])

        if len(ang2) > 0:
            lane2 = self.lane(self.height, self.width, line2[0], line2[1], line2[2], line2[3])
            lane_line.append([lane2])

        if len(ang3) > 0:
            lane3 = self.lane(self.height, self.width, line3[0], line3[1], line3[2], line3[3])
            lane_line.append([lane3])

        if len(ang4) > 0:
            lane4 = self.lane(self.height, self.width, line4[0], line4[1], line4[2], line4[3])
            lane_line.append([lane4])

        return lane_line


    def lane(self,ht,wt,slope,inter,x,y):
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
        if slope > 0.75:
            y1 = ht
            y2 = 0 #self.top
            x1 = max(-wt,min(2*wt,(y1-inter)/slope))
            x2 = max(0,min(wt,(y2-inter)/slope))
            lane = [int(x1),y1,int(x2),y2]
        else:
            x1 = 0
            x2 = wt
            y1 = min(2*ht,max(-ht,slope*x1 + inter))
            y2 = min(2*ht,max(-ht,slope*x2 + inter))
            lane = [x1,int(y1),x2,int(y2)]

        return lane

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
        cv2.imshow('Hough', new)
        cam.show()
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
        # cv2.imshow('Heading', new)                                            #commented only for debug
        #cam.show()
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
