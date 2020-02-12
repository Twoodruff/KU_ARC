'''
File: lane_control.py
Author: Thomas Woodruff
Date: 02/12/20
Revision: 0.1
Description: Helper classes that performs lane detection and control.
'''

# IMPORTS
import numpy as np
import cv2
import math

class TwoLines:
    '''
    description of class
    '''
    def __init__(self):
        '''
        Inputs:
            input1 : height, width
        '''
        # INITIALIZE VARS
        self.prevxoff = 0

    def track(self, frame, lines):
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

        self.height, self.width, _ = frame.shape

        # INITIALIZE ARRAYS
        lane_line = []
        ll_s = []
        lr_s = []
        rr_s = []
        rl_s = []

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
                    if x2 < int(self.width*(2/3)):
                        #neg slope on left side (straight/right turn)
                        ll_s.append((slope,intercept))
                    else:
                        #neg slope on right side (right turn)
                        lr_s.append((slope,intercept))
                elif slope > 0:
                    if x2 >  int(self.width*(1/3)):
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
            left_lane = self.lane(self.height,self.width,avgll[0],avgll[1])
            lane_line.append([left_lane])
        elif len(rl_s) > 0:
            left_lane = self.lane(self.height,self.width,avgrl[0],avgrl[1])
            lane_line.append([left_lane])

        #right lane
        if len(rr_s) > 0:
            right_lane = self.lane(self.height,self.width,avgrr[0],avgrr[1])
            lane_line.append([right_lane])
        elif len(lr_s) > 0:
            right_lane = self.lane(self.height,self.width,avglr[0],avglr[1])
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
        y2 = int(self.height*0.5)
        x1 = max(-wt,min(2*wt,(y1-inter)/slope))
        x2 = max(-wt,min(2*wt,(y2-inter)/slope))
        lane = [int(x1),y1,int(x2),y2]
        return lane


    def control(self, lanes):
        # COMPUTE HEADING ANGLE
        if len(lanes)>1:
            #if both lanes are detected, find the middle
            _, _, x_left, _ = lanes[0][0]  #first row, first column
            _, _, x_right, _ = lanes[1][0] #second row, first column
            x_off = (x_left + x_right)/2 - int(self.width/2)  #offset from frame center
        elif len(lanes)>0:
            #if only one lane is detected
            x1, _, x2, _ = lanes[0][0]
            x_off = x2-x1
        else:
            #if no lanes are detected, use previous heading
            x_off = self.prevxoff

        self.prevxoff = x_off
        y_off = int(self.height/2)
        heading_rad = math.atan(x_off/y_off)            #compute heading angle (rad)
        heading_deg = int(heading_rad * 180 / math.pi)  #convert to deg
        return heading_deg
