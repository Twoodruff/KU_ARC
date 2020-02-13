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


class MultiLane():
    def __init__(self):
        pickle = 1

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

        # CLASSIFY LINES BASED ON SLOPE
        for line in lines:
            for x1,y1,x2,y2 in line:
                #find slope using points and categorize left from right
                fit = poly.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[1]
                intercept = fit[0]
                angle = math.degrees(math.atan((y2-y1)/(x2-x1)))
                # print("angle: {}\nslope: {}\n".format(angle,slope))

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

        # FIND AVG OF EACH TYPE OF LINE
        avg1 = np.nanmean(ang1, axis = 0)
        # print("\nline1: ", avg1)

        avg2 = np.nanmean(ang2, axis = 0)
        # print("line2: ", avg2)

        avg3 = np.nanmean(ang3, axis = 0)
        # print("line3: ", avg3)

        avg4 = np.nanmean(ang4, axis = 0)
        # print("line4: {}".format(avg4))

        # CREATE LINE OBJECTS BASED ON AVG's
        if len(ang1) > 0:
            line1 = tools.line(avg1[0], avg1[1], avg1[2], avg1[3])
            #lane1 = line1.lane(self.height, self.width, avg1[2], avg1[3])
            lane_line.append(line1)

        if len(ang2) > 0:
            line2 = tools.line(avg2[0], avg2[1], avg2[2], avg2[3])
            #lane2 = line2.lane(self.height, self.width, avg2[2], avg2[3])
            lane_line.append(line2)

        if len(ang3) > 0:
            line3 = tools.line(avg3[0], avg3[1], avg3[2], avg3[3])
            #lane3 = line3.lane(self.height, self.width, avg3[2], avg3[3])
            lane_line.append(line3)

        if len(ang4) > 0:
            line4 = tools.line(avg4[0], avg4[1], avg4[2], avg4[3])
            #lane4 = line4.lane(self.height, self.width, avg4[2], avg4[3])
            lane_line.append(line4)

        # self.int_pts = []
        # for i in range(len(lane_line)):
        #     try:
        #         x, y = lane_line[i].intersection(self.height, self.width, lane_line[i+1].m, lane_line[i+1].b)
        #         self.int_pts.append((x,y))
        #         # if self.int_pts[i][0] > 0:
        #         #     print("intersection point: {},{}".format(self.int_pts[i][0], self.int_pts[i][1]))
        #
        #     except IndexError:
        #         break

        return lane_line
