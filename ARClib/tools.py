'''
File: tools.py
Author: Thomas Woodruff
Date: 10/25/19
Revision: 0.1
Description: module to store necessary
             helper functions.
'''

# IMPORTS
import math
import cv2
from pathlib import Path

class median:
    '''
    computes median filter over range of size
    '''
    import math
    def __init__(self, size=3):
        '''
        Inputs:
            size  : [3,inf), size of filter
        '''
        self.size = size
        self.medX = math.ceil(size/2)  # compute position of med value

        self.store = []
        self.medArray = []
        for i in range(size-1):
            self.store.append(0)
            self.medArray.append(0)
        self.medArray.append(0)

        self.loop = size + 1

    def run(self,value):
        '''
        Inputs:
            input : value to be filtered

        Function: computes median filter

        Outputs:
            medVal : median value
        '''
        self.store.append(value)
        i = self.loop - 2
        self.medArray[self.loop-i] = self.store[self.loop-2]
        self.medArray[self.loop-i-1] = self.store[self.loop-3]
        self.medArray[self.loop-i-2] = self.store[self.loop-4]

        sortArr = sorted(self.medArray)    # sort the input low to high

        self.loop += 1

        return sortArr[self.medX-1]      # compute median value



class memory:
    '''
    description of class
    '''
    def __init__(self, file):
        '''
        Inputs:
            input1 :
            input2 :
        '''
        from pathlib import Path
        from datetime import datetime
        import os
        #create directory for images
        now = datetime.now()
        dt_string = now.strftime("%b-%d-%y_%H-%M-%S")
        os.makedirs(dt_string)
        #specify file path
        self.filepath = file / dt_string


    def saveImage(self, input):
        #save input to File
        frame, heading, loop = input
        name = "drive_"+str(loop)+"_"+str(heading)+".png"
        filename = self.filepath / name
        cv2.imwrite(str(filename), frame)



class line:
    def __init__(self, slope, intercept):
        self.m = slope
        self.b = intercept


    def point(self, x=0, y=0):
        '''
        returns a point on the line
        '''
        if y == 0:
            return self.m * x + self.b
        elif x == 0:
            return (y - self.b)/self.m


    def lane(self,ht,wt,x=0,y=0):
        '''
        Inputs:
            ht,wt : size of image frame
            x,y   : average position of the lane

        Function: helper function to compute lanes

        Outputs:
            lane : set of two (x,y) pairs that define
                   a lane line
        '''
        if self.m > 0.75:
            y1 = ht
            y2 = 0 #self.top
            x1 = max(-wt,min(2*wt,(y1-self.b)/self.m))
            x2 = max(0,min(wt,(y2-self.b)/self.m))
            lane_pts = [int(x1),y1,int(x2),y2]
        else:
            x1 = 0
            x2 = wt
            y1 = min(2*ht,max(-ht,self.m*x1 + self.b))
            y2 = min(2*ht,max(-ht,self.m*x2 + self.b))
            lane_pts = [x1,int(y1),x2,int(y2)]

        return lane_pts


    def intersection(self, ht, wt, m2, b2):
        x = (b2-self.b)/(self.m-m2)
        if x > wt or x < 0:
            return -1, -1

        y1 = self.m*x + self.b
        y2 = m2*x + b2
        # check they're equal (or close enough)
        # and within the frame
        if math.fabs(y1-y2) <= 1e-5*max(math.fabs(y1), math.fabs(y2)):
            return -1, -1
        elif y1 > ht or y1 < 0:
            return -1, -1

        return x, y1
