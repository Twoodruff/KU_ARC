'''
File: models.py
Author: Thomas Woodruff
Date: 03/05/2020
Revision: 0.1
Description: dynamic models of the system
'''

# IMPORTS
from numpy import cos, sin, tan

class KinBike:
    '''
    basic kinematic bike model
    states are position (x,y), long. velocity (v), and heading angle (theta)
    inputs are steering angle (steer) and accleration (acc)
    assumes no tire-slip and flat ground
    '''
    def __init__(self):
        # INITIALIZE VARS
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.dt    = None
        self.L     = None
        self.input = [0.0, 0.0]

        self.running = True

    def run(self):
        '''
        Function: computes states at next time step
        '''

        if self.running:
            (x, y, theta, v) = self.state
            (steer, acc)     = self.input
            (dt, L)          = self.dt, self.L

            x_next     = x      + (v*cos(theta))*dt
            y_next     = y      + (v*sin(theta))*dt
            theta_next = theta  + (v/L*tan(steer))*dt
            v_next     = v      + acc*dt

            self.state = [x_next, y_next, theta_next, v_next]

    def update(self):
        '''
        Outputs:
            self.state : list of state vars
        '''
        return self.state

    def shutdown(self):
        #Clean up anything when done
        self.running = False

    def setInput(self, steer, acc):
        '''
        Inputs:
            steer : steering angle
            acc   : acceleration

        Function: set current inputs
        '''
        self.input = [steer, acc]
