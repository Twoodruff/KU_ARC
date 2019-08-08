"""
File: PID.py
Author: Thomas Woodruff
Date: 5/3/19
Description: Basic PID controller.
"""

#----------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or research purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://bark-project.com
#
# Attribution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.ganzales@berkely.edu). The cloud services integration with ROS was developed
# by Kiet Lam (kiet.lam@berkeley.edu). The web-server app Dator
# was based on an open source project by Bruce Wooton
#----------------------------------------------------------------------------

class PID:
    def __init__(self, P=1.0, I=1.0, D=0.0, setpoint=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sp = setpoint
        self.e = 0

        self.e_int = 0
        self.int_max = 500
        self.int_min = -500

        self.current = 0

    def update(self, curr_val, dt):
        self.current = curr_val

        e_t = self.sp - self.current                                            #current error
        de_t = (e_t - self.e)/dt                                                #derivative error

        self.e_int = self.e_int + e_t*dt                                        #integral error
        if self.e_int > self.int_max:
            self.e_int = self.int_max
        elif self.e_int < self.int_min:
            self.e_int = self.int_min

        Pval = self.Kp*e_t
        Ival = self.Ki*self.e_int
        Dval = self.Kd*de_t

        PID = Pval + Ival + Dval
        self.e = e_t                                                            #update for derivative error

        return PID

    def setSP(self, setpoint):
        '''
        reset values for new setpoint value
        '''
        self.sp = setpoint
        self.e_int = 0
        self.e = setpoint - self.current

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.sp

    def getErr(self):
        return self.e


def f(x, u, dt):
    x_next = x + (3*x +u)*dt
    return x_next

if __name__ == "__main__":

    from numpy import zeros

    x = zeros(200)
    x[0] = 20
    pid = PID(3.7, 5, 0.5)
    dt = 0.1

    for i in range(199):
        u = pid.update(x[i], dt)
        x[i+1] = f(x[i], u, dt)
        print(i, x[i+1])
