#! /usr/bin/env python3

'''
Filename: test_fb.py
Author: Thomas Woodruff
Date: 10/17/19
Description: simple feedback control to
             help tune PID
'''

from ARClib.moco import MotorController, Accel
from ARClib.PID import PID
import time
from numpy import zeros
import matplotlib.pyplot as plt

#create PID controller
pid = PID(0.5,1,0)

#create car interfaces
car = MotorController()

#need to add threaded car control 

#pid settings
setpoint_spd = 0.5
dt = 0.1
pid.setSP(setpoint_spd)

#initialize
current_spd = 0
x = zeros(100)
n = 0

while abs(pid.getErr()) > 0.05:
    print('Loop #: ', n)
    print('Speed: ', current_spd)
    x[n]=current_spd

    new_spd = pid.update(current_spd, dt)
    car.setSteer(new_spd)
    car.run()
    car.update()

    print('Error: ', pid.getErr(), '\n')
    current_spd = new_spd
    n = n+1
    time.sleep(dt)

car.shutdown()
plt.plot(x)
plt.show()
