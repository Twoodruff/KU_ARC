# test_fb.py

from moco import MotorController, Accel
from PID import PID
import time

#create PID controller
pid = PID(0.5,1,0)

#create car interfaces
car = MotorController()
#move = Accel(car)

current_spd = 0

#pid settings
setpoint_spd = 0.5
dt = 0.1
pid.setSP(setpoint_spd)

n = 0
while abs(pid.getErr()) > 0.01:
    print('Loop #: ', n)
    print('Speed: ', current_spd)
    new_spd = pid.update(current_spd, dt)
    
    #move.rampSpd(current_spd, new_spd)
    car.setDrive(new_spd)
    car.run()
    
    current_spd = new_spd
    print('Error: ', pid.getErr(), '\n')
    n= n+1
    time.sleep(dt)
    






