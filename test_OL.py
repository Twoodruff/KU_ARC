from keyboard_OL import KC
import time
from moco import MotorController, Accel
import sys

curr_spd = 0.0
car = MotorController()
control = KC(car)
car.setDrive(curr_spd)
car.run()


while 1:
    
    #comm = input("Command: ")
    comm = sys.stdin.readline()
    #time.sleep(0.5)
    key = ord(comm[0])
    control.keyControl(key)
    

cap.release()
cv2.destroyAllWindows()
