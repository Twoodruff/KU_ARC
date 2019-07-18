#import actuators
from moco import MotorController, _Ramp
import time

car = MotorController()

##loop = 0
##while loop < 5:
##    print("Loop: ", loop)
##    car.run()
##    loop += 1
##    #car.setSteer(4*loop)
##    car.setDrive(0.25*loop)
##    time.sleep(1)
##    
##print("Exit Loop")
##car.shutdown()
##car.setSteer(0)
##car.setDrive(0)
##print("pickle")

#print(car.setDrive(0.1))
#print(car.setSteer(5))

##for i in range(30):
##    #car.Drive(90+i)
##    car.Steer(90 - i)
##    print("Steering: " + str(int(90-i)))
##    time.sleep(0.5)
##
##car.startup(mchannel = 3, schannel = 0)

rp = _Ramp(car)
#car.run()
#rp.rampSpd(0, 1, 1)
#print("Stop")
#rp.stop()
#print("Backwards")
#rp.rampSpd(0, -1, -1)
#rp.stop()
print("Steer")
rp.rampDir(0, 45, 1)
rp.stop()