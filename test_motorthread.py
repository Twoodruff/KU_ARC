#! /usr/bin/env python3

from ARClib.moco import MotorController
from threading import Thread
import time

car = MotorController()

car_thread = Thread(target = car.run())
car_thread.start()

print('Driving')
car.setDrive(0.6)
#car.run()
print('waiting')
time.sleep(3)
#for i in range(1, 9, 1):
#    car.setDrive(i/10)
#    time.sleep(3)

car.shutdown()
car_thread.join()