'''
File: imu.py
<<<<<<< HEAD
Author: Thomas Woodruff
Date: 1/9/2020
=======
Author: First Last
Date: 1/8/2020
>>>>>>> more plotting, created imu.py class
Revision: 0.1
Description: Computes measurements of velocity and heading
             with IMU (myAHRS+).
'''

# IMPORTS
import sys
import time
import serial
import traceback
import math
<<<<<<< HEAD
import numpy as np
import matplotlib.pyplot as plt

=======
import matplotlib.pyplot as plt
import numpy as np
>>>>>>> more plotting, created imu.py class

class IMU:
    '''
    description of class
    '''
<<<<<<< HEAD
    def __init__(self,serial_device,output = 'rpy'): #add in default for RPI
        '''
        Inputs:
            serial_device : the machine IMU is connected to
            output : 'rpy' - roll, pitch, yaw
                     'quat' - quaternion (x,y,z,w)
        '''
        # INITIALIZE PORT DEVICE
        try:
            self.serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
=======
    def __init__(self,inputs):
        '''
        Inputs:
            input1 :
            input2 :
        '''
        # INITIALIZE VARS
        try:
            serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
>>>>>>> more plotting, created imu.py class
        except serial.serialutil.SerialException:
            print('Can not open serial port(%s)'%(serial_device))
            traceback.print_exc()

<<<<<<< HEAD
        # SET MODE AND OUTPUT TYPE
        self.send_command(self.serial_port, 'mode,AT')

        if output == 'rpy':
            self.send_command(self.serial_port, 'asc_out,RPYIMU')
        elif output == 'quat':
            self.send_command(self.serial_port, 'asc_out,QUATIMU')
        else:
            raise Exception('Invalid output type. Please use rpy or quat')

        # INITIALIZE VARS
        self.vel_x = 0
        self.vel_y = 0
        self.heading = 0
        self.last = time.time()
        self.running = True


    def run(self):
=======
        vel_x = 0
        vel_y = 0
        heading = yaw
        self.running = True

    def run(self, inputs):
>>>>>>> more plotting, created imu.py class
        '''
        Inputs: (optional)
            input1 : range/description
            input2 : range/description

        Function: description
        '''
<<<<<<< HEAD

        if self.running:

            # SEND TRIGGER
            self.send_command(self.serial_port, 'trig')

            # RECIEVE MESSAGE AND PARSE
            data_message = self.serial_port.readline().strip().decode()
            data_message = (data_message.split('*')[0]).strip() # discard crc field
            fields = [x.strip() for x in data_message.split(',')]

            # GET VARS
            if(fields[0] == '$RPYIMU'):
                sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])

                self.current = time.time()
                dt = self.current - self.last

                # COMPUTE REQ. VARS
                accel_x, accel_y, accel_z = self.remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw)
                self.vel_x = self.vel_x + accel_x*9.81*dt
                self.vel_y = self.vel_y + accel_y*9.81*dt
                #vel_z = vel_z + accel_z*9.81*dt
                self.heading = yaw

            else: # '$QUATIMU'
                sequence_number, x, y, z, w, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])

                current = time.time()
                dt = current - last

                # COMPUTE REQ. VARS
                # accel_x, accel_y, accel_z = self.remove_gravity(accel_x, accel_y, accel_z, quaternion vars)
                self.vel_x = self.vel_x + accel_x*9.81*dt
                self.vel_y = self.vel_y + accel_y*9.81*dt
                #vel_z = vel_z + accel_z*9.81*dt
                # self.heading = quaternion eq

            self.last = self.current


    def update(self):
        '''
        Function: returns x,y-velocity and heading angle

        Outputs:
            vel_ : m/s
            heading : degrees
        '''
        return self.vel_x, self.vel_y, self.heading

=======
        #This method will be accessed by the Thread function
        #Loops continually

        while self.running:
            last_loop = time.time()
            last_write = last_loop

            try:
                # send trigger command
                send_command(serial_port, 'trig')

                # recieve data_message then parse it for data
                items = parse_data_message_rpyimu(serial_port.readline().strip().decode())

                if(items):
                    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items

                    accel_x, accel_y, accel_z = remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw)

                    current_loop = time.time()
                    dt = current_loop - last_loop
                    total_time = total_time + dt

                    vel_x = vel_x + accel_x*dt
                    vel_y = vel_y + accel_y*dt
                    vel_z = vel_z + accel_z*9.81*dt

                    last_loop = current_loop

    def update(self,inputs):
        '''
        Inputs: (optional)
            input1 : range/description
            input2 : range/description

        Function: description

        Outputs:
            output1 : range/description
            output2 : range/description
        '''
        #This method will only be accessed once per drive cycle
        #Return values should go here
        return self.vars
>>>>>>> more plotting, created imu.py class

    def shutdown(self):
        #Clean up anything when done
        self.running = False
<<<<<<< HEAD
        self.serial_port.close()

=======
>>>>>>> more plotting, created imu.py class

    def send_command(self, serial_port, cmd_msg):
        '''
        Inputs:
            serial_port : range/description
            cmd_msg : range/description

        Function: description

        Outputs:
            line : range/description
        '''
        cmd_msg = '@' + cmd_msg.strip()
        crc = 0
        for c in cmd_msg:
            crc = crc^ord(c)
        message = cmd_msg + '*%02X'%crc + '\r\n'
        serial_port.write(message.encode())

        # wait for response
        if(cmd_msg != '@trig'):
            while(True):
                line = serial_port.readline().strip().decode()
                print(line[1])
                if(line[0] == '~'):
                    return line

<<<<<<< HEAD

    def remove_gravity(self, accel_x, accel_y, accel_z, pitch, roll, yaw):
        '''
        Inputs: (optional)
            accel : acceleration in x,y,z
            rpy : roll, pitch, yaw

        Function: removes gravity acceleration from accel values

        Outputs:
            accel : new accleration in x,y,z
        '''
        grav_x = math.sin(math.radians(pitch))
        grav_y = -math.sin(math.radians(roll))
        grav_z = -math.cos(math.radians(pitch))*math.cos(math.radians(roll))

        accel_x = accel_x - grav_x
        accel_y = accel_y - grav_y
        accel_z = accel_z - grav_z

        return(accel_x, accel_y, accel_z)


if __name__ == '__main__':
    imu = IMU('COM3')
    i = 0
    while i < 200:
        imu.run()
        vel_x, vel_y, heading = imu.update()
        print("x-velocity: {}\ny-velocity: {}\nheading: {}\n".format(vel_x, vel_y, heading))
        time.sleep(0.1)
        i += 1
=======
def parse_data_message_rpyimu(self, data_message):
    '''
    Inputs: (optional)
        data_message : range/description

    Function: description

    Outputs:
        RPY : range/description
        Accel : range/description
        Mag: range/description
        Temp: range/description
    '''
    # $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F

    data_message = (data_message.split('*')[0]).strip() # discard crc field
    fields = [x.strip() for x in data_message.split(',')]

    if(fields[0] != '$RPYIMU'):
        return None

    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])
    return (int(sequence_number), roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)

def remove_gravity(self, accel_x, accel_y, accel_z, pitch, roll, yaw):
    '''
    Inputs: (optional)
        accel : acceleration in x,y,z
        rpy : roll, pitch, yaw

    Function: removes gravity acceleration from accel values

    Outputs:
        accel : new accleration in x,y,z
    '''
    grav_x = math.sin(math.radians(pitch))
    grav_y = -math.sin(math.radians(roll))
    grav_z = -math.cos(math.radians(pitch))*math.cos(math.radians(roll))

    accel_x = accel_x - grav_x
    accel_y = accel_y - grav_y
    accel_z = accel_z - grav_z

    return(accel_x, accel_y, accel_z)
>>>>>>> more plotting, created imu.py class
