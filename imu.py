'''
File: imu.py
Author: First Last
Date: 1/8/2020
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
import matplotlib.pyplot as plt
import numpy as np

class IMU:
    '''
    description of class
    '''
    def __init__(self,inputs):
        '''
        Inputs:
            input1 :
            input2 :
        '''
        # INITIALIZE VARS
        try:
            serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
        except serial.serialutil.SerialException:
            print('Can not open serial port(%s)'%(serial_device))
            traceback.print_exc()

        vel_x = 0
        vel_y = 0
        heading = yaw
        self.running = True

    def run(self, inputs):
        '''
        Inputs: (optional)
            input1 : range/description
            input2 : range/description

        Function: description
        '''
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

    def shutdown(self):
        #Clean up anything when done
        self.running = False

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
