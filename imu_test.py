
<<<<<<< HEAD
'''
File: imu_test.py
Author: Michael Poskin
Date: 1/9/2020
Revision: 0.1
Description: Computes measurements of velocity and heading
             with IMU (myAHRS+).
'''


import sys
import time
import serial
import traceback
import math
import matplotlib.pyplot as plt
import numpy as np
=======

import sys
import time 
import serial 
import traceback
import math
>>>>>>> Add files via upload

def send_command(serial_port, cmd_msg):
    cmd_msg = '@' + cmd_msg.strip()
    crc = 0
    for c in cmd_msg:
        crc = crc^ord(c)
    message = cmd_msg + '*%02X'%crc + '\r\n'
    serial_port.write(message.encode())
<<<<<<< HEAD

    #
    # wait for response
    #
=======
    
    #
    # wait for response 
    #    
>>>>>>> Add files via upload
    if(cmd_msg != '@trig'):
        while(True):
            line = serial_port.readline().strip().decode()
            print(line[1])
            if(line[0] == '~'):
                return line


def parse_data_message_rpyimu(data_message):
    # $RPYIMU,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
<<<<<<< HEAD

    data_message = (data_message.split('*')[0]).strip() # discard crc field
    fields = [x.strip() for x in data_message.split(',')]

    if(fields[0] != '$RPYIMU'):
        return None

=======
    
    data_message = (data_message.split('*')[0]).strip() # discard crc field  
    fields = [x.strip() for x in data_message.split(',')]
    
    if(fields[0] != '$RPYIMU'):
        return None
    
>>>>>>> Add files via upload
    sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])
    return (int(sequence_number), roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)

# Function that removes gravity acceleration from accel values
def remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw):
    grav_x = math.sin(math.radians(pitch))
    grav_y = -math.sin(math.radians(roll))
    grav_z = -math.cos(math.radians(pitch))*math.cos(math.radians(roll))

    accel_x = accel_x - grav_x
    accel_y = accel_y - grav_y
    accel_z = accel_z - grav_z
<<<<<<< HEAD

=======
    
>>>>>>> Add files via upload
    return(accel_x, accel_y, accel_z)

#Function that finds max noise of imu (currently not used)
def set_noise(serial_port):
    last_loop = time.time()
    accel_noise_x = 0;
    accel_noise_y = 0;
    accel_noise_z = 0;
<<<<<<< HEAD

=======
    
>>>>>>> Add files via upload
    print("Calculating noise")
    while(time.time() - last_loop) < 5:
        send_command(serial_port, 'trig')
        items = parse_data_message_rpyimu(serial_port.readline().strip().decode())
        if(items):
            sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
<<<<<<< HEAD

            accel_x, accel_y, accel_z = remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw)

=======
    
            accel_x, accel_y, accel_z = remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw)
    
>>>>>>> Add files via upload
            if abs(accel_x) > accel_noise_x:
                accel_noise_x = abs(accel_x)
            if abs(accel_y) > accel_noise_y:
                accel_noise_y = abs(accel_y)
            if abs(accel_z) > accel_noise_z:
                accel_noise_z = abs(accel_z)
<<<<<<< HEAD

    print('anx %.4f, any %.4f, anz %.4f'%(accel_noise_x, accel_noise_y, accel_noise_z))

=======
                
    print('anx %.4f, any %.4f, anz %.4f'%(accel_noise_x, accel_noise_y, accel_noise_z))
    
>>>>>>> Add files via upload
    return(accel_noise_x, accel_noise_y, accel_noise_z)

#Main function, reads from imu and prints values
def read_rpyimu(serial_device):
    try:
        serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
    except serial.serialutil.SerialException:
        print('Can not open serial port(%s)'%(serial_device))
        traceback.print_exc()
<<<<<<< HEAD
        return

    # Get version
    rsp = send_command(serial_port, 'version')
    print(rsp)

    # Data transfer mode : ASCII, TRIGGER
    rsp = send_command(serial_port, 'mode,AT')
    print(rsp)

    # Select output message type (to my understanding only RPYIMU gives euler angles)
    rsp = send_command(serial_port, 'asc_out,RPYIMU')
    print(rsp)

    vel_x = []
    vel_y = []
    vel_z = []
    x_accel = []
    y_accel = []
    z_accel = []
    vel_xm = 0
    vel_ym = 0
    vel_zm = 0

    last_loop = time.time()
    last_write = last_loop
    loop_num = 0

    total_time = 0
    timet = []

    try:
    #Loops endlessly, retrieves values, then prints
        while True:
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

                vel_xm = vel_xm + accel_x*dt
                vel_ym = vel_ym + accel_y*dt
                vel_zm = vel_zm + accel_z*9.81*dt

                vel_x.append(vel_xm)
                vel_y.append(vel_ym)
                vel_z.append(vel_zm)
                x_accel.append(accel_x)
                y_accel.append(accel_y)

                last_loop = current_loop

                timet.append(total_time)
                print("\nloop_num: ",loop_num)
                print("loop time: ", dt)

                #if(current_loop - last_write)>.05:
                #print('vx %.4f, vy %.4f, vz %.4f'%(vel_xm, vel_ym, vel_zm))
                #last_write = current_loop

                loop_num += 1

    except KeyboardInterrupt:
        serial_port.close()
        #loop_time = np.linspace(0, loop_num, loop_num)
        fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
        ax1.plot(timet, vel_x, label='x-velocity')
        ax1.plot(timet, vel_y, label='y-velocity')
        ax1.legend()
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('velocity (m/s)')
        ax1.axis('on')
        ax2.plot(timet, x_accel, label='x-acceleration')
        ax2.plot(timet, y_accel, label='y-acceleration')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('acceleration (m/s^2)')
        ax2.legend()
        #plt.plot(loop_time, vel_z)
        plt.show()

if __name__ == '__main__':
    if(len(sys.argv) < 2):
        serial_device = '/dev/ttyACM0'
    else :
        serial_device = sys.argv[1]

    read_rpyimu(serial_device)
=======
        return 
    
    # Get version 
    rsp = send_command(serial_port, 'version')
    print(rsp) 
    
    # Data transfer mode : ASCII, TRIGGER 
    rsp = send_command(serial_port, 'mode,AT')
    print(rsp)  
    
    # Select output message type (to my understanding only RPYIMU gives euler angles) 
    rsp = send_command(serial_port, 'asc_out,RPYIMU')
    print(rsp)         
    
    vel_x = 0
    vel_y = 0
    vel_z = 0
    
    last_loop = time.time()
    last_write = last_loop
    
    #Loops endlessly, retrieves values, then prints
    while True:
        # send trigger command 
        send_command(serial_port, 'trig')
        
        # recieve data_message then parse it for data
        items = parse_data_message_rpyimu(serial_port.readline().strip().decode())
        
        if(items):
            sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
        
            accel_x, accel_y, accel_z = remove_gravity(accel_x, accel_y, accel_z, pitch, roll, yaw)
        
            current_loop = time.time()
            dt = current_loop - last_loop
        
            vel_x = vel_x + accel_x*9.81*dt
            vel_y = vel_y + accel_y*9.81*dt
            vel_z = vel_z + accel_z*9.81*dt
        
            last_loop = current_loop
            
            if(current_loop - last_write)>.05:
                print('vx %.4f, vy %.4f, vz %.4f'%(vel_x, vel_y, vel_z))
                last_write = current_loop

    serial_port.close()    

if __name__ == '__main__': 
    if(len(sys.argv) < 2):
        serial_device = '/dev/ttyACM0'
    else : 
        serial_device = sys.argv[1]
                        
    read_rpyimu(serial_device)

    



>>>>>>> Add files via upload
