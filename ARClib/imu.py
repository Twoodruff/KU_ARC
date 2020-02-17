'''
File: imu.py
Author: Thomas Woodruff
Date: 1/9/2020
Revision: 0.1
Description: Computes measurements of velocity and heading
             with IMU (myAHRS+).
'''

# IMPORTS
import time
import serial
import traceback
import math
import numpy as np
import matplotlib.pyplot as plt


class IMU:
    '''
    description of class
    '''
    def __init__(self, serial_device, output='rpy'):  #add in default for RPI
        '''
        Inputs:
            serial_device : the machine IMU is connected to
            output : 'rpy' - roll, pitch, yaw
                     'quat' - quaternion (x,y,z,w)
        '''
        # INITIALIZE PORT DEVICE
        try:
            self.serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
        except serial.serialutil.SerialException:
            print('Can not open serial port {}'.format(serial_device))
            traceback.print_exc()

        # SET MODE AND OUTPUT TYPE
        self.send_command(self.serial_port, 'mode,AT')

        if output == 'rpy':
            self.send_command(self.serial_port, 'asc_out,RPYIMU')
        elif output == 'quat':
            self.send_command(self.serial_port, 'asc_out,QUATIMU')
        else:
            raise Exception('Invalid output type. Please use \'rpy\' or \'quat\'')

        # INITIALIZE VARS
        self.vel_x = 0
        self.vel_y = 0
        self.heading = 0
        self.last = time.time()
        self.running = True

        self.initialize()


    def run(self):
        '''
        Inputs: (optional)
            input1 : range/description
            input2 : range/description

        Function: description
        '''

        if self.running:

            # SEND TRIGGER
            self.send_command(self.serial_port, 'trig')

            # RECIEVE MESSAGE AND PARSE
            data_message = self.serial_port.readline().strip().decode()
            data_message = (data_message.split('*')[0]).strip()  # discard crc field
            fields = [x.strip() for x in data_message.split(',')]

            # GET VARS
            if(fields[0] == '$RPYIMU'):
                sequence_number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])

                self.current = time.time()
                self.dt = self.current - self.last

                # COMPUTE REQ. VARS
                self._accel_x, self._accel_y, self._accel_z = self.remove_gravity(accel_x, accel_y, accel_z,
                                                                                  pitch, roll, yaw)

                self._yaw = yaw
                self._gyro_x = gyro_x
                self._gyro_y = gyro_y
                self._gyro_z = gyro_z

            elif(fields[0] == '$QUATIMU'):  #'$QUATIMU'
                sequence_number, x, y, z, w, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = (float(x) for x in fields[1:])

                self.current = time.time()
                dt = self.current - self.last

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
        self.vel_x = self.vel_x + (self._accel_x - self.calib_accel[0])*9.81*self.dt
        self.vel_y = self.vel_y + (self._accel_y - self.calib_accel[1])*9.81*self.dt

        self.heading = self._yaw - self.heading_ref #self.heading + (self._yaw - self.heading) - self.heading_ref

        return self.vel_x, self.vel_y, self.heading, self._gyro_x, self._gyro_y, self._gyro_z


    def shutdown(self):
        # Clean up anything when done
        self.running = False
        self.serial_port.close()


    def initialize(self):
        '''
        Function: calibrate to account for drift
                  average accleration over 3 sec
        '''
        acc_x = []
        acc_y = []
        acc_z = []
        init_time = 3
        start = time.time()
        while time.time()<(start+init_time):
            self.run()
            acc_x.append(self._accel_x)
            acc_y.append(self._accel_y)
            acc_z.append(self._accel_z)
        self.calib_accel = [np.mean(acc_x), np.mean(acc_y), np.mean(acc_z)]

        self.run()
        self.heading_ref = self._yaw

    def send_command(self, serial_port, cmd_msg):
        '''
        Inputs:
            serial_port : serial device to communicate with
            cmd_msg : data to send to serial_port

        Function: sends data over serial comms

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
    #imu = IMU('COM3')
    imu = IMU('/dev/ttyACM0')

    velx = []
    vely = []
    head = []
    gyro = []
    seconds = 3
    tim = np.linspace(0, seconds, num=10*seconds)
    for i in (tim):
        imu.run()
        vel_x, vel_y, heading, gyrox, gyroy, gyroz = imu.update()
        velx.append(vel_x)
        vely.append(vel_y)
        head.append(heading)
        gyro.append([gyrox, gyroy, gyroz])

        #print("x-velocity: {}\ny-velocity: {}\nheading: {}\n".format(vel_x, vel_y, heading))
        #print("gyro x: {}\ngyro y: {}\ngyro z: {}\n".format(gyrox, gyroy, gyroz))
        time.sleep(0.1)

    gyro = np.array(gyro)

    #plotting results
    fig1, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex='col')
    ax1.plot(tim, velx, label='x-velocity')
    ax1.plot(tim, vely, label='y-velocity')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('velocity (m/s)')
    ax1.legend()
    ax1.axis('on')

    #ax2.plot(tim, x_accel, label='x-acceleration')
    #ax2.plot(tim, y_accel, label='y-acceleration')
    #ax2.set_ylabel('acceleration (m/s^2)')
    #ax2.legend()

    fig2, (ax3, ax4) = plt.subplots(nrows=2, ncols=1, sharex='col')
    ax3.plot(tim, gyro[:,0], label='theta_x dot')
    ax3.plot(tim, gyro[:,1], label='theta_y dot')
    ax3.plot(tim, gyro[:,2], label='theta_z dot')
    ax3.set_ylabel('angular velocity (deg/s)')
    ax3.legend()

    ax4.plot(tim, head)
    ax4.set_xlabel('time (s)')
    ax4.set_ylabel('angle (deg)')
    ax4.legend()

    plt.show()
