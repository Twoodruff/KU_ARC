'''
File: logger.py
Author: Thomas Woodruff
Date: 02/27/2020
Revision: 0.1
Description: module responsible for saving data
'''

# IMPORTS
import os
from pathlib import Path
from datetime import datetime

class DataLogger:
    '''
    save generic data to text file
    '''
    def __init__(self, filename, filepath='/home/pi/Documents/KU_ARC/data'):
        '''
        Inputs:
            filepath : path to file
            filename : name of save file
        '''
        # INITIALIZE VARS
        print('Start')
        now = datetime.now()
        dt_string = now.strftime("%H-%M-%S")
        filename = filename + "_" + dt_string + ".txt"
        file = open(filename, "w+")
        self.filepath = filepath / filename
        print('File Created...')
        self.running = True

    def run(self, data):
        '''
        Inputs:
            input1 : list of data to save
            input2 : range/description

        Function: description
        '''
        #This method will be accessed by the Thread function
        #Loops continually

        if self.running:

            with open(self.filepath, "a") as file:
                for data in data:
                    data_string = "{:8}".format(data)
                    file.write(data_string)

    # def update(self,inputs):
    #     '''
    #     Inputs: (optional)
    #         input1 : range/description
    #         input2 : range/description
    #
    #     Function: description
    #
    #     Outputs:
    #         output1 : range/description
    #         output2 : range/description
    #     '''
    #     #This method will only be accessed once per drive cycle
    #     #Return values should go here
    #     return self.vars
    #
    # def shutdown(self):
    #     #Clean up anything when done
    #     self.running = False
    #
    # def function1(self, inputs):
    #     '''
    #     Inputs:
    #         input1 : range/description
    #         input2 : range/description
    #
    #     Function: description
    #
    #     Outputs:
    #         output1 : range/description
    #         output2 : range/description
    #     '''
    #     #Helper function for something in self.run() (optional)


class ImageLogger:
    '''
    saves images with current heading angle and loop counter
    '''
    def __init__(self, file):
        '''
        Inputs:
            file: filepath for new folder
        '''
        #create directory for images
        now = datetime.now()
        dt_string = now.strftime("%b-%d-%y_%H-%M-%S")
        os.makedirs(dt_string)
        #specify file path
        self.filepath = file / dt_string
        print('Directory Created...')


    def saveImage(self, input):
        #save input to File
        frame, heading, loop = input
        name = "drive_"+str(loop)+"_"+str(heading)+".png"
        filename = self.filepath / name
        cv2.imwrite(str(filename), frame)


if __name__ == '__main__':
    filepath = Path('C:/User/jazzy/Documents/KU_ARC/data/')
    filepath = Path('/data')
    saver = DataLogger('test_data', filepath=filepath)
