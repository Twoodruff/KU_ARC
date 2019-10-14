'''
File: filename.py
Author: First Last
Date: mm/dd/yy
Revision: 0.1
Description: high level description of module.
             Module must include __init__, run, update, shutdown
'''

# IMPORTS

class ClassName:
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
            #do something
            x += 1 #example

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

    def function1(self, inputs):
        '''
        Inputs:
            input1 : range/description
            input2 : range/description

        Function: description

        Outputs:
            output1 : range/description
            output2 : range/description
        '''
        #Helper function for something in self.run() (optional)
