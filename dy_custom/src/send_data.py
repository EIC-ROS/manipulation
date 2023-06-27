import rospy
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import _thread
import time
import numpy as np

dobot_Enable = True

def Com2CR():
        DIN = 2
        DOUT = 3

        if client_dashboard.DI(DIN) != 0:    
            #Copy Digital Input from 
            Digital_Input = client_dashboard.DI(DIN)
            client_dashboard.DO(DOUT,Digital_Input)

def CR2Com():
        DIN = 2
        DOUT = 3
        if client_dashboard.DI(DIN) != 0:    
            #Copy Digital Input from 
            Digital_Input = client_dashboard.DI(DIN)
            client_dashboard.DO(DOUT,Digital_Input)

def stop_watch():
    global start, dobot_Enable
    if time.time() - start > 10:
        dobot_Enable = False
        

# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
client_feedback = dobot_api_feedback('192.168.5.6', 30003)


client_dashboard.DisableRobot()
time.sleep(1)

# Remove alarm
client_dashboard.ClearError()
time.sleep(0.5)

# Description The upper function was enabled successfully
client_dashboard.EnableRobot()
time.sleep(0.5)

try:
    while dobot_Enable == True:
        # Call the JointMovJ directive
        """
        Set digital signal output (Queue instruction)
        index : Digital output index (Value range:1~24)
        status : Status of digital signal output port(0:Low level，1:High level
        """
        Com2CR()
        CR2Com()
        
    
except KeyboardInterrupt:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    
client_dashboard.close()
client_feedback.close()
print("END program")



