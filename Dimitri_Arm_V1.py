import numpy as np
import sys
import PyDynamixel_v2 as pd

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

MOTOR1_ID = 0
MOTOR2_ID = 1
MOTOR3_ID = 2
MOTOR4_ID = 3

# Instantiate port
port = pd.DxlComm(PORT, BAUDRATE)   

# Instantiate joint objects for both motors
motor1 = pd.Joint(MOTOR1_ID)
motor2 = pd.Joint(MOTOR2_ID)
motor3 = pd.Joint(MOTOR3_ID)
motor4 = pd.Joint(MOTOR4_ID)

port.attach_joints([motor1, motor2, motor3, motor4])

motor1.enable_torque()
motor2.enable_torque()
motor3.enable_torque()
motor4.enable_torque()

def def2rad(deg):
    '''Converts angles from degress to radians
    '''
    return np.pi * deg/ 180.0

def rad2deg(rad):
    ''' Converts angles from radians to degrees
    '''
    return 180.0 * rad / np.pi

def dh(a, alfa, d, theta):
    ''' Builds the Homogeneus Transformation matrix
        corresponding to each line of the Denavit-Hartenberg
        parameters
    '''
    m = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alfa), \
        np.sin(theta)*np.sin(alfa), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alfa), \
         -np.cos(theta)*np.sin(alfa), a*np.sin(theta)],
        [0,np.sin(alfa), np.cos(alfa), d],
        [0,0,0,1]
    ])
    return m


# Robot Arm Class
# ---------------
class Arm:


