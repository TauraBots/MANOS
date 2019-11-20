import numpy as np
import sys
import PyDynamixel_v2 as pd
import keyboard
import time

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

def deg2rad(deg):
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
        np.sin(theta)*np.sin(alfa), a*np.cos(theta)], \
        [np.sin(theta), np.cos(theta)*np.cos(alfa), \
        -np.cos(theta)*np.sin(alfa), a*np.sin(theta)], \
        [0,np.sin(alfa), np.cos(alfa), d], \
        [0,0,0,1]
    ])
    return m


# Robot Arm Class
# ---------------
class Arm:

    def __init__(self):
        self.zeros = np.array([208.0, 246.0, 66.0, 214.0])
        self.goals = np.array([0.0 for i in range(4)])

   
    def sendAngles(self):
        motors = [motor1,motor2,motor3,motor4]
        for i in range(4):
            motors[i].send_angle(self.goals[i] + self.zeros[i])


    def fk(self):
        '''Forward Kinematics
        '''
        #convert angles from degress to radians
        t = [deg2rad(x) for x in self.goals]

        #register the DH parameters
        hs = []
        hs.append(dh(0, -np.pi/2, 4.3, t[0]))
        hs.append(dh(20.0, np.pi/2, 0,t[1]))
        hs.append(dh(0, -np.pi/2, 4.3, t[2]))
        hs.append(dh(20.8, 0, 0, t[3]))

        m = np.eye(4)
        d_01 = [np.array([0,0,0])]
        r_01 = [np.array([0,0,1])]

        for h in hs:
            m = m.dot(h)
            d_01.append(np.array(m[0:3,3]))
            r_01.append(np.array(m[0:3,2]))
        
        return m

    def getPoint(self, m, p):
        p = np.array([p[0], p[1], p[2], 1.0])
        q = m.dot(p)
        return np.array([q[0]/q[3], q[1]/q[3], q[2]/q[3]])


if __name__ == '__main__':
    a = Arm()
    quit = False
    count = 0
    a.sendAngles()
    time.sleep(2)
    while not quit:
        if keyboard.is_pressed('esc'):
            quit = True
        count = count + 1
        for i in range(4):
            a.goals[2]  =  30*np.sin(deg2rad(count*2))
            a.sendAngles()
        m = a.fk()
        print(a.getPoint(m, [0,0,0]))


