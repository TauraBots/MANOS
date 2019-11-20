#Runing with PyDynamixel_V2
#Need to keep it on folder

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

while True:

    motor1.enable_torque()
    motor2.enable_torque()
    motor3.enable_torque()
    motor4.enable_torque()

    pos1 = motor1.get_angle()
    pos2 = motor2.get_angle()
    pos3 = motor3.get_angle()
    pos4 = motor4.get_angle()
    print('setou')
    motor1.send_angle(220)

    print(pos1, pos2, pos3, pos4)