import numpy as np
import sys
#import PyDynamixel_v2 as pd
import keyboard
import time
import pandas
import pydmps.dmp_discrete
import numpy as np
import psot

pandas.set_option('display.max_rows', 500)
pandas.set_option('display.max_columns', 500)
pandas.set_option('display.width', 1000)

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

MOTOR1_ID = 0
MOTOR2_ID = 1
MOTOR3_ID = 2
MOTOR4_ID = 3
MOTOR5_ID = 4 # aberto: 260 fechado: 166
MOTOR6_ID = 5 # aberto: 100 fechado: 196

# Instantiate port
'''port = pd.DxlComm(PORT, BAUDRATE)   

# Instantiate joint objects for both motors
motor1 = pd.Joint(MOTOR1_ID)
motor2 = pd.Joint(MOTOR2_ID)
motor3 = pd.Joint(MOTOR3_ID)
motor4 = pd.Joint(MOTOR4_ID)
motor5 = pd.Joint(MOTOR5_ID)
motor6 = pd.Joint(MOTOR6_ID)

port.attach_joints([motor1, motor2, motor3, motor4, motor5, motor6])

motor1.enable_torque()
motor2.enable_torque()
motor3.enable_torque()
motor4.enable_torque()
motor5.enable_torque()
motor6.enable_torque()'''

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
        [np.cos(theta), -np.sin(theta)*np.cos(alfa), 
        np.sin(theta)*np.sin(alfa), a*np.cos(theta)], 
        [np.sin(theta), np.cos(theta)*np.cos(alfa), 
        -np.cos(theta)*np.sin(alfa), a*np.sin(theta)], 
        [0,np.sin(alfa), np.cos(alfa), d], 
        [0,0,0,1]
    ])
    return m

def DMP (y_des):
    ''' Dynamic Movement Primitive 
    '''
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=4, n_bfs=500, ay=np.ones(4)*10.0)
    y_track = []
    dy_track = []
    ddy_track = []
    dmp.imitate_path(y_des=y_des, plot=False)

    y_track, dy_track, ddy_track = dmp.rollout()
    return y_track

# Robot Arm Class
# ---------------
class ArmJacobian:

    def __init__(self):
        self.zeros = np.array([210.0, 180.0, 65.0, 153.0])
        self.goals = np.array([0.0 for i in range(4)])
        
    def sendAngles(self):
        motors = [motor1,motor2,motor3,motor4]
        for i in range(4):
            motors[i].send_angle(self.goals[i] + self.zeros[i])
        #    print(self.goals[i]+self.zeros[i],self.goals[i],self.zeros[i])

    def fk(self):
        '''Forward Kinematics
        '''
        #convert angles from degress to radians
        t = [deg2rad(x) for x in self.goals]
        #print(t)
        #register the DH parameters
        hs = []
        hs.append(dh(0, -np.pi/2, 4.3, t[0]))
        hs.append(dh(0, np.pi/2, 0.0,     t[1]))
        hs.append(dh(0, -np.pi/2, 24.3, t[2]))
        hs.append(dh(27.0, np.pi/2, 0.0, t[3]-np.pi/2))

        m = np.eye(4)
        j = np.zeros(6*4).reshape(6,4)
        d_01 = [np.array([0,0,0])]
        r_01 = [np.array([0,0,1])]
        
        for h in hs:
            m = m.dot(h)
            d_01.append(np.array(m[0:3,3]))
            r_01.append(np.array(m[0:3,2]))
        
        for i in range(4):
            j[0:3,i] = np.cross(r_01[i],m[0:3,3]-d_01[i])
            j[3:6,i] = r_01[i]

        return m, j

    def getPoint(self, m, p):
        p = np.array([p[0], p[1], p[2], 1.0])
        q = m.dot(p)
        return np.array([q[0]/q[3], q[1]/q[3], q[2]/q[3]])

    def ik(self, iterations, positions):
        ''' Inverse Kinematics
        '''
        x, y, z, roll, pitch, yaw = positions/float(iterations)
        time.sleep(1)
        df = pandas.DataFrame()
        for i in range(iterations):
            _, j = self.fk()
            inv_j = np.linalg.pinv(j)
            v = np.array([x, y, z, roll, pitch, yaw])
            dq = inv_j.dot(v)
            dq[2] = -dq[2]
            self.goals[:4] = self.goals[:4] + rad2deg(dq)
            dgoals = pandas.DataFrame(self.goals).T
            frame = [df, dgoals]
            df = pandas.concat(frame)
            print(df)

        return df

    def tracking(self, track, sl):
        ''' Receive one dataframe with 4 columns and send goals for 4 motors
        '''
        for i in range(250):
            send = track.iloc[i,:].values
            self.goals[:4] = send
            time.sleep(sl)
            self.sendAngles()
            print(send)

class ArmPSO:
    def fk(self,goals):
        # Convert angles from degress to radians
        t = [deg2rad(x) for x in goals]

        # Register the DH parameters
        m1 = dh(0,    -np.pi/2, 4.3,   t[0])
        m2 = dh(0,    np.pi/2,  0.0,   t[1])
        m3 = dh(0,    -np.pi/2, 24.3,  t[2])
        m4 = dh(27.0, np.pi/2,  0.0,   t[3]-np.pi/2)

        # Multiply all matrix and colect the position vector
        m = m1.dot(m2)
        m = m.dot(m3)
        m = m.dot(m4)
        m = np.array(m[0:3,3])
        
        # f(|Actual - Target|) is the result to send to PS0
        m[0] = abs(m[0] - self.target[0]) 
        m[1] = abs(m[1] - self.target[1])
        m[2] = abs(m[2] - self.target[2])
        m = m[0] + m[1] + m[2]
        print (m)
        
        return m

    def __init__(self, target, iterations):
        self.target = target
        self.iterations = iterations
        zeros = np.array([210.0, 180.0, 65.0, 153.0])
        self.goals = zeros

        # Initial starting location [x1,x2...]
        initial=[0,0,0,0]               
        
        # Input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
        bounds=[(-zeros[0],abs(360-zeros[0])),(-zeros[1], abs(360-zeros[1])),\
                (-zeros[2],abs(360-zeros[2])),(-zeros[3],abs(360-zeros[3]))]  
        
        # Inverse Kinematics
        global_best_position, global_best_error = psot.pso(self.fk,initial,bounds,num_particles=1000,iterations=300)
        
        # Create motors and split the sum for final angles
        #motors = [motor1,motor2,motor3,motor4]
        s_goals = np.array(np.array(global_best_position)/self.iterations)
        
        # Save all states of sum in DataFrame track
        track = pandas.DataFrame()
        for i in range(self.iterations):
            self.goals[0] = self.goals[0] + s_goals[0]
            self.goals[1] = self.goals[1] + s_goals[1]
            self.goals[2] = self.goals[2] + s_goals[2]
            self.goals[3] = self.goals[3] + s_goals[3]
            print(self.goals)
            dgoals = pandas.DataFrame(self.goals).T
            frame = [track, dgoals]
            track = pandas.concat(frame)
        print(track)

        # Return DMP tracking
        dmp_track = DMP(track.T.values)
        dmp_track = pandas.DataFrame(dmp_track)
        time.sleep(1.5)
        sl = 0.05  
        
        # Split and send angles
        for i in range(self.iterations):
            send = dmp_track.iloc[i,:].values
            self.goals[:4] = send
            time.sleep(sl)
            #self.sendAngles(motors)
            print(send)

        print('\nTarget: ', target)
        print('Initial goals: ', zeros)
        print('Goals: ', send)
        print('Erro: ', global_best_error)

    def sendAngles(self, motors):
        motors[0].send_angle(self.goals[0])
        motors[1].send_angle(self.goals[1])
        motors[2].send_angle(self.goals[2])
        motors[3].send_angle(self.goals[3])

def openg():
    ''' Open the Grab
    '''
    motor5.send_angle(260)
    motor6.send_angle(100)

def fechar():
    ''' Close the Grab
    '''
    motor5.send_angle(180)
    motor6.send_angle(190)

if __name__ =='__main__':  

    '''
    a = ArmJacobian()
    a.sendAngles()
    positions = np.array([30.0, 0.0, -20.0, 0.0, 0.0, 0.0])
    iterations = 100

    # Create the matrix tracking of inverse kinematics 
    track = a.ik(iterations, positions)

    # Return DMP tracking
    dmp_track = DMP(track.T.values)
    dmp_track = pandas.DataFrame(dmp_track)
    time.sleep(1.5)
    a.tracking(dmp_track, 0.05)
    '''
    
    # 5 = aberto: 260 fechado: 180
    # 6 = aberto: 100 fechado: 190

    # Close the grab and sleep 0.5s
    #fechar()
    time.sleep(0.5)
    
    # Send initial default angles and sleep 0.5s
    #a = ArmJacobian()
    #a.sendAngles()
    time.sleep(0.5)
    
    # Open the grab
    #openg()
    
    # Set positions XYZ, iterations number and send both for PSO algorithm
    target = np.array([45,19,5])
    iterations = 100
    ArmPSO(target, iterations)
    
    # Close the grab and sleep 2s
    #fechar()
    time.sleep(2)
    
    # Set positions XYZ, iterations number and send both for PSO algorithm
    target = np.array([20,-30,10])
    iterations = 100
    ArmPSO(target, iterations)
    
    # Sleep 2s and open the grab 
    time.sleep(2)
    #openg()
