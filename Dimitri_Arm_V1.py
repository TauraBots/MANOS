import numpy as np
import import PyDynamixel_v2 as pd

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
    m = np.arrat([
        np.con(theta), -np.sin(theta)*np.cos(alfa), \
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


