# -*- coding: utf-8 -*-

"""
PyDynamixel TauraBots Library Version 2.0
Supports both Protocol 1 and Protocol 2.0 dynamixels.
Most of the code here was taken from the dynamixel_sdk examples and previous PyDynamixel library.
"""

from dynamixel_sdk import *
from math import pi
import sys

PROTOCOL_VERSION                = 1

# MX-28 table
if PROTOCOL_VERSION == 1:
    ADDR_MX_TORQUE_ENABLE       = 24    # Address for torque enable
    ADDR_MX_PRESENT_POSITION    = 36    # Address for the current position
    ADDR_MX_GOAL_POSITION       = 30    # Address for goal position
    LEN_MX_GOAL_POSITION        = 2     # MX's have different message lengths in Protocol 1 vs. 2
                                        # and protocol 1.0 does not use present_position length
elif PROTOCOL_VERSION == 2:
    ADDR_MX_TORQUE_ENABLE       = 64    # Address for torque enable
    ADDR_MX_PRESENT_POSITION    = 132   # Address for the current position
    ADDR_MX_GOAL_POSITION       = 116   # Address for goal position
    LEN_MX_GOAL_POSITION        = 4
    LEN_MX_PRESENT_POSITION     = 4


class DxlComm(object):
    ''' This class implements low level
    communication with the dynamixel
    protocol.
    '''

    def __init__(self, port="/dev/ttyUSB0", baudrate=None, baudnum=None):
        ''' Initializes DxlComm.
        port: the path to the serial device. Default is /dev/ttyUSB0.
        The constructor optionally takes a baudnum argument:
           baudrate = 2Mbps / (baudnum + 1)
        If no baudrate or baudnum is provided, the default baudrate is 57600.
        '''

        self.port           = port

        if baudrate:
            self.baudrate = baudrate
        elif baudnum:
            # Baudnum selected in dynamixel config in RoboPlus Software
            self.baudrate = 20000000/(baudnum + 1)
        else:
            self.baudrate = 57600

        self.port_handler   = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.joint_ids  = [] # All attached joint ids
        self.joints     = [] # All attached joints (whole class)
        self.total      = 0  # Total joints attached

        # Connection to declared serial port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

        # Set port baudrate
        if self.port_handler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

        # Instantiates sync read and write variables
        if PROTOCOL_VERSION == 2:
            # Sync Read is only present in Protocol 2.0
            self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
        self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

    def attach_joints(self, joints):
        ''' This method attaches a list of joints so that
        the communication can be handled by this class.
        '''
        for joint in joints:
            self.attach_joint(joint)

    def attach_joint(self, joint):
        ''' This method attaches a single joint so
        that the communication can be handled by this class.
        '''
        # Registers the joint in the database
        self.joints.append(joint)
        self.joint_ids.append(joint.servo_id)
        joint._set_port_and_packet(self.port_handler, self.packet_handler)
        self.total = self.total + 1
        self.joint_ids.sort()

    def broadcast_ping(self): # CHECK
        ''' Broadcast ping to all attached joints.
        Finally, checks if detected dynamixels are equal to previously attached joints.
        Only for Protocol 2.0.
        Return: list of detected dynamixels.
        '''
        if PROTOCOL_VERSION == 2:
            detected_servos = []

            dxl_data_list, dxl_comm_result = self.packet_handler.broadcastPing(self.port_handler)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))

            print("Detected Dynamixels: ")
            for servo_id in dxl_data_list:
                detected_servos.append(servo_id)
                print("[ID: %03d] model version: %d | firmware version: %d" % (servo_id, dxl_data_list.get(servo_id)[0], dxl_data_list.get(servo_id)[1]))

            if len(detected_servos) != self.total:
                print("Detected servos value different from total added joints. Check for connection issues.")

            return detected_servos
        else:
            print("Broadcast Ping only available for Protocol 2.0.")
            return False

    def send_angles(self, values=None, radian=False):
        ''' Sends goal position with sync write to all or desired servos.

        If no values are sent, then it gets which servos to send goal values to from
        their 'changed' param, which was modified by each joint set_goal_value() function.

        The 'values' dict should have the name length of current connected joints.
        Therefore, if you chose to send through the 'values' way, you will send angles
        to all connected dynamixels.

        Usage: (assuming 3 attached joints)
            self.send_angles({dyn1_id: 90, dyn3_id: 30, dyn2_id: 60})
            OR
            self.send_angles({dyn3_id: pi/2, dyn2_id: pi/6, dyn1_id: pi/3}, radian=True)
            OR
            self.joints[0].set_goal_value(90) OR self.joints[0].set_goal_value(pi/2, radian=True)
            self.send_angles()
        '''
        if values == None:
            ch_joints = [j for j in self.joints if j.changed is True]
            self._sync_write(servos=ch_joints)

            for i in ch_joints:
                i.changed = False
        else:
            if isinstance(values, dict): #this needs further checking
                list_values = [values[i] for i in sorted(values)] # sort dict by joint id
                if len(list_values) == len(self.joint_ids):
                    # list_values is sorted as is serial.joint_ids (from lowest to highest id)
                    self._sync_write(values=list_values, radian=radian)
            else:
                print("Make sure the values parameter is a list of the same length of connected dynamixels.")


    def _sync_write(self, servos=None, values=None, radian=False):
        ''' this is an adaptation from dynamixel's sdk for
            the sync_write '''
        if servos != None:
            for i, s in enumerate(servos):
                # goal_value = self.set_goal_value(s.goal_value, radian=radian)
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(s.goal_value)), DXL_HIBYTE(DXL_LOWORD(s.goal_value)), DXL_LOBYTE(DXL_HIWORD(s.goal_value)), DXL_HIBYTE(DXL_HIWORD(s.goal_value))]
                dxl_add_param_result = self.group_sync_write.addParam(s.servo_id, param_goal_position)
                if dxl_add_param_result != True:
                    print("[ID: %03d] groupSyncWrite addParam failed" %s.servo_id)
        elif values != None:
            for i, goal_value in enumerate(values):
                goal_value = self.set_goal_value(goal_value, radian=radian)
                servo_id = self.joint_ids[i]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_value)), DXL_HIBYTE(DXL_LOWORD(goal_value)), DXL_LOBYTE(DXL_HIWORD(goal_value)), DXL_HIBYTE(DXL_HIWORD(goal_value))]
                dxl_add_param_result = self.group_sync_write.addParam(servo_id, param_goal_position)
                if dxl_add_param_result != True:
                    print("[ID: %03d] groupSyncWrite addParam failed" % servo_id)

        dxl_comm_result = self.group_sync_write.txPacket() # sync write
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        self.group_sync_write.clearParam() # clears buffer

    def get_angles(self, radian=False):
        ''' Read angles from all attached joints.
        Only available in Protocol 2.0'''
        if PROTOCOL_VERSION == 2:
            servos_angles = self._sync_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION, radian=radian)
            return servos_angles
        else:
            # no sync read for Protocol 1.0
            for joint in self.joints:
                joint.get_angle(radian=radian)

    def _sync_read(self, addr, info_len, radian=False):
        ''' Sync read. Only available in Protocol 2.0
        '''
        if PROTOCOL_VERSION == 2:
            for i, servo_id in enumerate(self.joint_ids):
                # add all servos to sync read request
                dxl_add_param_result = self.group_sync_read.addParam(servo_id)
                if dxl_add_param_result != COMM_SUCCESS:
                    #print("[ID: %03d] groupSyncRead addParam failed." % servo_id)
                    pass

            # SyncRead present position
            dxl_comm_result = self.group_sync_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))

            # check if sync read data from each dynamixel is available
            # then, get save that data
            servos_position = {}
            for i, servo_id in enumerate(self.joint_ids):
                dxl_getdata_result = self.group_sync_read.isAvailable(servo_id, addr, info_len)
                if dxl_getdata_result != True:
                    print("[ID:0%3d] groupSyncRead getdata failed." % servo_id)
                else:
                    dxl_present_position = self.group_sync_read.getData(servo_id, addr, info_len)
                    # put the returned value into a dict with all values and in the joints curr_value variable
                    if radian:
                        present_angle = (pi*dxl_present_position)/2048.0
                    else:
                        present_angle = (180*dxl_present_position)/2048.0
                    servos_position[servo_id] = present_angle
                    self.joints[i].curr_value = present_angle

            self.group_sync_read.clearParam()

            # return the dict
            return servos_position
        else:
            print("Sync Read is only available in Protocol 2.0.")
            return False

    def enable_torques(self):
        ''' Enable torque for all motors connected
        in this port.
        '''
        self.packet_handler.write1ByteTxRx(self.port_handler, BROADCAST_ID, ADDR_MX_TORQUE_ENABLE, 1)

    def disable_torques(self):
        ''' Disables torque for all motors connected
        to this port
        '''
        self.packet_handler.write1ByteTxRx(self.port_handler, BROADCAST_ID, ADDR_MX_TORQUE_ENABLE, 0)

    def set_goal_value(self, angle, radian=False):
        '''Sets goal value (0 to 1024), from given goal angle.
        '''
        goal_angle = float(angle)
        if radian:
            goal_value = int(2048.0*angle/pi)
        else:
            goal_value = int(2048.0*angle/180)
        return goal_value

    def release(self):
        ''' This method should be called for
        the class to explicitly close the
        open port_handler
        '''
        self.port_handler.closePort()

class Joint(object):
    ''' This class represents a single Dynamixel servo motor.
    Contains functions to ping, reboot, get angles and send angles to the dynamixel.
    '''

    def __init__(self, servo_id, center_value=0):
        ''' The constructor takes the servo id
        as the argument. Argument center_value
        can be set to calibrate the zero
        position of the servo.
        '''
        self.servo_id = servo_id
        self.center_value = center_value
        self.goal_angle = -1

    def _set_port_and_packet(self, port_handler, packet_handler):
        ''' This sets this joint's packet and port handlers to be equal to DxlComm ones.
        These variables are used to communicate with dynamixel.
        '''
        self.packet_handler = packet_handler
        self.port_handler = port_handler

    def set_goal_value(self, angle, radian=False):
        '''Sets goal value (0 to 1024), from given goal angle.
        Usage:
            set_goal_value(90) -> angle in degrees
            set_goal_value(pi/2, radian=True) -> angle in radians
        '''
        self.goal_angle = float(angle)
        if radian:
            self.goal_value = int(2048.0*angle/pi) + self.center_value
        else:
            self.goal_value = int(2048.0*angle/180) + self.center_value
        self.changed = True

    def send_angle(self, angle, radian=False):
        ''' Sends a command to this specific servomotor to set
        its goal value from a degree or radian goal angle.
        Usage:
            send_angle(90) -> sends 90 degree angle
            OR
            send_angle(pi/2, radian=True) -> send 90 degree angle, but in radians
            OR
            set_goal_value(90) OR set_goal_value(pi/2, radian=True)
            send_angle()
        '''
        #if angle >= 0:
        self.set_goal_value(angle, radian=radian)
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_GOAL_POSITION, self.goal_value)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        # precisa fazer changed = false?

    def get_angle(self, radian=False):
        ''' Reads the current position of this
        servomotor alone. The read position is
        stored in both self.curr_angle and self.curr_value.
        However, only self.curr_angle is retorned by this function.
        '''
        if PROTOCOL_VERSION == 1:
            self.curr_value, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_PRESENT_POSITION)
        elif PROTOCOL_VERSION == 2:
            self.curr_value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_PRESENT_POSITION)

        #self.curr_value -= self.center_value #TODO implement this in other functions

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

        if radian:
            self.curr_angle = pi*float(self.curr_value)/2048.0
        else:
            self.curr_angle = 180*float(self.curr_value)/2048.0

        return self.curr_angle

    def enable_torque(self):
        ''' Enables torque in this joint
        Usage: self.enable_torque()
        '''
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def disable_torque(self):
        ''' Disables torque in this joint
        Usage:
            self.disable_torque()
        '''
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_TORQUE_ENABLE, 0)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def ping(self): # check
        ''' Ping this joint.
        Usage:
            self.ping()
        '''
        dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.servo_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            print("[ID: %03d] ping succeeded. Model number: %d" % (self.servo_id, dxl_model_number))
            return self.servo_id

    def reboot(self): # check
        ''' Reboots this joint.
        Only works in Protocol 2.0.
        Usage:
            self.reboot()
        '''
        if PROTOCOL_VERSION == 2:
            dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, self.servo_id)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            else:
                print("[ID: %03d] reboot succeeded" % self.servo_id)
        else:
            print("Reboot is only present in Protocol 2.0.")
