# This file is used for the basic commands to the driver for example enabling/disabling drivers and shit 

# The PID controller class is also in this file 

# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

import numpy as np
import math
import time
import serial  ## for serial communication: pip install pyserial
import matplotlib.pyplot as plt
import pygame
import keyboard
import sys 
import datetime

import timeit
import struct
import scipy.io as sio ## for reading and writing MATLAB mat files
from numpy import linalg as LA
import matplotlib.pyplot as plt
from scipy.optimize import fsolve


# =================================================================================================
# -- max tries ------------------------------------------------------------------------------------
# =================================================================================================

max_tries = 20                      # maximum tries of driver for reading encoder position
Gear_ratio = 50                     # important------check before run
Rated_torque = 2.68                 # Motor Rated Torque
encoder_resolution = 10000          #unit: inc
L1= 0.305                           # Upper Arm (actuated link) length
L2= 0.595                           # Fore Arm (parallelogram link) length
r = 0.10                            # End-Effector radious
R = 0.13                            # base radious
r1 = L1/2                           # geometric center of actuated link
r2 = L2/2                           # geometric center of parallelogram link


global offset_pos


# kinematics 
e= (1/math.tan(np.deg2rad(30))) * 20
f= (1/math.tan(np.deg2rad(30))) * 26
re = 59.5
rf =  30.9

#s      = 165*2
sqrt3  = math.sqrt(3.0)
pi     = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3


# =================================================================================================
# -- driver functions -----------------------------------------------------------------------------
# =================================================================================================


def Activate_serial():
    global Actuator1_serial, Actuator2_serial, Actuator3_serial
    STARTING_ROBOT = input("Activate Serials?(y/n)")
    if STARTING_ROBOT.upper() == 'Y':
        Actuator1_serial = serial.Serial(port='COM3',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
        Actuator2_serial = serial.Serial(port='COM4',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)
        Actuator3_serial = serial.Serial(port='COM5',baudrate=38400,parity='N',stopbits=1, bytesize=8, timeout=None, xonxoff=False, rtscts=False,  writeTimeout=None, dsrdtr=False, interCharTimeout=None)


def Close_serial():

    Actuator1_serial.close()
    Actuator2_serial.close()
    Actuator3_serial.close()


    
def Chks_calculation(value) :
    value =  -1*sum(value)  
    value = 2**32 + value 
    chks = list(struct.unpack('>4B',struct.pack('>L',value)))      
    return chks[3]



def Conv_to_hex(value) :
    if value < 0 :
        value = 2**32 + value  
    Hexadecimal = list(struct.unpack('>4B',struct.pack('>L',value)))
    return Hexadecimal



def Write_register(ID, CMD_data_volume, left_index, right_index, subindex, object_value):
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, CMD_data_volume, right_index, left_index, subindex, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)

    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 : 
        Actuator2_serial.write(Down_master_send) 
    elif ID == 3 : 
        Actuator3_serial.write(Down_master_send)
    


def Drive_enable(ID):
    object_value = 0x2F
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x40, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)
    
    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 :
        Actuator2_serial.write(Down_master_send)
    elif ID == 3 :
        Actuator3_serial.write(Down_master_send)



def Drive_disable(ID) :
    object_value = 0x06
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x40, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)
    
    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
    elif ID == 2 :
        Actuator2_serial.write(Down_master_send)
    elif ID == 3 :
        Actuator3_serial.write(Down_master_send)



def Operation_mode(ID,Mode):
    value_Hex = Conv_to_hex(Mode)
    Down_master_send = [ID, 0x2F, 0x60, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)

    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    if ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    if ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()



def Current_mode(ID):
    Actuator1_serial.flushInput()
    
    Up_master_send = [ID, 0x40, 0x60, 0x60, 0x00, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    
    
    if ID == 1 :
        Actuator1_serial.write(Up_master_send)
        Up_slave_response = list(Actuator1_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data
    
    elif ID == 2:
        Actuator2_serial.write(Up_master_send)
        Up_slave_response = list(Actuator2_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data
    
    elif ID == 3:
        Actuator3_serial.write(Up_master_send)
        Up_slave_response = list(Actuator3_serial.read(10))
        value = Up_slave_response
        [print('0x{0:0{1}X}'.format(_bytes, 2)[2:], end =" | ") for _bytes in Up_slave_response]
        data = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
        print(data)
        return data



def Target_speed_rpm(ID,Speed_rpm):
    if Speed_rpm > 20:
        Speed_rpm = 20
    if Speed_rpm < -20:
        Speed_rpm = -20
        
    Speed_Decimal = math.floor(Speed_rpm*2730.66*Gear_ratio)  ###### remember to add 50 for gearbox effect
    object_value = Speed_Decimal
    value_Hex = Conv_to_hex(object_value)
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x23, 0xFF, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)

    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    elif ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    elif ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()



def Torque_speed_limit (ID,max_current,max_speed):
    Write_register(ID,0x2b,0x60,0x80,0x0,max_speed)



def Target_torque (ID,Target_Torque_Nm):
    # Actuator1_serial.flushInput()
    '''rated torque for this type of motor is equal to: 2.68 nm'''
    if Target_Torque_Nm > 20:
        Target_Torque_Nm = 20
    if Target_Torque_Nm < -20:
        Target_Torque_Nm = -20
    Target_Torque_Decimal = math.floor( (Target_Torque_Nm*100*10) / (Rated_torque*Gear_ratio) )  
    object_value = Target_Torque_Decimal
    value_Hex = Conv_to_hex(object_value)
    Down_master_send = [ID, 0x2B, 0x71, 0x60, 0x00, int(value_Hex[3]), int(value_Hex[2]), int(value_Hex[1]), int(value_Hex[0])]
    Down_master_send.append(Chks_calculation(Down_master_send))
    Down_master_send = bytearray(Down_master_send)


    if ID == 1 : 
        Actuator1_serial.write(Down_master_send)
        Actuator1_serial.flushInput()
    if ID == 2 : 
        Actuator2_serial.write(Down_master_send)
        Actuator2_serial.flushInput()
    if ID == 3 : 
        Actuator3_serial.write(Down_master_send)
        Actuator3_serial.flushInput()



def Enable_all_drivers(mode):

    answer = input("Is robot in home position?(y/n)")

    if answer.upper() != "Y":
        return

    ## Torque mode ==> mode = 4
    ## Speed Mode ==> mode = -3
    Drive_enable(1)
    Drive_enable(2)
    Drive_enable(3)
    Operation_mode(1,mode)
    Operation_mode(2,mode)
    Operation_mode(3,mode)
    #Set speed rpms to 0 for safety
    Target_speed_rpm(1,0)
    Target_speed_rpm(2,0)
    Target_speed_rpm(3,0)
    
    Offset()

    input("\033[91mPlease Remove The Bars!\033[0m")

def Disable_all_drivers():

    disable_flag = input("Are you sure you want to disable?(y/n)")
    if disable_flag.upper() != "Y":
        return
        
    Target_speed_rpm(1,0)
    Target_torque(1, 0)
    Drive_disable(1)
    # time.sleep(0.008)
    Target_speed_rpm(2,0)
    Target_torque(2, 0)
    Drive_disable(2)
    # time.sleep(0.008)
    Target_speed_rpm(3,0)
    Target_torque(3, 0)
    Drive_disable(3)
    # time.sleep(0.008)

def Emergency_stop(): # emergency stop 
    Target_speed_rpm(1,0)
    Target_speed_rpm(2,0)  
    Target_speed_rpm(3,0)



def Position_convert(value):
    position_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        position_hex = -2**32 + position_hex
    position_deg = ((position_hex/10000)*360)/Gear_ratio   ##### remember to add 50 for gearbox
    return position_deg



def Velocity_convert(value) :
    velocity_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        velocity_hex = -2**32 + velocity_hex
    velocity_rpm = (velocity_hex/2730.66)/Gear_ratio  ##### remember to add 50 for gearbox
    return velocity_rpm



def Torque_convert(value)  :
    torque_hex = ((value[8] * 16**6) +(value[7] * 16**4) +(value[6] * 16**2) + value[5])
    if value[8] >= 255:
        torque_hex = -2**32 + torque_hex
    torque_nm = ( (torque_hex*Rated_torque*Gear_ratio) / (10*100) )    ##### remember to add 50 for gearbox

    return torque_nm



def Position_actual(ID):
    Up_master_send = [ID, 0x40, 0x63, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    if ID == 1 :
        Actuator1_serial.flushInput()       
        Actuator1_serial.write(Up_master_send)
        Up_slave_response = list(Actuator1_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator1_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break
            
    elif ID == 2 :
        Actuator2_serial.flushInput()      
        Actuator2_serial.write(Up_master_send)
        Up_slave_response = list(Actuator2_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator2_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break       
            
    elif ID == 3 :
        Actuator3_serial.flushInput()   
        Actuator3_serial.write(Up_master_send)
        Up_slave_response = list(Actuator3_serial.read(10))

        while len(Up_slave_response) !=10 :

            Up_slave_response = list(Actuator3_serial.read(10))
            print("Unable to read position data from Driver={}".format(ID))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read position data from Driver={}".format(ID))
                break
               
    if ( len(Up_slave_response)!=10 ):
        return -333333
    position_deg = Position_convert(Up_slave_response)

    return position_deg 



def Offset():
    global offset_pos

    offset_1 = 26
    offset_2 = 27
    offset_3 = 23

    offset_pos = [offset_1, offset_2, offset_3]
    
    print("This is offset:", offset_pos)



def Position_absolute_read(ID):
    global offset_pos
    if 'offset_pos' not in globals():
        offset_pos = [0, 0, 0]
    Pos_relative_read = Position_actual(ID)
    num_of_tries = 0
    while Pos_relative_read == -333333:
        Pos_relative_read = Position_actual(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in position data readings from driver={}".format(ID))
            break

    position_abs =  Position_actual(ID) - offset_pos[ID-1]

    return position_abs



def Velocity_actual_rpm(ID) :
    Actuator1_serial.flushInput()
    Actuator2_serial.flushInput()    
    Actuator3_serial.flushInput()
    '''torque: 6077'''
    '''speed:  606C'''
    Up_master_send = [ID, 0x40, 0x6C, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    if ID == 1 :
        # Actuator1_serial.flushInput()
        Actuator1_serial.write(Up_master_send)
        Actuator1_serial.flushInput()
        Up_slave_response = list(Actuator1_serial.read(10))
        Actuator1_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator1_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break
    
    elif ID == 2 :
        # Actuator1_serial.flushInput()
        Actuator2_serial.write(Up_master_send)
        Actuator2_serial.flushInput()
        Up_slave_response = list(Actuator2_serial.read(10))
        Actuator2_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator2_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break

    elif ID == 3 :
        # Actuator1_serial.flushInput()
        Actuator3_serial.write(Up_master_send)
        Actuator3_serial.flushInput()
        Up_slave_response = list(Actuator3_serial.read(10))
        Actuator3_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read velocity data from Driver={}".format(ID))
            Up_slave_response = list(Actuator3_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read velocity data from Driver={}".format(ID))
                break
    
    if ( len(Up_slave_response)!=10 ):
        return -666666
    velocity_rpm = Velocity_convert(Up_slave_response)

    return velocity_rpm



def Velocity_read_rpm(ID):
    velocity_rpm = Velocity_actual_rpm(ID)
    num_of_tries = 0
    while velocity_rpm == -666666 :
        velocity_rpm = Velocity_actual_rpm(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in velocity data readings from driver={}".format(ID))
            break
    return velocity_rpm



def Torque_actual(ID):
    Actuator1_serial.flushInput()
    Actuator2_serial.flushInput()    
    Actuator3_serial.flushInput()

    '''torque: 6077'''
    Up_master_send = [ID, 0x40, 0x77, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0]
    Up_master_send.append(Chks_calculation(Up_master_send))
    Up_master_send = bytearray(Up_master_send)
    num_of_tries = 0
    
    if ID == 1 :
        # Actuator1_serial.flushInput()
        Actuator1_serial.write(Up_master_send)
        Actuator1_serial.flushInput()
        Up_slave_response = list(Actuator1_serial.read(10))
        Actuator1_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator1_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break
    
    elif ID == 2 :
        # Actuator1_serial.flushInput()
        Actuator2_serial.write(Up_master_send)
        Actuator2_serial.flushInput()
        Up_slave_response = list(Actuator2_serial.read(10))
        Actuator2_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator2_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break      
            
    elif ID == 3 :
        # Actuator1_serial.flushInput()
        Actuator3_serial.write(Up_master_send)
        Actuator3_serial.flushInput()
        Up_slave_response = list(Actuator3_serial.read(10))
        Actuator3_serial.flushInput()   
        while len(Up_slave_response) !=10 :
            print("Unable to read torque data from Driver={}".format(ID))
            Up_slave_response = list(Actuator3_serial.read(10))
            num_of_tries = num_of_tries + 1
            if num_of_tries == max_tries:
                print("Max Tries Unable to read torque data from Driver={}".format(ID))
                break
    
    if ( len(Up_slave_response)!=10 ):
        return -999999
    torque_nm = Torque_convert(Up_slave_response)
    # while abs(torque_nm) > (3000*Gear_ratio) :
        # return -999999

    return torque_nm



def Torque_read_nm(ID):
    torque_nm = Torque_actual(ID)
    num_of_tries = 0
    while torque_nm == -999999 :
        torque_nm = Torque_actual(ID)
        num_of_tries += 1
        if num_of_tries > max_tries :
            Emergency_stop()
            Disable_all_drivers()
            print("fault in torque data readings from driver={}".format(ID))
            break
    return torque_nm



def Motion_z_endeffector(speed):
    Target_speed_rpm(1,speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,speed)
        
def Motion_y_endeffector(speed):
    Target_speed_rpm(1,-1*speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,speed)

def Motion_x_endeffector(speed):
    Target_speed_rpm(1,speed)
    Target_speed_rpm(2,speed)
    Target_speed_rpm(3,-1*speed)

def Motion_theta1(speed):
    Target_speed_rpm(1, speed)
    Target_speed_rpm(2, 0)
    Target_speed_rpm(3, 0)
    
def Motion_theta2(speed):
    Target_speed_rpm(1, 0)
    Target_speed_rpm(2, speed)
    Target_speed_rpm(3, 0)

def Motion_theta3(speed):
    Target_speed_rpm(1, 0)
    Target_speed_rpm(2, 0)
    Target_speed_rpm(3, speed)

# =================================================================================================
# -- kinematics -----------------------------------------------------------------------------------
# =================================================================================================

def Forward(theta1, theta2, theta3):
    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    
    t = (f-e) * tan30 / 2.0
    dtr = pi / 180.0
    
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr
    
    y1 = -(t + rf*math.cos(theta1) )
    z1 = -rf * math.sin(theta1)
    
    y2 = (t + rf*math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)
    
    y3 = (t + rf*math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)
    
    dnm = (y2-y1)*x3 - (y3-y1)*x2
    
    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3
    
    # x = (a1*z + b1)/dnm
    a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
    b1= -( (w2-w1)*(y3-y1) - (w3-w1)*(y2-y1) ) / 2.0
    
    # y = (a2*z + b2)/dnm
    a2 = -(z2-z1)*x3 + (z3-z1)*x2
    b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0
    
    # a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    d = b*b - 4.0*a*c
    if d < 0.0:
        return [1,0,0,0] 
    
    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm

    return [0,x0,y0,z0]

# Inverse kinematics

def _angle_yz(x0, y0, z0, theta=None):
    y1 = -0.5*0.57735*f # f/2 * tg 30
    y0 -= 0.5*0.57735*e # shift center to edge
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2.0*z0)
    b = (y1-y0) / z0

    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if d<0:
        return [1,0] 

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1) 
    zj = a + b*yj
    theta = math.atan(-zj / (y1-yj)) * 180.0 / pi + (180.0 if yj>y1 else 0.0)
    
    return [0,theta] # return error, theta

def Inverse(x0, y0, z0):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    status = _angle_yz(x0,y0,z0)

    if status[0] == 0:
        theta1 = status[1]
        status = _angle_yz(x0*cos120 + y0*sin120,
                                   y0*cos120-x0*sin120,
                                   z0,
                                   theta2)
    if status[0] == 0:
        theta2 = status[1]
        status = _angle_yz(x0*cos120 - y0*sin120,
                                   y0*cos120 + x0*sin120,
                                   z0,
                                   theta3)
    theta3 = status[1]

    return [status[0],theta1,theta2,theta3]


# =================================================================================================
# -- PID ------------------------------------------------------------------------------------------
# =================================================================================================


class PID:
    def __init__(self,Kp,Ki,Kd,setPoint=0,SampleTime=60): #sample time is in millisconds
        if Kp<0 or Kd<0 or Ki<0:
            print("invalid pid constants")
            return
        self.SampleTime=SampleTime
        sampleTimeInSec=SampleTime/1000 
        self.kp=Kp
        self.ki=Ki#*sampleTimeInSec
        self.kd=Kd#/sampleTimeInSec
        self.lastError=0
        self.integralTerm=0 #used for I term
        self.integralTerm_torque = 0 #used for I term
        self.lastTime=datetime.datetime.now()
#        startTime=startTime.seconds()*1000
        self.minOutput=0
        self.maxOutput=0
        self.Error=0

    def Compute(self,feedback):
        presentTime=datetime.datetime.now()
        timeChange=(presentTime-self.lastTime).total_seconds()*1000

        if timeChange>self.SampleTime: #if a time interval equal to sample time has passed
            #Compute Working Error Variables
            self.Error = self.setPoint-feedback
            # print("this is error: " + str(self.Error))
            dError = self.Error-self.lastError #error- last error
            
            
            self.lastError = self.Error #added
            
            
            self.integralTerm = self.integralTerm+self.ki*self.Error

            derivativeTerm = self.kd*dError
            
            proportionalTerm = self.kp*self.Error
            
            PIDOutput = self.integralTerm + derivativeTerm + proportionalTerm
            
            if self.maxOutput != 0 and PIDOutput>self.maxOutput:
                PIDOutput=self.maxOutput
                
            elif self.minOutput != 0 and PIDOutput<self.minOutput:
                PIDOutput=self.minOutput
                
            return PIDOutput
        
        self.lastTime = presentTime
            
    def Compute_torque_pid(self,feedback):
        
        # Mass_matrix = I_bt + m_ee_total*(J_x).T * J_x

        # Torque_grav = -(J_x).T * ( m_ee_total + (3/2)*m_forearm )*[0 , 0 , 9.81] - L1*(0.5*m_upper_arm + m_middle_joint + 0.5*m_forearm)*9.81*[np.cos(theta_des_actlink(1)) , np.cos(theta_des_actlink(2)) , np.cos(theta_des_actlink(3))] 

        
        presentTime=datetime.datetime.now()
        timeChange=(presentTime-self.lastTime).total_seconds()*1000

        if timeChange>self.SampleTime: #if a time interval equal to sample time has passed
            
            #Compute Working Error Variables
            self.Error = self.setPoint-feedback
            # print("this is error: " + str(self.Error))
            dError = self.Error - self.lastError #error- last error
            
            self.integralTerm_torque = -self.integralTerm + self.ki*self.Error*4

            derivativeTerm_torque = self.kd*dError*-15
            
            proportionalTerm_torque = self.kp*self.Error*-12
            
            PIDOutput_torque = -(self.integralTerm_torque + derivativeTerm_torque + proportionalTerm_torque )
            
            if self.maxOutput!=0 and PIDOutput_torque >self.maxOutput:
                PIDOutput_torque = self.maxOutput
            elif self.minOutput!=0 and PIDOutput_torque <self.minOutput:
                PIDOutput_torque=self.minOutput
            return PIDOutput_torque
        
        self.lastTime=presentTime
        
    def SetSampleTime(self,newSampleTime):
        ratio = newSampleTime/self.SampleTime
        self.ki = self.ki*ratio
        self.kd = self.kd/ratio
        self.SampleTime = newSampleTime

    def SetOutputLimits(self,minOut,maxOut):
        self.minOutput = minOut
        self.maxOutput = maxOut
        
    def DefineSetpoint(self,coord):
        self.setPoint = coord
        
    def set_PID_constants(self,Kp,Ki,Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd




# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

def main():
    pass 

if __name__ == "__main__":
    main()
