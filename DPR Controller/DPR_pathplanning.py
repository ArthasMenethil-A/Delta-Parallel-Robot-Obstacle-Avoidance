# Move the robot manually. This file the highest level functions of the DPR controller files

# It also has a function for implementing the PID controller class 

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

from path_planning_mltp import * 
from delta_robot import * 
from DPR_drivercom import * 

# joystick mode 
from pygame import K_UP 
from pygame import K_DOWN 
from pygame import K_LEFT 
from pygame import K_RIGHT 
from pygame import K_SPACE
from pygame import K_w 
from pygame import K_q 
from pygame import K_s 


# =================================================================================================
# -- constants ------------------------------------------------------------------------------------
# =================================================================================================

# these are the pid controller coefficients for each of the motors which are fine tuned to our robot 

kp1 = 1.3
ki1 = 0.0008
kd1 = 0.12


kp2 = 1.6
ki2 = 0.0006
kd2 = 0.1


kp3 = 1.9
ki3 = 0.0006
kd3 = 0.1



#3 pids for 3 angles

pid1=PID(kp1,ki1,kd1)
pid2=PID(kp2,ki2,kd2)
pid3=PID(kp3,ki3,kd3)



# =================================================================================================
# -- path planning point to point -----------------------------------------------------------------
# =================================================================================================

# helper function for determining if a point is inside workspace 
def _is_point_inside_triangle(P):

    # Set vertices for the triangle
    vertices = np.array([[15, 35], [-35, -5], [25, -33]])

    A, B, C = vertices

    # Calculate the vectors from A to the test point P
    v0 = C - A
    v1 = B - A
    v2 = P - A

    # Calculate dot products
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    # Calculate barycentric coordinates
    inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    # Check if the point is inside the triangle
    return (u >= 0) and (v >= 0) and (u + v <= 1)





# moves the EE from initial point to final point 
# final_xyz = [x_final, y_final, z_final]
def Goto_xyz(final_xyz, duration, trajectory='4567'):
    

    # if not(_is_point_inside_triangle(final_xyz[0:2]) and (final_xyz[2] <= -37) and (final_xyz[2] >= -70)):
    #     return 
    
    
    start_time = datetime.datetime.now()
    
    # init values 
    current_position    = [0, 0, 0]
    last_position       = [0, 0, 0]
    distance            = [0, 0, 0]
        
    E, current_position[0], current_position[1], current_position[2] = Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

    print("Current Position is: ", current_position)

    # calculating the distance from the goal
    for i in range(3):
        distance[i] = final_xyz[i] - current_position[i]
    
    print("distance:", distance)
        
    dtime = 0 
    
    # history 
    time_history = []
    theta_history = []
    ee_history = []
    angular_speed_history = []

    while dtime<duration:

        # emergency stop with q on keyboard 
        if keyboard.is_pressed('q'):
            Emergency_stop()
            print("Loop terminated by user!")
            break

        # checking the passed time 
        last_time = datetime.datetime.now()
        dtime = (last_time - start_time).total_seconds()

        # after reaching destination
        if dtime>=duration:
            Motion_z_endeffector(0)
            Motion_z_endeffector(0)
            break

        # trajectory 
        tau = dtime/duration
        if trajectory=='4567':
            s = _trajectory_4567(duration, 0, start_time)
        elif trajectory=='345':
            s = _trajectory_345(duration, 0, start_time)

        for i in range(3): 
            last_position[i] = s*distance[i] + current_position[i]

        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)
        
        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])

        # uncomment if want to save plot 
        # appending history 
        # time_history = time_history + [dtime/duration]
        # theta_history = theta_history + [[in1, in2, in3]]
        # ee_history = ee_history + [Forward(in1, in2, in3)[1:4]]
        # angular_speed_history = angular_speed_history + [system_input]
        
    
    Motion_z_endeffector(0)



    # uncomment if want to save plot 
    # with open('time_history_' + trajectory + '.npy', 'wb') as file:
    #     np.save(file, time_history)
    # with open('theta_history_' + trajectory + '.npy', 'wb') as file:
    #     np.save(file, theta_history)
    # with open('ee_history_' + trajectory + '.npy', 'wb') as file:
    #     np.save(file, ee_history)
    # with open('angular_speed_history_' + trajectory + '.npy', 'wb') as file:
    #     np.save(file, angular_speed_history)

# helper function of 4-5-6-7 interpolating polynomial trajectory
def _trajectory_4567(time_map,time_shift,start_time): #return value within 0 , 1
    last_time=datetime.datetime.now()
    dtime=(last_time-start_time).total_seconds()
    tau=(dtime-time_shift)/time_map
    s=-20*(tau**7)+70*(tau**6)-84*(tau**5)+35*(tau**4)
    return s

# helper function of 3-4-5 interpolating polynomial trajectory
def _trajectory_345(time_map, time_shift, start_time):
    last_time = datetime.datetime.now()
    dtime = (last_time - start_time).total_seconds()
    tau = (dtime - time_shift)/time_map
    s = 6*(tau**5) - 15*(tau**4) + 10*(tau**3)
    return s 




# =================================================================================================
# -- path planning multi point --------------------------------------------------------------------
# =================================================================================================

# moves the EE through certain points
# path = [[x1, y1, z1], [x2, y2,z2], ... [xn, yn, zn]]
def mltp_cubic_spline(path, duration):

    global answer2

    # init values 
    current_position    = [0, 0, 0]
    last_position       = [0, 0, 0]

    E, current_position[0], current_position[1], current_position[2] = Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3))

    print("Current Position is: ", current_position)

    # the path should become: [[x0, y0, z0], [x1, y1, z1], ... [xn, yn, zn]]
    path = np.array([current_position] + path)

    print("This is the path:", path, "\nshape", path.shape)

    # path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
    path_criteria = path.T
    
    # print("this is the path criteria", path_criteria)

    # defining the robot 
    delta_robot = DeltaRobot(30.9, 59.5, 13, 10)

    # defining the path planner 
    path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)

    # cubic spline 
    (_, _, _, _, _, ThetaVector) = path_planner.cubic_spline()
    

    start_time = datetime.datetime.now()
    dtime = 0 
    
    # history 
    time_history = []
    theta_history = []
    ee_history = []
    angular_speed_history = []


    while dtime<duration:

        # safety 
        if keyboard.is_pressed('q'):
            Emergency_stop()
            print("Loop terminated by user!")
            break

        # checking the passed time 
        last_time = datetime.datetime.now()
        dtime = (last_time - start_time).total_seconds()


        if dtime>=duration:
            Motion_z_endeffector(0)
            break

        FREQUENCY = 10000 
        tau = dtime/duration
        index = int(tau*FREQUENCY)

        for i in range(3): 
            last_position[i] = ThetaVector[index][i]
            # if i == 2:
            #     print("this is the trajectory being generated", last_position[i])
 
        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)

        
        # print("this is the input speed", system_input)


        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])

        # uncomment if want to save plot 
        # appending history 
        # time_history = time_history + [dtime/duration]
        # theta_history = theta_history + [[in1, in2, in3]]
        # ee_history = ee_history + [Forward(in1, in2, in3)[1:4]]
        # angular_speed_history = angular_speed_history + [system_input]
        
    
    Motion_z_endeffector(0)
    Motion_z_endeffector(0)
    Motion_z_endeffector(0)

    # uncomment if want to save plot 
    # with open('time_history_' + 'cubic_spline' + '.npy', 'wb') as file:
    #     np.save(file, time_history)
    # with open('theta_history_' + 'cubic_spline' + '.npy', 'wb') as file:
    #     np.save(file, theta_history)
    # with open('ee_history_' + 'cubic_spline' + '.npy', 'wb') as file:
    #     np.save(file, ee_history)
    # with open('angular_speed_history_' + 'cubic_spline' + '.npy', 'wb') as file:
    #     np.save(file, angular_speed_history)



# =================================================================================================
# -- movement using PID without trajectory  -------------------------------------------------------
# =================================================================================================

def PID_goto(last_position, duration):
   
   
    start_time = datetime.datetime.now()

    dtime = 0
    
    time_history = []
    velocity_history = []


    while dtime<duration:

        last_time = datetime.datetime.now()
        dtime = (last_time - start_time).total_seconds()
        time_history.append(dtime)
        
        in1 = Position_absolute_read(1)
        in2 = Position_absolute_read(2)
        in3 = Position_absolute_read(3)
        
        

        feedback = [in1, in2, in3]

        system_input = implement_PID(last_position, feedback)

        velocity_history.append(system_input)


        Target_speed_rpm(1, system_input[0])
        Target_speed_rpm(2, system_input[1])
        Target_speed_rpm(3, system_input[2])
        
    
    print(np.max(np.abs(np.array(velocity_history)),axis = 0))

    Motion_z_endeffector(0)
    plt.plot(time_history,velocity_history, label=[["motor1"], ["motor2"], ["motor3"]])
    plt.legend()
    plt.show()
    



# =================================================================================================
# -- manual control -------------------------------------------------------------------------------
# =================================================================================================

def EE_manual_controller(movement_speed=0.1):
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("DPR EE Manual Controller")

    # Keep track of key states
    key_up_pressed = False
    key_down_pressed = False
    key_left_pressed = False
    key_right_pressed = False
    key_w_pressed = False
    key_s_pressed = False
    key_q_pressed = False

    is_key_pressed =    key_up_pressed or key_down_pressed or key_left_pressed or key_right_pressed \
                        or key_w_pressed or key_s_pressed

    running = True

    while running:
        for event in pygame.event.get():

            # checking if any key is pressed 
            is_key_pressed =    key_up_pressed or key_down_pressed or key_left_pressed or key_right_pressed \
                                or key_w_pressed or key_s_pressed

            if event.type == pygame.KEYDOWN and event.key == K_q:
                Motion_z_endeffector(0)
                pygame.quit()
                sys.exit()
                return 
            if event.type == pygame.KEYDOWN and event.key == K_SPACE:
                print(Forward(Position_absolute_read(1), Position_absolute_read(2), Position_absolute_read(3)))
            elif event.type == pygame.KEYDOWN and not is_key_pressed:
                
                if event.key == K_UP:
                    print("key up is pressed")
                    Motion_y_endeffector(movement_speed)
                    key_up_pressed = True
                elif event.key == K_DOWN:
                    print("key down is pressed")
                    Motion_y_endeffector(-movement_speed)
                    key_down_pressed = True
                elif event.key == K_LEFT:
                    print("key left is pressed")
                    Motion_x_endeffector(-movement_speed)
                    key_left_pressed = True
                elif event.key == K_RIGHT:
                    print("key right is pressed")
                    Motion_x_endeffector(movement_speed)
                    key_right_pressed = True
                elif event.key == K_w:
                    print("key w is pressed")
                    Motion_z_endeffector(-movement_speed)
                    key_w_pressed = True
                elif event.key == K_s:
                    print("key s is pressed")
                    Motion_z_endeffector(movement_speed)
                    key_s_pressed = True

            elif event.type == pygame.KEYUP:
                if event.key == K_UP and not (is_key_pressed and not key_up_pressed):
                    print("key up is released")
                    Motion_z_endeffector(0)
                    key_up_pressed = False
                elif event.key == K_DOWN and not (is_key_pressed and not key_down_pressed):
                    print("key down is released")
                    Motion_z_endeffector(0)
                    key_down_pressed = False
                elif event.key == K_LEFT and not (is_key_pressed and not key_left_pressed):
                    print("key left is released")
                    Motion_z_endeffector(0)
                    key_left_pressed = False
                elif event.key == K_RIGHT and not (is_key_pressed and not key_right_pressed):
                    print("key right is released")
                    Motion_z_endeffector(0)
                    key_right_pressed = False
                elif event.key == K_w and not (is_key_pressed and not key_w_pressed):
                    print("key w is released")
                    Motion_z_endeffector(0)
                    key_w_pressed = False
                elif event.key == K_s and not (is_key_pressed and not key_s_pressed):
                    print("key s is released")
                    Motion_z_endeffector(0)
                    key_s_pressed = False

    pygame.quit()




# =================================================================================================
# -- PID controller impelementation ---------------------------------------------------------------
# =================================================================================================



def implement_PID(set_point_coord,feedback):
    
    controllerOutput = []

    # converting xyz coord to angle by inverse kinematics
    E,theta1,theta2,theta3=Inverse(set_point_coord[0],set_point_coord[1],set_point_coord[2])
    
    #system input is the return value of controller
    pid1.DefineSetpoint(theta1)
    pid2.DefineSetpoint(theta2)
    pid3.DefineSetpoint(theta3)
   
    controllerOutput.append(pid1.Compute(feedback[0]))
    controllerOutput.append(pid2.Compute(feedback[1]))
    controllerOutput.append(pid3.Compute(feedback[2]))
    
    return controllerOutput




# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

def main():
    Activate_serial()
    Enable_all_drivers(-3)

if __name__ == "__main__":
	main()