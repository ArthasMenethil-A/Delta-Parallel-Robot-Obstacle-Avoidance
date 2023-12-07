# Delta Parallel Robot - Obstacle Avoidance
------
<ins>**A word from me:**</ins> 

This is the newest project I want to start working on. After figuring out trajectory planning of Delta Parallel Robot (DPR) in my previous project, I wanted to do something a bit more advanced. So the abmition for this project is:
**Detect the target object with Image Processing, then move it through dynamic obstacles to reach a certain position.**

So ... yeah ... it's a bit of a step up from my last project. If you need to ask any questions, here's my email: 
arvin1844m@gmail.com

And here are some of my Articles regarding these researchs: 
...WIP...

Overview: 
- Outline and Introduction
  - Object Detection
  - Grasping, Moving, and Calibration
  - Obstacle Avoidance
  - Errors
- Setup Challenges and Solutions 
  - Kinematics Errors
  - Homing Errors 
- Object Detection
  - Dataset
  - Yolo-V5 Implementation and Results
- References

## 1 - OUTLINE and INTRODUCTION 
------
There are 3 key elements to this project:

1. Object Detection
2. Grasping, Moving, and Calibration
3. Obstacle Avoidance

These 3 elements need to be implemented separately and then fused together. So let's talk about each of them individually. 


### 1.1 - Object Detection
-------
The objects that need to be detected are divided into two main classes: 

1. Targets for simple PPO (such as test tubes and dropping tubes)
2. Obstacles (objects that will block robot movement)

Of course, these two classes can include the same objects, for example if a test tube is not the target of the pick-and-place operation, then it's an obstacle. But we'll go into more detail later on. The main contenders for Object Detection are: 

- Open-CV: Classic Edge Detection
- Yolo-V5: Ultralytics
- Yolo-V8: Ultralytics
- Mask RCNN: Object Segmentation, Detection
- U-Net: Object Segmentation, Detection


### 1.2 - Grasping, Moving, and Calibration
-------
When an object is detected we need to take the following steps: 

1. Calibration: Convert image coordinates to robot coordinates
2. Move: Using Trajectory Planning, move the robot End-Effector (EE) to the given coordinates
3. Grasping: Grasp the object and the move it to another location.

The most challenging task in this section is none of three mentioned above, but rather the errors we have on the robot which need immediate attention. We'll talk about the errors later. 

### 1.3 - Obstacle Avoidance
-------
We need to be able to maneuver the robot through Static/Dynamic obstacles to reach a certain target. 

### 1.4 - Errors
-------
Before starting the project there are a bunch of errors that need attention: 

1. PID controller fine-tuning: $k_p, k_i, k_d$
2. Geometrical errors of kinematics calculations 
3. Calibration with camera
4. Homing errors
5. The Code is really messy and the user interface is lacking

## 2 - Setup Challenges and Solutions 
-------
This section is dedicated to addressing the issues of developement which may result in errors which decreases the overall performance of the robot.

### 2.0 - Modular Code 
-------

![Delta Parallel Robot](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/assets/69509720/140d367c-5eec-4489-b82d-ee53d1928131)

First and formost, the ideal way to code the controller is with C++ since it's a lot faster than Python. But since this is for the purpose of research and not industrial use, we're going to make do with Python. But the code needs to be more modular from here onwards. The codes are included [here](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/tree/main/DPR%20Controller). The files include the following content: 

- `DPR_drivercom.py`: The functions of communication with robot + PID controller
- `DPR_pathplanning.py`: The functions of Commands for path planning 
- `delta_robot.py`: The geometry of the robot 
- `path_planning_mltp.py`: the functions of multi point path planning 
- `path_planning_ptp.py`: the functions of point to point path planning

### 2.1 - Kinematics Errors
-------

![DPR scheme](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/assets/69509720/d15208d6-85fe-4506-92e2-a2755f0116d3)

The problem of kinematic and geometry is this: we design a robot and build it, but the built model will never be a perfect replika to the designed model. In this instance let's say we set the three upper arms of the robot to be 30.9 cm, but the built model will have three arms of 31, 30.5, 31.2 cm. This means the calculations that we simplify in calculating the forward and inverse kinematics will give us a certain error. We don't want that error. The full report on the kinematics study of DPR is included in [this file](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/tree/main/Research/Kinematic%20Study) [1].

### 2.2 - PID Controller Errors
-------
This was pretty simple in conceptual terms. We have a PID controller and the coefficients of kp, ki, kd are not fine-tuned and pretty random. That make the robot slow and inaccurate. So regarding this matter to fix it, in the file `DPR_pathplanning.py` we wrote the following function: 

```
def PID_goto(last_position, duration):
   

    kp3 = float(input('kp:\n'))
    ki3 = float(input('ki:\n'))
    kd3 = float(input('kd:\n'))

    pid3.set_PID_constants(kp3, ki3, kd3)

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
    
```

Using this function we could basically change the PID values and output a plot of the robot behavior. After starting with the values of $k_p = 0.1, \quad k_i = 0.01, \quad k_d = 0.01$ for all motors, we reach the following values for each motor: 

```
kp1 = 1.3
ki1 = 0.0008
kd1 = 0.12

kp2 = 1.6
ki2 = 0.0006
kd2 = 0.1

kp3 = 1.9
ki3 = 0.0006
kd3 = 0.1
```

So the robot movement became incredibly more precise and faster. The precision measured when moving a distance of 10 cm in 0.8 seconds was 0.5 mm which was a huge improvement to previous errros.


### 2.3 - Homing Errors
-------
*WIP* -Using Microswitch-


## 3 - Image Processing
-------
You know what object detection is. the first goal of the alogrithm is to detect some classes of objects. Then we have to identify which one is the target and which one is the obstacle. 

## 3.1 - Dataset
-------
The dataset used is link [here](https://universe.roboflow.com/delta-parallel-robot-obstacle-avoidance-project/tube-detection-x52mi). The classes of our dataset are:

- Tube - Open
- Tube - Close
- Tube - Full
- Tube Rack
- Tube Available Space
- Petridish


## REFERENCES 
------
[1] Kinematic Analysis of Delta Parallel Robot: Simulation Study - A. Eltayeb

