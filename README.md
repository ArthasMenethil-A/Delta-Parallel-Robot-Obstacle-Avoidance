# Delta Parallel Robot - Obstacle Avoidance
------
<ins>**A word from me:**</ins> 

This is the newest project I want to start working on. After figuring out trajectory planning of Delta Parallel Robot (DPR) in my previous project, I wanted to do something a bit more advanced. So the abmition for this project is:
**Detect the target object with Image Processing, then move it through dynamic obstacles to reach a certain position.**

[FULL RESEARCH OUTLINE](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/tree/main/Research/Full%20Research%20Outline)

So ... yeah ... it's a bit of a step up from my last project. If you need to ask any questions, here's my email: 
arvin1844m@gmail.com

And here are some of my Articles regarding these researchs: 
...WIP...

Overview:
- Setup Challenges and Solutions 
  - Kinematics Errors
  - Homing Errors 
- Object Detection
  - Dataset
  - Yolo-V5 Implementation and Results
- References

## 1 - Setup Challenges and Solutions 
-------
This section is dedicated to addressing the issues of developement which may result in errors which decreases the overall performance of the robot.

### 1.1 - Modular Code 
-------

![Delta Parallel Robot](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/assets/69509720/140d367c-5eec-4489-b82d-ee53d1928131)

First and formost, the ideal way to code the controller is with C++ since it's a lot faster than Python. But since this is for the purpose of research and not industrial use, we're going to make do with Python. But the code needs to be more modular from here onwards. The codes are included [here](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/tree/main/DPR%20Controller). The files include the following content: 

- `DPR_drivercom.py`: The functions of communication with robot + PID controller
- `DPR_pathplanning.py`: The functions of Commands for path planning 
- `delta_robot.py`: The geometry of the robot 
- `path_planning_mltp.py`: the functions of multi point path planning 
- `path_planning_ptp.py`: the functions of point to point path planning

### 1.2 - Kinematics Errors
-------

![DPR scheme](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/assets/69509720/d15208d6-85fe-4506-92e2-a2755f0116d3)

The problem of kinematic and geometry is this: we design a robot and build it, but the built model will never be a perfect replika to the designed model. In this instance let's say we set the three upper arms of the robot to be 30.9 cm, but the built model will have three arms of 31, 30.5, 31.2 cm. This means the calculations that we simplify in calculating the forward and inverse kinematics will give us a certain error. We don't want that error. The full report on the kinematics study of DPR is included in [this file](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/blob/main/Research/Kinematic%20Study/DPR___kinematic_study.pdf) [1].

### 1.3 - PID Controller Errors
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

So the robot movement became incredibly more precise and faster. The precision measured when moving a distance of 20 cm in 0.8 seconds was 0.2 mm which was a huge improvement to previous errros.


### 1.4 - Homing Errors
-------
*WIP* -Using Microswitch-


## 2 - Image Processing
-------
You know what object detection is. the first goal of the alogrithm is to detect some classes of objects. Then we have to identify which one is the target and which one is the obstacle. 

## 2.1 - Dataset
-------
The dataset used is link [here](https://universe.roboflow.com/delta-parallel-robot-obstacle-avoidance-project/tube-detection-x52mi). The classes of our dataset are:

- Tube - Open
- Tube - Close
- Tube - Full
- Tube Rack
- Tube Available Space
- Petridish


![Dataset Samples](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-Obstacle-Avoidance/assets/69509720/cfebec13-f06b-463e-8c68-321850d283dd)




## REFERENCES 
------
[1] Kinematic Analysis of Delta Parallel Robot: Simulation Study - A. Eltayeb

