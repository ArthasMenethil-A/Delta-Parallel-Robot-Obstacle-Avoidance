
## RESEARCH OUTLINE
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
