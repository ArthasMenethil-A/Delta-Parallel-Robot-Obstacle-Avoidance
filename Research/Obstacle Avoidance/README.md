# Obstacle Avoidance Study
------
Here I'll try to get a better understanding of the topic at hand, Obstacle Avoidance. 

Overview: 
-  Springer Handbook of Robotics: Chapter 35 - Motion Planning and Obstacle Avoidance
-  ...

## 1. Springer Handbook of Robotics: Chapter 35 - Motion Planning and Obstacle Avoidance
------
This chapter describes motion planning and obstacle avoidance for **mobile robots**.

### 1.1 From Motion Planning to Obstacle Avoidance
------
Up to now we have described motion planning techniques. Their objective is to compute a collision-free trajectory to the target configuration that complies with the vehicle constraints. They assume a **perfect model** of the robot and scenario. The advantage of these techniques is that they provide complete and **global solutions** of the problem. Nevertheless, **when the surroundings are unknown and unpredictable, these techniques fail**.

### 1.2 Definition of Obstacle Avoidance
------
![Untitled](https://github.com/ArthasMenethil-A/Delta-Parallel-Robot-PPO-And-Obstacle-Avoidance/assets/69509720/6c110f68-4b1f-49fb-98d7-bc6d8df5d386)

The obstacle avoidance problem consists of computing a motion control that avoid collisions wtih the obstacles gathered by the sensors, whilst driving the robot towards the target location. The result of applying this technique at each time is a sequence of motions that drive the vehicle free of collisions to the target. 

### 1.3 Potential Field Methods 
------
ذ
