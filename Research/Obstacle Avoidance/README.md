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

### 1.2 Defintion of Obstacle Avoidance
------
Let $A$ be the robot moving in the workspace $W$, whose configuration space is $CS$. Let $q$ be a configuration, $q_t$ this configuration at time $t$, and $A(q_t) \in W$ the space occupied by the robot in this configuration. 
