# Q-Six_axis-Robotic_Arm 
Path Planning with Autonomous Obstacle Avoidance Using Reinforcement Learning for Six-axis Arms
### Paper Link: [DOI:10.1109/ICNSC48988.2020.9238112](https://ieeexplore.ieee.org/document/9238112)

## Abstract
In this project, a strategy of path planning for autonomous obstacle avoidance using reinforcement learning for six-axis arms is proposed. This strategy gives priority to planning the obstacle avoidance path for the terminal of the mechanical arm, and then uses the calculated terminal path to plan the poses of the mechanical arm. For the points on the terminal path that the mechanical arm cannot avoid obstacles within the limit of the safe distance, this strategy will record these points as new obstacles and plan a new obstacle avoidance path for the terminal of mechanical arm. The above process is accelerated by the assisted learning strategies and looped until the correct path being calculated. The method proposed in this paper has been applied to a six-axis mechanical arm, and the simulation results show that this method can effectively plan an optimal path and poses for the mechanical arm.<br>
`The flow chart shows below:`

<div align=center>
<img src='https://github.com/Ys-Jia/Q-Six_axis-Robotic_Arm/blob/main/Flow%20Chart.png' width="600" height="600">
</div>

## How To Use
Please make sure all files in `Code_Package` are in your local directory!<br>
Need MATLAB 7.0 and onward with rvctools [`download`](https://petercorke.com/toolboxes/robotics-toolbox/)

There are two ways to use avoid simulation:<br>
Ⅰ. Open the file `Path_Schedule.m` and change obstacles coordinate and terminal coordinate(in line 25, terminal, line 26, obstacle) to satisfy your simulation environment.<br>
Ⅱ. Set the training parameters--Iterations, max_tolerance, learning rate ALPHA, GAMA <br>
Ⅲ. Run the file `Pose_Schedule_with_Reschedule.m` to atumaticially plan path and pose of robotic arm.<br>

Also, we have provided two successful training environment data--`[-1,2,0]_Obstacle.mat`, `[0,3,0]_Obstacle.mat`.<br>
Ⅰ. Load the file `[-1,2,0]_Obstacle.mat` or `[0,3,0]_Obstacle.mat`.<br>
Ⅱ. Run the file `Plot_Arm_Movement` to visualize the result. <br>


Here is the sample of terminal at [-1, 2, 0], the side of obstacle is 0.9:
<div align=center>
<img src='https://github.com/Ys-Jia/Q-Six_axis-Robotic_Arm/blob/main/(-1%2C%202%2C%200)Sample.gif' width="600" height="400">  
</div>
