# Q-Six_axis-Robotic_Arm 
Path Planning with Autonomous Obstacle Avoidance Using Reinforcement Learning for Six-axis Arms

## Abstract:
In this project, a strategy of path planning for autonomous obstacle avoidance using reinforcement learning for six-axis arms is proposed. This strategy gives priority to planning the obstacle avoidance path for the terminal of the mechanical arm, and then uses the calculated terminal path to plan the poses of the mechanical arm. For the points on the terminal path that the mechanical arm cannot avoid obstacles within the limit of the safe distance, this strategy will record these points as new obstacles and plan a new obstacle avoidance path for the terminal of mechanical arm. The above process is accelerated by the assisted learning strategies and looped until the correct path being calculated. The method proposed in this paper has been applied to a six-axis mechanical arm, and the simulation results show that this method can effectively plan an optimal path and poses for the mechanical arm.

### Paper Link: [DOI:10.1109/ICNSC48988.2020.9238112](https://ieeexplore.ieee.org/document/9238112)
