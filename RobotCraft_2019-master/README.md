# RobotCraft_2019
Work done during RobotCraft 2019 internship

These packags contain basic framework to solve a simple wall maze with the RobotCraft robot (powered by Arduino Mega 2560
and Raspberry Pi 3) in simulator and in the real world.

## Submodule descriptions
### Maze solving in the Stage Simulator

### robotcraft_maze
Solves the maze using wall following algorithm while also mapping the maze using 2D RPlidar compatible with ROS.

### robotcraft_maze_pro 
Solves the maze using wave propagation algorithm to compute the shortest path, uses wheel odometry for localization, runs in the Stage Simulator.

### robotcraft2019_driver_g09
A square following algorithm can be tested in the Stage simulator, uses rotation and line-following control laws.

### Maze solving with the real robot running on Raspberry Pi 3 and Arduino
### robotcraft_maze_real
Solves the maze using wall following algorithm and ultrasonic distance sensors while also mapping the maze using 2D RPlidar compatible with ROS.

### robotcraft_maze_pro_real
Solves the maze using wave propagation algorithm to compute the shortest path, uses wheel odometry for localization, requires a map of the maze to calculate the shortest path.

### robotcraft_maze_pro_real_amcl
Solves the maze using wave propagation algorithm to compute the shortest path, uses amcl ROS package and a 2D lidar for localization, requires a map of the maze to calculate the shortest path. (was not completed due to hardware problems)

### controllMT
Motor control package, runs on Arduino Mega 2560, intefaces with Raspberry Pi 3 using rosserial package. Includes model based and standard PI controllers for speed control of the two wheel motors.

### controllMT_final
final version of controllMT
