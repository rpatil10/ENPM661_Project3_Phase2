# Project 3: Implementation A* algorithm for turtlebot in 2D and 3D(ROS-Gazebo)
## _ENPM661_
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

## Authors
Rohit M Patil
Email ID: rpatil10@umd.edu

Anish Mitra
Email ID: amitra12@umd.edu

## Description
Implementation of A* algorithm on a differential drive (non-holonomic) TurtleBot robot. Navigate a differential drive robot (TurtleBot) in a given  map environment from a given start point to a given goal point. Considered differential drive constraints while implementing the A* algorithm, with 8 set of action space. Checks the feasibility of all inputs/outputs (if user gives start and goal nodes that are in the obstacle space they will be informed by a message and they should try again). Code outputs an animation/video of optimal path generation between start and goal point on the map. It shows both the node exploration as well as the optimal path generated.

## Dependencies

| Plugin | 
| ------ |
| heapq |
| numpy | 
| cv2 | 
| math | 
| queue | 
| ast | 
| sys | 
| argv |
| time |
| argv |
| rospy |
| rospkg |
| Twist |

| Software | 
| ------ |
|Python|
| ROS |
|Gazebo|
|Rviz|


## Part 1: 2D turtlebot
Clone or download the project repository and open the Part1 folder,
**Test case 1:**
Input start location as shown in below fashion, **start_x_location,start_y_location,start_orientation** _space_ **goal_x_location,goal_y_location** _space_ **rpm1,rpm2,clearance**
Open terminal in the part1 folder and type below command:
```bash
python turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5
```
or
```bash
python3 turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5
```

**Output**
**Part1 testcase 1 visualization video:** [Test case 1](https://youtu.be/7hsKKAhZoWE).
![Optimal_path_testcase1](/outputs/Part1_2D_A_star_turtlebot_testcase_1_image.png?raw=true)

**Test case 2:**
Input start location as shown in below fashion, **start_x_location,start_y_location,start_orientation** _space_ **goal_x_location,goal_y_location** _space_ **rpm1,rpm2,clearance**
Open terminal in the part1 folder and type below command:
```bash
python turtlebot_A_star.py -4,1,0 4.25,2.75 30,25,5
```
or
```bash
python3 turtlebot_A_star.py -4,1,0 4.25,2.75 30,25,5
```
**Output**
**Part1 testcase 2 visualization video:** [Test case 2](https://youtu.be/IZhh9Sk9AvM).
![Optimal_path_testcase2](/outputs/Part1_2D_A_star_turtlebot_testcase_2_image.png?raw=true)

## Part 2: 3D turtlebot
Traverse to location containing **proj3_ws**. This is the workspace that contains all the files necessary to run A* for Turtlebot3 in ROS-Gazebo. 
Once inside **proj3_ws**, type the following commands to prepare the files for execution:
```bash
cd ~(your location)/proj3_ws/
catkin_make
source devel/setup.bash
```
If any errors pop up, **caktin_make** can be replaced by:
```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
Traverse into the package **proj3_gazebo** and run the 2D A* Program to generate wheel velocities for 3D implementation:
```bash
cd src/proj3_gazebo
python3 turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5
```
To create an executable for **a_star_turtlebot_gazebo.py** located in **proj3_ws/src/proj3_gazebo/src/** , run the following command:
```bash
cd proj3_ws/src/proj3_gazebo/src/
chmod +x a_star_turtlebot_gazebo.py
```
To execute the launch file and view turtlebot in gazebo, run the following command:
```bash
roslaunch proj3_gazebo map.launch
```
If you wish to view output for different **start_x_location,start_y_location,start_orientation**, first generate the velocities by running **turtlebot_A_star.py** (as shown earlier) and change the following in the **map.launch** file located in  **proj3_ws/src/proj3_gazebo/launch/** in line 13,14,15:
```bash
13 <arg name="x_pos" default="(your start_x_location)"/>
14 <arg name="y_pos" default="(your start_y_location)"/>
15 <arg name="z_pos" default="(your start_orientation)"/>
```
NOTE: Be **absolutely** certain that the arguments entered while running the 2D implementation are the **same as the values entered in the launch file**. 
**Output**
![Optimal_path_testcase2](/outputs/Part2_3D_A_star_gazebo.png?raw=true)

**Video Output**
**Part2 visualization video:** [-](https://youtu.be/N0UJI5H6qMY).
