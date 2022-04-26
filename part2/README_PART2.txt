
part2
|
|
|--- proj3_ws

	|--- build
	|
	|--- devel
	|       
	|--- src
		|--- proj3_gazebo
			|
			|--- launch
			|
			|--- src
			|
			|--- worlds
			|
			|
			|----- turtlebot_A_star.py
			|
			|
			|----- /utilityfunctions
			|       |
			|       |--- constants.py
			|       |
			|       |--- explorer.py
			|       |
			|       |--- node.py
			|       |
			|		|--- obstacle_space.py
			|
			|----- /images
			|		|
			|		|--- map.png (generated at runtime)
			|
			|----- /path_files
			|		|
			|		|--- velocities.txt (generated at runtime)


Traverse to location containing proj3_ws. This is the workspace that contains all the files necessary to run A* for Turtlebot3 in ROS-Gazebo. 
Once inside proj3_ws, type the following commands to prepare the files for execution:

cd ~(your location)/proj3_ws/
catkin_make
source devel/setup.bash

If any errors pop up, caktin_make can be replaced by:

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
Traverse into the package proj3_gazebo and run the 2D A* Program to generate wheel velocities for 3D implementation:

cd src/proj3_gazebo
python3 turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5

To create an executable for a_star_turtlebot_gazebo.py located in proj3_ws/src/proj3_gazebo/src/ , run the following command:

cd proj3_ws/src/proj3_gazebo/src/
chmod +x a_star_turtlebot_gazebo.py

To execute the launch file and view turtlebot in gazebo, run the following command:

roslaunch proj3_gazebo map.launch

If you wish to view output for different start_x_location,start_y_location,start_orientation, first generate the velocities by running turtlebot_A_star.py (as shown earlier) and change the following in the map.launch file located in  proj3_ws/src/proj3_gazebo/launch/ in line 13,14,15:

13 <arg name="x_pos" default="(your start_x_location)"/>
14 <arg name="y_pos" default="(your start_y_location)"/>
15 <arg name="z_pos" default="(your start_orientation)"/>

NOTE: Be absolutely certain that the arguments entered while running the 2D implementation are the same as the values entered in the launch file. 