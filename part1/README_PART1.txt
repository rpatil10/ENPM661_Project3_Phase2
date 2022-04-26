#ENPM 661 Project 3 Phase 2- Part1
Anish Mitra (amitra12@umd.edu)
Rohit Patil (rpatil10@umd.edu)

This README file contains the instructions for executing A-Star algorithm for TurtleBot using ROS in 2D Space. 

File Structure

part1
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
|
|


'map.png' is the image illustrating the obstacle space to perform A_star path planning algorithm.

turtlebot_A_star.py is the program that is used to traverse the map for a given start and end goal using A* path planning. To run the program, traverse to the location containing the program. Make sure that the utilityfunctions folder is present in the same location as the program. 

turtlebot_A_star.py takes in the following arguments: start_x_location,start_y_location,start_orientation goal_x_location,goal_y_location rpm1,rpm2,clearance

Testcase 1:
python3 turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5

Testcase 2:
python3 turtlebot_A_star.py -4,-4.5,0 4.25,2.75 30,25,5




