import ast
from sys import argv
from time import time
from utilityfunctions.obstacle_space import Map
from utilityfunctions.explorer import Explorer, is_node_valid

###  start_node:x,start_node:y,start_node:theta <space> goal_node:x,goal_node:y <space> robot_parameter:RPM,robot_parameter:RPM,clearance_value
script, start_node, goal_node, robot_parameter = argv
# Transform input arguments into tuples
robot_parameter = tuple(ast.literal_eval(robot_parameter))
start_node = tuple(ast.literal_eval(start_node))
goal_node = tuple(ast.literal_eval(goal_node))
# Instantiate the class map and get map to check for obstacles
obstacle_map_space = Map(robot_parameter[2])
check_image = obstacle_map_space.check_img
# Transform start and goal nodes given by user input into coordinates from map frame
start_node = obstacle_map_space.get_position_inside_map((start_node[0], start_node[1]), start_node[2])
goal_node = obstacle_map_space.get_position_inside_map(goal_node)
# Check viability/validity of start and goal nodes
if not (is_node_valid(check_image, start_node[1], obstacle_map_space.height - start_node[0]) and is_node_valid(check_image, goal_node[1], obstacle_map_space.height - goal_node[0])):
    print('Given point/points lie inside obstacle space!!\nPlease run again')
    quit()
# Instantiate the explorer class to find the goal node
explorer_object = Explorer(start_node, goal_node, (robot_parameter[0], robot_parameter[1]),(obstacle_map_space.map_img, check_image), int(1))
start_time = time()
# Begin node exploration
explorer_object.exploration()
print('Time required for exploration:', time() - start_time)
start_time = time()
explorer_object.path_generation()
print('Time required for path Generation:', time() - start_time)