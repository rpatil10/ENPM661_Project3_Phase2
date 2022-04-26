# Minimum distance threshold in map
distance_threshold = 0.01
scaling_factor = int(1 / distance_threshold)
# Map size
width, height = 10, 10
map_size = (scaling_factor * height), (scaling_factor * width)
map_center_point = (map_size[0] // 2), (map_size[1] // 2)
# Robot parameters in centimeter
diameter_of_robot = 21
radius_of_robot = diameter_of_robot / 2
wheel_distance = 16
wheel_radius = 3.3
# Threshold around goal
goal_threshold = radius_of_robot
# Maximum number of possible actions
maximum_actions = 8
# Robot moves by 1cm translation-step
# Orientation of the robot is a multiple of angular-step
angular_step_size = 20
# Time between each movement and total time for each action
time_step_size = 1
total_time = 10
time_scaling = 0.5
# Total angle of a complete circle
total_angle = 360
# Exploration constants
no_parent = -1
node_generated = 1
start_parent = -99