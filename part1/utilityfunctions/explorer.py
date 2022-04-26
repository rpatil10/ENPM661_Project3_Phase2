import cv2
import numpy as np
from math import sqrt
from queue import PriorityQueue
from utilityfunctions import constants
from utilityfunctions.node import Node

# Function to check if given point lies within any obstacle
def is_node_valid(check_img, x, y):
    # Check condition for out of bounds
    if x <= 0 or x >= constants.map_size[1] or y <= 0 or y >= constants.map_size[0]:
        return False
    # Check condition to verify if point lies within obstacle
    elif check_img[y, x].all() == 0:
        return False
    return True

# Function to obtain euclidean distance between given two nodes/points
def obtain_euclidean_distance(first_node, second_node):
    return sqrt((second_node[0] - first_node[0]) ** 2 + (second_node[1] - first_node[1]) ** 2)

class Explorer:
    def __init__(self, start_node, goal_node, robot_rpm, map_img, animation):
        # Start and goal nodes
        self.start_node = Node(start_node, None, 0, 0, None)
        self.goal_node = goal_node
        # Animation
        self.animation = animation
        # RPMs of robot
        self.robot_rpm = robot_rpm
        # Original map
        self.map_img = map_img[0]
        # Extended map due to robot radius and clearance
        self.check_img = map_img[1]
        # Angular step size and translation step size
        self.step_theta = constants.angular_step_size
        # Map dimensions
        self.map_size = constants.map_size[0], constants.map_size[1], (constants.total_angle // self.step_theta)
        # Empty list to store all generated nodes and path nodes
        self.closed_nodes = []
        self.path_nodes = []
        # 3-D arrays to store information about generated nodes and parent nodes
        self.parent = np.full(fill_value=constants.no_parent, shape=self.map_size)
        # Video-writer of open-cv to record the exploration and final path
        video_format = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
        self.video_output = cv2.VideoWriter('2D_A_star_turtlebot.avi', video_format, 200.0, (self.map_size[1], self.map_size[0]))

    # Function to obtain euclidean distance heuristic - cost to goal
    def obtain_heuristic_score(self, node):
        return obtain_euclidean_distance(self.goal_node, node)

    # Function to add cost-to-goal and cost-to-come to get final cost
    def obtain_final_weight(self, node, base_cost):
        return self.obtain_heuristic_score(node) + base_cost

    # Function to convert theta from radians to a grid world index based on pre-defined angular step
    def obtain_orientation_index(self, theta):
        # Limit orientation between 0 to 360
        if theta >= 2 * np.pi:
            n = int(theta / (2 * np.pi))
            theta = theta - n * 2 * np.pi
        elif theta < 0:
            # Convert negative angle into positive and maintain the orientation between 0 to 360
            theta = abs(theta)
            if theta > 2 * np.pi:
                n = int(theta / (2 * np.pi))
                theta = theta - n * 2 * np.pi
        # Get orientation in degrees
        theta = theta + (180 * theta / np.pi)
        # Limit orientation between 0 to 360
        if theta > constants.total_angle:
            theta = theta - constants.total_angle
        # Return index by dividing orientation in degrees by angular step
        return int(theta / self.step_theta)

    # Function to defing action space
    def action_set(self, action):
        if action == 0:
            return 0, self.robot_rpm[0]
        elif action == 1:
            return 0, self.robot_rpm[1]
        elif action == 2:
            return self.robot_rpm[0], 0
        elif action == 3:
            return self.robot_rpm[1], 0
        elif action == 4:
            return self.robot_rpm[0], self.robot_rpm[0]
        elif action == 5:
            return self.robot_rpm[1], self.robot_rpm[1]
        elif action == 6:
            return self.robot_rpm[0], self.robot_rpm[1]
        return self.robot_rpm[1], self.robot_rpm[0]

    # Function to call various actions based on an integer and get new child coordinates and orientation
    def perform_action(self, parent_node, action):
        rpm = self.action_set(action)
        # Convert rpm into left and right wheel velocities respectively
        left_wheel_velocity = rpm[0] * (2 * np.pi / 60)
        right_wheel_velocity = rpm[1] * (2 * np.pi / 60)
        return self.fetch_child_node(parent_node, left_wheel_velocity, right_wheel_velocity)

    # Function to explore and find the goal
    def exploration(self):
        node_queue = PriorityQueue()
        start_node = self.start_node.get_node_data()
        self.parent[int(start_node[0])][int(start_node[1])][int(start_node[2])] = constants.start_parent
        node_queue.put(self.start_node)
        # Start exploring
        while not node_queue.empty():
            # Get node with minimum total cost
            current_node = node_queue.get()
            self.closed_nodes.append(current_node)
            # Add node to generated nodes array
            # Check for goal node
            if (self.obtain_heuristic_score(current_node.get_node_data()) <= constants.goal_threshold or current_node.get_node_data() == self.goal_node):
                self.path_nodes.append(current_node)
                break
            # Generate child nodes from current node
            for i in range(constants.maximum_actions):
                child_node = self.perform_action(current_node, i)
                if child_node is not None:
                    # Add child node to priority queue
                    node_queue.put(child_node)

    # Function to get child node based on wheel velocities and show exploration by generating intermediate nodes
    def fetch_child_node(self, parent_node, left_velocity, right_velocity):
        # An empty list to store intermediate nodes
        intermediate_nodes = []
        valid_path = True
        # Define grey
        yellow = [255, 255, 0]
        # Get coordinates  and orientation of parent node
        parent_node_data = parent_node.get_node_data()
        y, x = parent_node_data[0], parent_node_data[1]
        theta = np.pi * parent_node_data[2] * self.step_theta / 180
        prev_y, prev_x, prev_theta = y, x, parent_node_data[2]
        # Get linear and angular velocities in m/s and rad/s respectively
        linear_x = 0.5 * constants.wheel_radius * (left_velocity + right_velocity) * np.cos(theta) / constants.scaling_factor
        linear_y = 0.5 * constants.wheel_radius * (left_velocity + right_velocity) * np.sin(theta) / constants.scaling_factor
        linear_velocity = sqrt((linear_x ** 2) + (linear_y ** 2))
        angular_velocity = (constants.wheel_radius / constants.wheel_distance) * (right_velocity - left_velocity)
        t = 0
        while t < constants.total_time:
            # Increment time
            t += constants.time_step_size
            # Get new  coordinates and orientation using time step
            x += (0.5 * constants.wheel_radius * (left_velocity + right_velocity) * np.cos(theta) * constants.time_step_size * constants.time_scaling)
            y += (0.5 * constants.wheel_radius * (left_velocity + right_velocity) * np.sin(theta) * constants.time_step_size * constants.time_scaling)
            theta += ((constants.wheel_radius / constants.wheel_distance) * (right_velocity - left_velocity) * constants.time_step_size * constants.time_scaling)
            # Get index of current orientation in grid world to check for parent
            theta_index = self.obtain_orientation_index(theta)
            # Check for validity of intermediate node
            if (is_node_valid(self.check_img, int(x), self.map_size[0] - int(y)) and self.parent[int(y)][int(x)][theta_index] == constants.no_parent):
                # Add intermediate to its list
                intermediate_nodes.append(([prev_y, prev_x, prev_theta], [y, x, theta], [linear_velocity, angular_velocity]))
                # Update previous coordinates and orientation
                prev_x, prev_y, prev_theta = x, y, theta
            else:
                valid_path = False
                break
        # Discard entire path if any intermediate point lies within obstacle space
        if valid_path:
            last_node = None
            # Define base-cost of the child node
            base_cost = parent_node.get_base_weight()
            len_intermediate_nodes = len(intermediate_nodes)
            # Add exploration to video
            for i in range(len_intermediate_nodes):
                prev_node, current_node, _ = intermediate_nodes[i]
                # Update base-cost of the node
                base_cost += obtain_euclidean_distance(prev_node, current_node)
                # Get index of orientation of intermediate node
                prev_node[2] = self.obtain_orientation_index(prev_node[2])
                current_node[2] = self.obtain_orientation_index(current_node[2])
                # Update parent of the intermediate node
                self.parent[int(current_node[0])][int(current_node[1])][int(current_node[2])] = np.ravel_multi_index([int(prev_node[0]), int(prev_node[1]), int(prev_node[2])], dims=self.map_size)
                # Add nodes to exploration video
                if self.animation:
                    cv2.arrowedLine(self.map_img, (int(prev_node[1]), self.map_size[0] - int(prev_node[0])), (int(current_node[1]), self.map_size[0] - int(current_node[0])), yellow)
                    self.video_output.write(self.map_img)
                # Make last node in the list as the child node and create is node class object
                if i == len_intermediate_nodes - 1:
                    last_node = Node(current_node, parent_node_data, float('inf'), float('inf'), intermediate_nodes)
                    last_node.set_base_weight(base_cost)
                    last_node.set_weight(self.obtain_final_weight(current_node, last_node.base_weight))
            return last_node
        return None

    # Function to generate path using back-tracking
    def path_generation(self):
        # Exit if exploration failed
        if not len(self.path_nodes):
            print('No path found')
            return False
        # Define colors
        red = [0, 0, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
        # Open text file to write velocities
        vel_txt = open('path_files/velocities.txt', 'w+')
        # Get first node nearest to the goal node to start backtracking
        last_node = self.path_nodes[0]
        print('Finding path...')
        # Iterate until we reach the initial node
        while last_node.get_node_data() != self.start_node.get_node_data():
            for node in self.closed_nodes:
                if node.get_node_data() == last_node.get_parent_node():
                    self.path_nodes.append(node)
                    last_node = node
                    break
        print('Path found')
        # Iterate through path nodes
        for i in range(len(self.path_nodes) - 2, -1, -1):
            # Get intermediate nodes of each node in path-nodes' list
            current_sub_nodes = self.path_nodes[i].get_sub_nodes()
            # Iterate through intermediate nodes' list to display path to be taken by the robot
            for j in range(0, len(current_sub_nodes)):
                current_node_data = current_sub_nodes[j]
                vel_txt.write(str(current_node_data[2][0]) + ',' + str(current_node_data[2][1]) + '\n')
                if self.animation:
                    cv2.line(self.map_img,(int(current_node_data[0][1]), self.map_size[0] - int(current_node_data[0][0])),(int(current_node_data[1][1]), self.map_size[0] - int(current_node_data[1][0])),blue)
                    self.video_output.write(self.map_img)
        vel_txt.write(str(0.0) + ',' + str(0.0) + '\n')
        if self.animation:
            # Draw start and goal node to the video frame in the form of filled circle
            cv2.circle(self.map_img,(int(self.path_nodes[-1].data[1]),self.map_size[0] - int(self.path_nodes[-1].data[0])),int(constants.radius_of_robot), red, -1)
            cv2.circle(self.map_img,(int(self.path_nodes[0].data[1]), self.map_size[0] - int(self.path_nodes[0].data[0])),int(constants.radius_of_robot), green, -1)
            # Show path for longer time
            for _ in range(1000):
                self.video_output.write(self.map_img)
        return True