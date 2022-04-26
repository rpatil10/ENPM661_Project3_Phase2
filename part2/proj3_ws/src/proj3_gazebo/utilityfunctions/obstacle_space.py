import cv2
import numpy as np
from utilityfunctions import constants

class Map:
    # Function to initialize map class with radius of the robot and clearance
    def __init__(self, clearance):
        self.height = constants.map_size[0]
        self.width = constants.map_size[1]
        self.thresh = int(constants.radius_of_robot) + clearance
        self.scaling = constants.scaling_factor
        self.black = (0, 0, 0)
        # Coordinates of the top-left corner of the square obstacles
        self.square_coords = np.array([[25, 425], [175, 425], [175, 575], [25, 575]]), np.array([[375, 425], [625, 425], [625, 575], [375, 575]]), np.array([[725, self.height-200], [875, self.height-200], [875, self.height-400], [725, self.height-400]])
        # Radius of circular obstacles
        self.circle_radius = self.scaling * 1
        # Centers of all circles
        self.circle_centers = np.array([(200, 200),(200, self.height-200)],dtype=np.int32)
        # Empty world and add obstacles to it
        self.map_img = self.plot_obstacles()
        # Get image to search for obstacles
        self.check_img = self.copy_image()

    # Function to plot the 4 circular obstacles on the map-image
    def plot_circles(self, img, thresh=0):
        for center in self.circle_centers:
            cv2.circle(img, (center[0], center[1]), self.circle_radius + thresh, self.black, -1)

    # Function to plot the 3 square obstacles on the map
    def plot_squares(self, img, thresh=0):
        for pts in self.square_coords:
            cv2.fillPoly(img, np.int32([pts]), (0,0,0))

    # Function to get eroded image to check for obstacles considering the robot radius and clearance
    def copy_image(self):
        # Get map with obstacles
        eroded_img = self.map_img.copy()
        # Erode map image for rigid robot
        self.plot_squares(eroded_img, self.thresh)
        self.plot_circles(eroded_img, self.thresh)
        return eroded_img

    # Function to plot map using half-plane equations
    def plot_obstacles(self):
        self.map_img=None
        if self.map_img is None:
            self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            self.map_img.fill(255)
            self.plot_circles(self.map_img)
            self.plot_squares(self.map_img)
            cv2.imwrite('images/map.png', self.map_img)
        return self.map_img

    # Function to get position by converting world frame coordinates and orientation into map frame
    def get_position_inside_map(self, point, theta=0):
        x, y = point
        theta = theta // constants.angular_step_size
        # Scale the coordinates and convert them into map frame
        x, y = constants.map_center_point[1] + int(self.scaling * x), constants.map_center_point[1] + int(self.scaling * y)
        return list((y, x, theta))
