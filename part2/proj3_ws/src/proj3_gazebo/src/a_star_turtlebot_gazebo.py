#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import rospkg


# function to convert file into array
def file_to_array(file):
    for i in range(len(file)):
        file[i] = file[i].split(',')

    return file


# function to run turtlebot3
def turtlebot_mover(order):
    end_flag=False
    # declaring object for class Twist
    msg = Twist()

    for i in range(len(order)):
        rate = rospy.Rate(2)

        msg.linear.x = float(order[i][0])
        msg.angular.z = float(order[i][1])

        rospy.loginfo(msg)
        pub.publish(msg)
        if msg.linear.x==0 and msg.angular.z==0:
            end_flag=True
            break
        rate.sleep()
    return end_flag


if __name__ == '__main__':

    global pub

    # publisher object publishing to command velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # initialize node
    rospy.init_node('planner', anonymous=True)

    # read text file
    ros_root = rospkg.RosPack()
    with open(ros_root.get_path('proj3_gazebo') + '/path_files/velocities.txt', 'r') as command:
        orders = command.readlines()

    # convert into array
    orders = file_to_array(orders)
    while True:
        
        flag=turtlebot_mover(orders)
        if flag:
            break

   
