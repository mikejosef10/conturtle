#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from math import cos, sin

# Global variables to store the closest point of laserscan coordinates
LASERSCAN_X = 0.0
LASERSCAN_Y = 0.0

# Global variables to store the closest point of laserscan coordinates
CURRENT_POSE_X = 0.0
CURRENT_POSE_Y = 0.0
CURRENT_POSE_THETA = 0.0

def scan_callback(data):
    global LASERSCAN_X, LASERSCAN_Y
    ranges = data.ranges
    
    # Filter out NaN values
    valid_ranges = [r for r in ranges if not math.isnan(r)]
    
    # Find the index of the closest point
    min_index = ranges.index(min(valid_ranges))

    # Calculate the x and y coordinates of the closest point
    closest_range = ranges[min_index]
    angle = data.angle_min + min_index * data.angle_increment
    LASERSCAN_X = closest_range * cos(angle)
    LASERSCAN_Y = closest_range * sin(angle)

def pose_callback(data):
    global CURRENT_POSE_X, CURRENT_POSE_Y, CURRENT_POSE_THETA
    
    CURRENT_POSE_X = data.x
    CURRENT_POSE_Y = data.y
    CURRENT_POSE_THETA = data.theta

def publish_closest_point(pub):
    global LASERSCAN_X, LASERSCAN_Y, CURRENT_POSE_X, CURRENT_POSE_Y, CURRENT_POSE_THETA
    
    # The center of the turtle seems to be 5.5, 5.5
    current_pose_x = CURRENT_POSE_X - 5.5
    current_pose_y = CURRENT_POSE_Y - 5.5
    
    # The laserscan starts at 0.5 m and should just be used for 0.2m, 
    # so that the center is at 0.55 m. 
    # The y component can be left as it is.
    laserscan_x = (LASERSCAN_X-0.55) 
    laserscan_y = (LASERSCAN_Y)
    
    # Adjust x and y value and if the closest point is to far just stopt the turtle
    if laserscan_x<-0.05 or laserscan_x>0.05:
        control_input_x=0.0
        control_input_y=0.0
        rospy.loginfo("Stopped!")
    else:
        
        #Make values extremer to need less movement
        laserscan_x = laserscan_x*100
        laserscan_y = laserscan_y*50
        
        # Constants for PID controller
        KP = 0.5  # Proportional gain
        KI = 0.1  # Integral gain
        KD = 0.2  # Derivative gain

        # Initialize variables
        previous_error_x = 0
        previous_error_y = 0
        integral_x = 0
        integral_y = 0
        


        # Calculate the error - x and y are turned because the are defined in that way
        error_x = laserscan_y - current_pose_x
        error_y = -laserscan_x - current_pose_y

        # Calculate the PID components for each axis
        proportional_x = KP * error_x
        integral_x += KI * error_x
        derivative_x = KD * (error_x - previous_error_x)

        proportional_y = KP * error_y
        integral_y += KI * error_y
        derivative_y = KD * (error_y - previous_error_y)

        # Calculate the control inputs for each axis
        control_input_x = proportional_x + integral_x + derivative_x
        control_input_y = proportional_y + integral_y + derivative_y

        # Update the previous errors
        previous_error_x = error_x
        previous_error_y = error_y

    # Create and publish the Twist message
    twist = Twist()
    twist.linear.x = control_input_x
    twist.linear.y = control_input_y
    twist.angular.z = 0.0  # Assuming no angular control needed
    pub.publish(twist)
    
    # Print Poses to log
    rospy.loginfo("Current pose (x, y): ({}, {})".format(current_pose_x, current_pose_y))
    rospy.loginfo("Wished pose (x, y): ({}, {})".format(laserscan_x, laserscan_y))
    # Print Speed to log
    rospy.loginfo("Current speed (x, y): ({}, {})".format(control_input_x, control_input_y))
    
    #LASERSCAN_X = 0.0
    #LASERSCAN_Y = 0.0

def listener():
    rospy.init_node('scan_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/turtle1/pose', Pose,pose_callback)

    # Initialize publisher
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_closest_point(pub)
        rate.sleep()

if __name__ == '__main__':
    listener()
