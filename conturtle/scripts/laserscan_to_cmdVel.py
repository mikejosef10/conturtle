#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
from math import cos, sin

# Global variables to store the closest point coordinates
closest_x = 0.0
closest_y = 0.0

def scan_callback(data):
    global closest_x, closest_y
    ranges = data.ranges
    
    # Filter out NaN values
    valid_ranges = [r for r in ranges if not math.isnan(r)]
    
    # Find the index of the closest point
    min_index = ranges.index(min(valid_ranges))

    # Calculate the x and y coordinates of the closest point
    closest_range = ranges[min_index]
    angle = data.angle_min + min_index * data.angle_increment
    closest_x = closest_range * cos(angle)
    closest_y = closest_range * sin(angle)

def publish_closest_point(pub):
    global closest_x, closest_y
    
    # Print closest points to log
    #rospy.loginfo("Closest point (x, y): ({}, {})".format(closest_x, closest_x))

    # Create a Twist message
    twist_msg = Twist()
    
    # Adjust x and y value and if the closest point is to far just stopt the turtle
    if closest_x<1:
        x_speed=max(0,-10*(closest_x-0.9)) # dont allow negativ values
        y_speed= -10*closest_y
    else:
        x_speed=0.0
        y_speed=0.0

    # Set linear and angular velocities
    twist_msg.linear.x = x_speed  # Set the forward/backward velocity
    twist_msg.angular.z = y_speed  # Set the left/right turn velocity

    # Publish the message on the /cmd_vel topic
    pub.publish(twist_msg)
    
    # Print Speed to log
    rospy.loginfo("Closest point (x, y): ({}, {})".format(x_speed, y_speed))
    
    closest_x = 0.0
    closest_y = 0.0

def listener():
    rospy.init_node('scan_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Initialize publisher
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_closest_point(pub)
        rate.sleep()

if __name__ == '__main__':
    listener()
