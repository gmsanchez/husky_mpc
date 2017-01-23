#!/usr/bin/env python
"""
Simple example of subscribing to sensor messages and publishing
twist messages to the turtlebot.

Author: Nathan Sprague
Version: 1/12/2015

"""
import rospy
import math

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# globals
ODOM = None

# This function will be called every time a new scan message is
# published.
def odom_callback(odom_msg):
    # Save a global reference to the most recent sensor state so that
    # it can be accessed in the main control loop.
    # (The global keyword prevents the creation of a local variable here.)
    global ODOM
    ODOM = odom_msg

# This is the 'main'
def start():
    
    # Turn this into an official ROS node named approach
    rospy.init_node('approach', anonymous=True)

    # Subscribe to the /scan topic.  From now on
    # scan_callback will be called every time a new scan message is
    # published.
    rospy.Subscriber("husky_velocity_controller/odom", Odometry, odom_callback)
    # Create a publisher object for sending Twist messages to the
    # turtlebot_node's velocity topic. 
    vel_pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    # Create a twist object. 
    # twist.linear.x represents linear velocity in meters/s.
    # twist.angular.z represents rotational velocity in radians/s.
    twist = Twist()

    # Wait until the first scan is available.
    while ODOM is None and not rospy.is_shutdown():
        rospy.sleep(.1)

    # Try to maintain this target distance to the wall.
    target = .75

    # Rate object used to make the main loop execute at 10hz.
    rate = rospy.Rate(50) 

    while not rospy.is_shutdown():

        # Back up if the scan is bad, or if we are too close. 
        # if math.isnan(SCAN.ranges[320]) or SCAN.ranges[320] < target:
        #     twist.linear.x = -0.1  # forward velocity in meters/second
        #     twist.angular.z = 0   # rotation in radians/second
        # else:
        #     twist.linear.x = 0.1 
        #     twist.angular.z = 0  

        # print "Range: {:.3f}".format(SCAN.ranges[320])

        if ODOM.pose.pose.position.x<10:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0
            
        print ODOM.pose.pose.position.x

        vel_pub.publish(twist) # These velocities will be applied for .6 seconds
                               # unless another command is sent before that. 
        rate.sleep()           # Pause long enough to maintain correct rate.
        

# This is how we usually call the main method in Python. 
if __name__ == "__main__":
    start()