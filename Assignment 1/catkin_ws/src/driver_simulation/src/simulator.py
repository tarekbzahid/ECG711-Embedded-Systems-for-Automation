#!/usr/bin/env python3

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist, Pose2D

# Initial parameters:
x = 0
y = 0
L = 1
theta = 0
current_time = time.time()
publish_time = time.time()

def cmd_vel_callback(msg):
    global x, y, theta, current_time, publish_time

    l_vel = msg.linear.x
    r_vel = msg.linear.y

    t = (current_time - publish_time)

    if l_vel == r_vel:      # If both wheels have the same velocity -> no rotation 
        x = x + l_vel * t * np.cos(theta)
        y = y + l_vel * t * np.sin(theta)
    else:
        R = L / 2.0 * ((l_vel + r_vel) / (r_vel - l_vel))       # If wheels have differential veloctiy -> rotation
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)
        omega = (l_vel - r_vel) / L
        dtheta = omega * t
        x = np.cos(dtheta) * (x - ICC_x) - np.sin(dtheta) * (y - ICC_y) + ICC_x
        y = np.sin(dtheta) * (x - ICC_x) + np.cos(dtheta) * (y - ICC_y) + ICC_y
        theta = theta + dtheta

    pose_msg = Pose2D(x, y, theta)
    pose_publisher.publish(pose_msg)

    # Print message on terminal

    rospy.loginfo("Simulator: Received cmd_vel message: left velocity=%f, right velocity=%f", l_vel, r_vel)
    rospy.loginfo("Simulator: Published pose_2d message: x=%f, y=%f, theta=%f", x, y, theta)

    publish_time = current_time

def main():
    rospy.init_node("simulator_node")
    global pose_publisher, current_time
    pose_publisher = rospy.Publisher("pose_2d", Pose2D, queue_size=10)
    cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        current_time = time.time()
        rate.sleep()

if __name__ == '__main__':
    main()
    rospy.spin()
