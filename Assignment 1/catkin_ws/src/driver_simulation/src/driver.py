#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("driver_node")  # Initialize the ROS node
    cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(1)  # cmd_vel publishing rate in Hz

    while not rospy.is_shutdown():
        # Create a Twist message and set the linear velocities
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1.0  # Left velocity
        cmd_vel_msg.linear.y= 2.0   # Right velocity

        # Publish the cmd_vel message
        cmd_vel_publisher.publish(cmd_vel_msg)

        rospy.loginfo("Driver: Published cmd_vel message: left velocity=%f, right velocity=%f",
                      cmd_vel_msg.linear.x, cmd_vel_msg.linear.y)

        rate.sleep()

if __name__ == '__main__':
    main()
