#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from random import randint
import time


def main():
    # Initialize the ROS system and become a node.
    rospy.init_node("driver")
    #nh = rospy.NodeHandle()

    # Create a publisher object.
    pub = rospy.Publisher("goal_pose_driver", Pose2D, queue_size=1)

    naptime = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        pose = Pose2D()
        pose.x = 10
        pose.y = 10
        pose.theta = 0.20
        rospy.loginfo("Target Pose: x = %f, y = %f, theta = %f rad", pose.x, pose.y, pose.theta)
        pub.publish(pose)
        naptime.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
