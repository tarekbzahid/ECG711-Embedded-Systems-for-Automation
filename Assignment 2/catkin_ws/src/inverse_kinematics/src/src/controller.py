#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from math import sqrt, atan2, cos, sin

# Global variables
current_pose = Pose2D()
goal_pose = Pose2D()
past_time = None

distance_tolerance = 0.5  # Position error tolerance
angular_tolerance = 0.5  # Angle error tolerance

publisher_cmd_vel = None
publisher_current_pose = None
pose = None

def pose_callback(pose_msg):
    global goal_pose
    goal_pose = pose_msg
    rospy.loginfo("Received goal_pose from Driver node: x = %f, y = %f, theta = %f", goal_pose.x, goal_pose.y, goal_pose.theta)

def get_distance():
    return sqrt((current_pose.x - goal_pose.x) ** 2 + (current_pose.y - goal_pose.y) ** 2)

def get_angle():
    return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x) - current_pose.theta

def move_goal():
    global current_pose
    global past_time

    # Define PID constants (unchanged from the previous code)
    kp_position = 7.0  # Proportional gain for position
    ki_position = 0.1  # Integral gain for position
    kd_position = 0.01  # Derivative gain for position
    kp_angle = 1.0  # Proportional gain for angle

    # Initialize PID error terms
    integral_position = 0.0
    previous_position_error = 0.0
    previous_angle_error = 0.0

    # Calculate time elapsed since the last update
    current_time = rospy.Time.now()
    dt = (current_time - past_time).to_sec()

    # Calculate position error
    position_error = get_distance()

    # Calculate angle error
    angle_error = get_angle()

    # Update integral error terms
    integral_position += position_error * dt

    # Calculate control inputs using PID
    linear_velocity = kp_position * position_error + ki_position * integral_position - kd_position * (position_error - previous_position_error)
    angular_velocity = kp_angle * angle_error

    # Update current pose based on control inputs
    current_pose.x += linear_velocity * cos(current_pose.theta) * dt
    current_pose.y += linear_velocity * sin(current_pose.theta) * dt
    current_pose.theta += angular_velocity * dt

    # Update past_time
    past_time = current_time

    # Log current and target positions
    rospy.loginfo("Current Position: x = %f, y = %f, theta = %f", current_pose.x, current_pose.y, current_pose.theta)
    rospy.loginfo("Target Position: x = %f, y = %f, theta = %f", goal_pose.x, goal_pose.y, goal_pose.theta)

    # Check if position and angle errors are within tolerance
    if position_error < distance_tolerance and abs(angle_error) < angular_tolerance:
        rospy.loginfo("Reached goal position and angle")
    else:
        rospy.loginfo("Moving towards goal")

def main():
    global publisher_cmd_vel
    global publisher_current_pose
    global pose
    global past_time

    rospy.init_node("controller")
    pose = rospy.Subscriber("goal_pose_driver", Pose2D, pose_callback)
    publisher_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
    publisher_current_pose = rospy.Publisher("goal_pose_controller", Pose2D, queue_size=1000)

    past_time = rospy.Time.now()
    rate = rospy.Rate(0.5)  # 0.5 Hz (2 seconds per cycle)

    while not rospy.is_shutdown():
        move_goal()

        # Publish the current pose to "goal_pose_controller" topic
        publisher_current_pose.publish(current_pose)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
