# Assignment 2 - Tarek Z

##  Assignment Summary

There will be two nodes: 

### the driver node -> publish a target desitination as Pose2D as goal_pose topic
### the controller node -> will try to reach to the destination position while publishing its current pose and differential speed

The controller node will calculate the error which the difference current linear and angular position and the destination linear and angular position. It will try to minimize the error using the PID controller. PID controller is designed to minimize the error without too much moving back and forth. We initially set the Kp , Ki, Kd values as constants. 
Its a feedback loop, the new position found using the PID is again set as current position and compared with the target position. This continues until the error comes within the tolerance level of linear and angualr position. 

Code: https://github.com/tarekbzahid/ECG711/tree/main/Assignment%202/catkin_ws

Images: https://github.com/tarekbzahid/ECG711/tree/main/Assignment%202/images 

## Code Description 

### The Driver node

1. It starts with a shebang line `#!/usr/bin/env python3` to specify that the script should be executed using Python 3.

2. It imports necessary libraries/modules, including `rospy` for ROS functionality, message types from `geometry_msgs`, `randint` for generating random numbers, and `time`.

3. Inside the `main()` function:
   - It initializes a ROS node named "driver."
   - Creates a publisher object that publishes `Pose2D` messages to the "goal_pose_driver" topic with a queue size of 1.
   - Sets the publishing rate to 0.5 Hz (every 2 seconds).
   - In a loop that runs as long as the ROS system is not shut down, it creates a `Pose2D` message with x, y, and theta values and publishes it to the topic. It also logs the target pose using `rospy.loginfo`.
   - It uses `naptime.sleep()` to control the loop rate.

4. Finally, it runs the `main()` function if this script is the main module, and it catches a `rospy.ROSInterruptException` if an interuuption occurs.

### The Controller node

1. The script starts with a shebang line specifying Python 3 for execution.

2. It imports essential ROS libraries (`rospy`) and modules for mathematical calculations (`math`) and message types (`geometry_msgs`).

3. Global variables are defined to store the current and goal poses, as well as tolerance values for position and angle errors. Publishers are created to send velocity commands and the current robot pose to other nodes.

4. A callback function (`pose_callback`) is defined to update the goal pose when new goal information is received.

5. Two utility functions, `get_distance()` and `get_angle()`, are provided to calculate the distance and angle errors between the current and goal poses.

6. The `move_goal()` function implements a Proportional-Integral-Derivative (PID) controller to compute control inputs. The PID controller adjusts the robot's position and orientation based on the current and desired poses. It calculates control inputs to minimize errors, allowing the robot to move towards its goal while monitoring its progress and maintaining error tolerances. The PID constants and error terms are involved in these calculations. The logic of the PID controller is provided by the Dr. Venki.

7. The `main()` function is the entry point for the node. It initializes the ROS node, sets up subscribers and publishers for communication, and controls the robot's movement by repeatedly calling `move_goal()` at a rate of 0.5 Hz (every 2 seconds).

8. The script checks whether it is the main module and runs the `main()` function. It also handles a `rospy.ROSInterruptException` if it occurs, which typically happens when the ROS system is interrupted.

## Instruction to recreate the program

The packages and the files in a similar fashion to assignment 1. 

### To run the launch file:  

roslaunch inverse_kinematics inverse_kinematics.launch


