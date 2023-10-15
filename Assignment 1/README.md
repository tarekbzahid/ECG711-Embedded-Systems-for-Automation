# Assignment 1 - Tarek Z

##  Assignment Summary

We are tasked with creating two nodes: a "driver" node, which serves as a command center transmitting differential wheel velocity commands to a "simulator" node, representing a robot in a field. The "driver" node publishes a single topic, "cmd_vel," containing linear velocity data for both wheels at a specified frequency. The "simulator" node subscribes to "cmd_vel," extracts differential velocity information, performs calculations based on this input, and publishes the resulting final position on another topic.

The "driver_node" is implemented with a ROS node that initializes a publisher for the "cmd_vel" topic. This publisher sends Twist messages with linear velocity information for the left and right wheels at a rate of 1 Hz until the node is shut down. 

The "simulator_node" simulates robot movement based on velocity commands from the "cmd_vel" topic. The subscriber node subscribe to the cmd_vel, and extracts the diiferential wheel velocities. If the velocities are equal the robot goes in straight line, if not then it rotates around a center point. Both the linear and circular velocities are function of time. The time difference is used to calculate how much time has elapsed since the last velocity command was published which allows the code to estimate the change in the robot's position and orientation over time. Finally the other topic publishes the final x,y and orientation as Pose2D message on the "pose_2d" topic. 

I have compelted the code using python3 and ros dependecies. I tired using ros.time but it was causing an error. So i used the built in python time.time library for time calculation. 


Code: https://github.com/tarekbzahid/ECG711/tree/main/Assignment%201/catkin_ws

Images: https://github.com/tarekbzahid/ECG711/tree/main/Assignment%201/images 

## Code Description 

### The Driver node
    Shebang Line: The script starts with a shebang line specifying the interpreter to be used, which is Python 3.

    Import Libraries: The necessary Python libraries are imported:
        rospy: The ROS Python library for working with ROS.
        geometry_msgs.msg.Twist: A message type from ROS that represents linear and angular velocities.

    Initialize ROS Node:
        rospy.init_node("driver_node"): This line initializes a ROS node named "driver_node." It is the starting point for a ROS program.

    Create Publisher:
        cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10): This line creates a publisher for the "cmd_vel" topic with a message type of Twist. The queue_size parameter specifies the maximum number of messages to store in the publisher's queue.

    Set Publishing Rate:
        rate = rospy.Rate(1): This line sets the publishing rate for the "cmd_vel" topic to 1 Hz. The code will publish messages at this rate.

    While Loop:
        The code enters a while loop using while not rospy.is_shutdown():. This loop will continue until the ROS node is shut down.

    Create Twist Message:
        cmd_vel_msg = Twist(): A new Twist message is created to hold linear velocity information.

    Set Linear Velocities:
        cmd_vel_msg.linear.x = 1.0: The left wheel's linear velocity is set to 1.0.
        cmd_vel_msg.linear.y = 2.0: The right wheel's linear velocity is set to 2.0.

    Publish Message:
        cmd_vel_publisher.publish(cmd_vel_msg): The code publishes the cmd_vel_msg on the "cmd_vel" topic.

    Log Information:
        rospy.loginfo(...): The code logs information about the published message, including the left and right wheel velocities.

    Control Publishing Rate:
        rate.sleep(): This line controls the publishing rate, making sure the loop runs at the specified rate (1 Hz).

    Main Function Call:
        if __name__ == '__main__': main(): The script's main function is called when the script is executed as the main program. The main() function initializes the ROS node and enters the main loop. The rospy.spin() call keeps the node running.


### The Simulator node
    Shebang Line: The script starts with a shebang line specifying the interpreter to be used, which is Python 3.

    Import Libraries: The necessary Python libraries are imported:
        rospy: The ROS Python library for working with ROS.
        numpy (np alias): NumPy is used for mathematical operations.
        time: The standard Python library for time-related functions.
        geometry_msgs.msg.Twist and Pose2D: ROS message types for representing velocity and pose, respectively.

    Initialize Parameters: Several variables are initialized to store information about the robot's pose, such as its position (x, y), axel length (L), heading (theta), and timestamps for timing calculations.

    Callback Function for cmd_vel:
        The cmd_vel_callback function processes messages received on the "cmd_vel" topic.
        It updates the robot's position and heading based on the differential wheel velocities received in the message.
        If both the wheel velocities are same, the robot moves in a straight line, with no change in theta. If they differ, the robot undergoes rotation there is a change in x, y and theta. I have checked it for further confirmation. 
        The calculation done for getting the final position of the robot is based on notes for the lecture 6.
        When
        The updated pose is published as a Pose2D message.
        Information is logged for monitoring.

    Main Function:
        main is the main function of the script.
        It initializes a ROS node named "simulator_node."
        It creates a publisher for the "pose_2d" topic, a subscriber for the "cmd_vel" topic, and sets the operating rate at 1 Hz.

    While Loop:
        The code enters a while loop using while not rospy.is_shutdown():.
        Within the loop, the current time is updated, and the rate is controlled to maintain the specified rate.

    Main Function Call:
        if __name__ == '__main__': main(): The script's main function is called when the script is executed as the main program. The main() function initializes the ROS node, sets up publishing and subscribing, and enters the main loop. The rospy.spin() call keeps the node running.

## Instruction to recreate the program

### Initialize Catkin Workspace
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

### Create a New Package
    catkin_create_pkg driver_simulation rospy std_msgs

### Add Python Scripts
    cd ~/catkin_ws/src/driver_simulation/src
    touch driver.py
    touch simulator.py
    chmod +x driver.py
    chmod +x simulator.py

### Write code in the node files. The codes are included in the assignment 1 folder.    

### Create a .launch file in the launch folder. 
    <launch>
        <node name="driver_node" pkg="driver_simulation" type="driver.py" output="screen"/>
        <node name="simulator_node" pkg="driver_simulation" type="simulator.py" output="screen"/>
    </launch>

### Build package
    cd ~/catkin_ws
    catkin_make

### Source workspace
    source ~/catkin_ws/devel/setup.bash

### Launch the ros nodes using the launch file
    roslaunch driver_simulation driver_simulation.launch