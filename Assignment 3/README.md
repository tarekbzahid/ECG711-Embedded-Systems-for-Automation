## Assignment 3

## Tarek Z

### Objectives

The objectives of this assignment are to:

1. Update the URDF file to accurately represent the actual robot.
2. Visualize the robot's motion in Rviz.
3. Simulate the robot's movement using keyboard commands.

### Resources

The necessary resources for this assignment can be found in the following repositories:

- Code: [https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/catkin_ws](https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/catkin_ws)
- Images: [https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/images](https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/images)
- Youtube: [https://www.youtube.com/playlist?list=PL-m3G6XXLHQtwv6o8FqcqjEjRjHKm8WNG](https://www.youtube.com/playlist?list=PL-m3G6XXLHQtwv6o8FqcqjEjRjHKm8WNG)

## Code Description

### Updated URDF File

The URDF file has been modified to incorporate the following changes:

- Addition of a lidar module on top of the robot frame to replicate the actual robot's configuration.
- Reduction of the frame thickness to enhance the visual representation of the robot.
- The header obeject almost closely represents out battery pack as we placed is behind the lidar towards the tail of the robot. So its not removed. 

The lidar module for the URDF file was obtained from Yahboom and is named laser_link.STL.

## Running the Robot

To operate the simulated robot, follow these steps:

1. Execute the `catkin_make` command.
2. Place the provided initial files in the `scr` folder.
3. Implement the necessary modifications to the URDF file.
4. Launch the robot using the `roslaunch ros_mobile_robot drive_robot.launch` command.
5. Control the robot's movement using the `rostopic pub` command:

```bash
rostopic pub -r 10 /robot_diff_drive_controller/cmd_vel geometry_msgs/Twist "linear:
 x: 1.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"
```

This command publishes a `geometry_msgs/Twist` message to the `/robot_diff_drive_controller/cmd_vel` topic, instructing the robot to move forward at a constant speed without rotating. The publication rate is set to 10 messages per second to ensure consistent velocity commands.

## Recreation Steps

To recreate the simulation environment, follow these steps:

1. Clone the provided repository: [https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/catkin_ws](https://github.com/tarekbzahid/ECG711/tree/main/Assignment%203/catkin_ws)
2. Execute the `catkin_make` command.
3. Modify the URDF file to incorporate the desired changes.
4. Launch the robot using the `roslaunch ros_mobile_robot drive_robot.launch` command.
5. Control the robot's movement using the `rostopic pub` command as described earlier.