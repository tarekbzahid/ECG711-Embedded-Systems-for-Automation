# ECG711 - Embedded Systems for Automation

This repository contains coursework and assignments for ECG711 (Embedded Systems for Automation), focusing on Robot Operating System (ROS), robotics simulation, autonomous navigation, and embedded systems programming.

**📖 [Read the detailed ABOUT document](./ABOUT.md) for an in-depth look at the course, learning objectives, technical deep dives, and real-world applications.**

## Author

**Tarek Z**
Email: zahid@unlv.nevada.edu
YouTube Playlist: [ECG 711 Course Videos](https://www.youtube.com/playlist?list=PL-m3G6XXLHQtwv6o8FqcqjEjRjHKm8WNG)

## Repository Overview

This repository demonstrates practical applications of embedded systems concepts in robotics, including:

- ROS node development and inter-node communication
- Robot kinematics and differential drive control
- PID control systems for autonomous navigation
- URDF modeling and robot visualization
- Sensor integration (IMU, LiDAR)
- SLAM (Simultaneous Localization and Mapping)
- Hardware setup and calibration for real robots

## Technologies Used

- **ROS (Robot Operating System)** - Melodic
- **Python 3** - For ROS node implementation
- **Gazebo** - Robot simulation environment
- **RViz** - 3D visualization tool
- **Ubuntu 18.04** - Operating system
- **Jetson Nano** - Embedded computing platform
- **STM32** - Microcontroller for motor control
- **RPLidar A3** - 2D LiDAR sensor
- **Gmapping/Hector SLAM** - Mapping algorithms

## Assignments

### [Assignment 1: Differential Drive Simulation](./Assignment%201)

Implemented a driver-simulator system with two ROS nodes:
- **Driver Node**: Publishes differential wheel velocity commands (`cmd_vel` topic)
- **Simulator Node**: Calculates robot position based on velocity commands and publishes pose

**Key Concepts**: ROS topics, publishers/subscribers, differential kinematics, pose estimation

### [Assignment 2: PID Controller for Goal Navigation](./Assignment%202)

Developed an autonomous navigation system using PID control:
- **Driver Node**: Publishes target destination as `Pose2D`
- **Controller Node**: Uses PID controller to minimize position and orientation errors

**Key Concepts**: PID control, feedback loops, error minimization, autonomous navigation

### [Assignment 3: URDF Modeling and RViz Visualization](./Assignment%203)

Modified URDF files to accurately represent a physical robot:
- Added LiDAR module to robot model
- Improved visual representation
- Simulated robot movement in Gazebo and RViz

**Key Concepts**: URDF/XACRO, robot modeling, RViz visualization, Gazebo simulation

### [Assignment 4: Physical Robot Setup and Calibration](./Assignment%204)

Complete hardware setup and calibration of a physical robot:
- Hardware assembly (Jetson Nano, STM32, motors, LiDAR, batteries)
- Ubuntu and ROS installation on Jetson Nano
- IMU calibration, linear and angular velocity calibration
- LiDAR SLAM mapping with gmapping

**Key Concepts**: Embedded hardware, sensor calibration, real-world robotics, SLAM

## Getting Started

### Prerequisites

- Ubuntu 18.04 (or compatible)
- ROS Melodic
- Python 3
- Catkin build system

### Basic Setup

1. Clone this repository:
   ```bash
   git clone https://github.com/tarekbzahid/ECG711-Embedded-Systems-for-Automation.git
   cd ECG711-Embedded-Systems-for-Automation
   ```

2. Navigate to the specific assignment directory:
   ```bash
   cd "Assignment 1"  # or Assignment 2, 3, 4
   ```

3. Follow the instructions in each assignment's README for detailed setup and execution steps.

### Building a Catkin Workspace (General)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## Project Structure

```
ECG711-Embedded-Systems-for-Automation/
├── Assignment 1/          # Differential drive simulation
│   ├── catkin_ws/
│   ├── images/
│   └── README.md
├── Assignment 2/          # PID controller navigation
│   ├── catkin_ws/
│   ├── images/
│   └── README.md
├── Assignment 3/          # URDF modeling and visualization
│   ├── catkin_ws/
│   ├── images/
│   └── README.md
├── Assignment 4/          # Physical robot setup
│   ├── images/
│   └── README.md
├── ABOUT.md               # Detailed course documentation
└── README.md              # This file
```

## Resources

- **Course Videos**: [YouTube Playlist](https://www.youtube.com/playlist?list=PL-m3G6XXLHQtwv6o8FqcqjEjRjHKm8WNG)
- **ROS Documentation**: [http://wiki.ros.org/](http://wiki.ros.org/)
- **Yahboom ROS Controller**: [http://www.yahboom.net/study/ROSMASTER-X1](http://www.yahboom.net/study/ROSMASTER-X1)

## License

This repository is for educational purposes as part of the ECG711 course.

## Acknowledgments

- Course instructor and teaching assistants
- Yahboom for robot hardware and documentation
- ROS community for excellent documentation and packages
