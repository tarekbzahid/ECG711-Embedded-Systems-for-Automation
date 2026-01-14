# About ECG711 - Embedded Systems for Automation

## Course Overview

ECG711 (Embedded Systems for Automation) is an advanced course that bridges the gap between theoretical robotics concepts and practical embedded systems implementation. This course explores the integration of hardware and software systems to create autonomous robots capable of navigation, sensing, and decision-making in real-world environments.

The coursework progresses from simulation-based learning to hands-on hardware implementation, providing a comprehensive understanding of modern robotics systems and embedded computing platforms.

## Learning Journey

### From Simulation to Reality

This repository documents a complete learning journey in robotics and embedded systems, starting with software simulation and culminating in a fully functional physical robot. The progression follows a deliberate pedagogical path:

1. **Simulation First**: Begin with ROS nodes and topics in a controlled simulation environment
2. **Control Systems**: Implement feedback control mechanisms for autonomous behavior
3. **3D Modeling**: Create accurate digital representations of physical robots
4. **Hardware Integration**: Deploy learned concepts on actual embedded hardware

This approach ensures strong theoretical understanding before tackling the complexities of real-world hardware.

## Learning Objectives

Through the four assignments in this course, students develop expertise in:

### Software Development
- **ROS Architecture**: Understanding nodes, topics, publishers, subscribers, and messages
- **Python Programming**: Implementing control algorithms and robot behaviors
- **State Management**: Tracking robot pose, velocity, and sensor data over time
- **Inter-Process Communication**: Managing data flow between distributed system components

### Control Theory
- **Differential Drive Kinematics**: Computing robot motion from wheel velocities
- **PID Control**: Implementing proportional-integral-derivative controllers for goal-seeking behavior
- **Feedback Loops**: Designing closed-loop control systems for autonomous navigation
- **Error Minimization**: Tuning control parameters for optimal performance

### Robot Modeling & Simulation
- **URDF/XACRO**: Creating robot descriptions with joints, links, and sensors
- **Gazebo Simulation**: Simulating realistic physics and sensor behavior
- **RViz Visualization**: Visualizing robot state, sensor data, and trajectories
- **Transform Trees**: Managing coordinate frames for multi-link robots

### Embedded Systems
- **Hardware Assembly**: Integrating processors, motor controllers, sensors, and power systems
- **Embedded Linux**: Working with Ubuntu on resource-constrained platforms (Jetson Nano)
- **Sensor Calibration**: Ensuring accurate IMU and odometry measurements
- **Real-Time Systems**: Managing timing constraints in robotics applications

### Sensor Integration & Mapping
- **LiDAR Processing**: Working with 2D laser scan data for environment perception
- **SLAM Algorithms**: Simultaneous localization and mapping using gmapping
- **Sensor Fusion**: Combining IMU and odometry data with Extended Kalman Filters
- **Occupancy Grid Mapping**: Creating 2D maps for navigation

## Technical Deep Dive

### Assignment 1: Foundation in ROS Communication

This assignment establishes the fundamental architecture of ROS-based robotics systems. By creating separate driver and simulator nodes, students learn the publish-subscribe pattern that underpins distributed robotics applications.

**Key Technical Achievements**:
- Understanding message passing between processes
- Computing robot pose from differential wheel velocities
- Implementing kinematic equations for circular and straight-line motion
- Managing temporal data for accurate position estimation

The simulator implements the differential drive kinematic model:
- For straight motion (equal wheel velocities): Linear displacement along the heading
- For curved motion (unequal wheel velocities): Rotation around an instantaneous center of rotation (ICR)

### Assignment 2: Autonomous Control Systems

Building on Assignment 1, this work introduces feedback control—a cornerstone of autonomous robotics. The PID controller continuously adjusts robot motion to minimize error between current and desired poses.

**Key Technical Achievements**:
- Implementing proportional, integral, and derivative terms
- Tuning control gains (Kp, Ki, Kd) for stable convergence
- Managing both position and orientation errors simultaneously
- Creating feedback loops that run at consistent frequencies

The PID controller architecture:
- **P (Proportional)**: Provides control signal proportional to current error
- **I (Integral)**: Eliminates steady-state error by accumulating past errors
- **D (Derivative)**: Dampens oscillations by responding to rate of error change

### Assignment 3: Digital Twin Development

This assignment focuses on creating accurate digital representations of physical robots. The URDF model serves as a "digital twin" that can be used for simulation, visualization, and motion planning.

**Key Technical Achievements**:
- Defining robot geometry with links and joints
- Adding sensors (LiDAR) to the robot model
- Configuring mass, inertia, and collision properties
- Visualizing robot motion in real-time with RViz

The enhanced URDF model closely matches the physical robot's:
- Mechanical dimensions and wheel configuration
- Sensor placement (LiDAR positioning)
- Visual appearance for intuitive monitoring

### Assignment 4: Physical Robot Deployment

The culmination of the course brings all previous concepts into the physical world. This assignment addresses the challenges unique to real hardware: power management, sensor noise, calibration errors, and environmental uncertainty.

**Key Technical Achievements**:
- Assembling multi-component robotics systems
- Installing and configuring embedded Linux (Ubuntu 18.04)
- Establishing serial communication between Jetson Nano and STM32
- Performing systematic IMU and motor calibration
- Running SLAM algorithms on resource-constrained hardware
- Creating usable maps of real environments

The complete system integration includes:
- **Computing**: Jetson Nano (ARM-based embedded platform)
- **Motor Control**: STM32 microcontroller with ROS Controller firmware
- **Sensing**: RPLidar A3 for environment mapping, built-in IMU for orientation
- **Power**: 12V battery system with voltage regulation
- **Communication**: USB serial for Jetson-STM32, wireless for remote operation

## Real-World Applications

The skills developed in this course directly apply to numerous industry applications:

### Autonomous Vehicles
- Mobile robots for warehouse automation
- Delivery robots for last-mile logistics
- Agricultural robots for precision farming
- Security and patrol robots

### Manufacturing & Industry 4.0
- Automated guided vehicles (AGVs) in factories
- Inspection robots for quality control
- Collaborative robots (cobots) for human-robot interaction

### Service Robotics
- Cleaning and maintenance robots
- Healthcare assistance robots
- Hospitality and customer service robots

### Research & Development
- Experimental platforms for algorithm development
- Testbeds for new sensor technologies
- Educational tools for robotics instruction

## Technical Stack Summary

### Software
- **ROS Melodic**: Robot middleware for distributed computing
- **Python 3**: Primary programming language for node implementation
- **Gazebo**: High-fidelity physics simulation
- **RViz**: 3D visualization of robot and sensor data
- **Gmapping**: Grid-based SLAM algorithm
- **robot_localization**: Extended Kalman Filter for sensor fusion

### Hardware
- **Jetson Nano 4GB**: NVIDIA embedded AI computing platform
- **STM32**: ARM Cortex-M microcontroller for motor control
- **RPLidar A3**: 360° 2D laser scanner (25m range)
- **12V DC Motors**: Four-wheel differential drive
- **IMU**: Accelerometer and gyroscope for orientation
- **PS2 Joystick**: Teleoperation interface

### Development Tools
- **Catkin**: ROS build system
- **STM32CubeIDE**: Development environment for STM32
- **MobaXterm**: SSH client for remote development
- **Git**: Version control for code management

## Skills Developed

By completing this course, students gain proficiency in:

1. **Systems Integration**: Combining hardware and software into functional systems
2. **Debugging**: Troubleshooting issues across software, firmware, and hardware layers
3. **Embedded Programming**: Working within resource constraints of embedded platforms
4. **Control Theory**: Applying classical control techniques to real systems
5. **Sensor Processing**: Filtering and interpreting noisy sensor data
6. **Project Documentation**: Creating clear technical documentation and tutorials

## Future Directions

The foundation built in this course enables exploration of advanced topics:

- **Computer Vision**: Adding cameras for object detection and recognition
- **Path Planning**: Implementing A*, RRT, or other planning algorithms
- **Multi-Robot Systems**: Coordinating multiple robots for collaborative tasks
- **Machine Learning**: Integrating neural networks for perception or control
- **Advanced SLAM**: 3D mapping, visual SLAM, or semantic SLAM
- **Navigation Stack**: Full autonomous navigation with costmaps and recovery behaviors

## Documentation Philosophy

This repository maintains detailed documentation for several reasons:

1. **Reproducibility**: Others can recreate the work by following the instructions
2. **Learning Resource**: Serves as a reference for future students and developers
3. **Portfolio**: Demonstrates technical competence to potential employers or collaborators
4. **Knowledge Retention**: Personal reference for reviewing concepts and implementations

Each assignment includes:
- High-level problem descriptions
- Detailed code explanations
- Step-by-step recreation instructions
- Visual aids (images, videos)
- Links to external resources

## Acknowledgments & Resources

This work builds upon the contributions of many in the robotics and open-source communities:

- **ROS Community**: For creating and maintaining the Robot Operating System
- **Yahboom**: For hardware documentation and ROS Controller firmware
- **Dr. Venki**: For course instruction and PID controller guidance
- **Open Source Contributors**: For packages like gmapping, robot_localization, and rplidar_ros

## Contact & Collaboration

For questions about this coursework or potential collaboration:

**Tarek Z**
Email: zahid@unlv.nevada.edu
YouTube: [ECG 711 Course Videos](https://www.youtube.com/playlist?list=PL-m3G6XXLHQtwv6o8FqcqjEjRjHKm8WNG)

---

*This document was created as part of the ECG711 course at UNLV. All code and documentation are provided for educational purposes.*
