# ROS PiCar autonomous navigation

This project is mostly structured as a ROS package, providing nodes for reading the PiCar's sensors and publishing odometry messages from a wheel encoder sensor and an IMU sensor.

## config
Configuration files for the robot_localization sensor-fusion node

## examples
### husky_control.py
Some basic code demonstrating how to autonomously control a robot based on incoming odometry data from its sensors.

## src
### ros_*.py
The files which will be run as ROS nodes, each one relates to a different sensor since ROS nodes are supposed to be small and single-task

### other files
Lower-level interfaces for the sensors, used by the ROS nodes above
