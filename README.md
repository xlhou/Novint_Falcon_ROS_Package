# Novint_Falcon_ROS_Package
This repository contains three ROS packages:
1. falcon
2. virtual_slave
3. ram_log

1. falcon package:
This package controls the NOVINT Falcon haptic joystick under the admittance mode.
It servos the position reference derived from virtual slave's velocities using a PID controller;
A user force estimation module is used to estimate the force input from the human operator, and mapped to the velocity reference to the virtual slave velocity regulator.

The falcon package subscribes to a ROS topic named "vehicle_state" of Float32MultiArray type, and publishes the velocity reference and its displacements relative to its workspace "center" to "VICON_CON" topic of Float64MultiArray type.

In order to compile this package, the falcon library is required, which can be found in the repo named "libnifalcon-1.0.2.tar.gz"

2. virtual_slave package:
This package simulates the dynamics and kinematics of an aerial robot with a simple P controller to servo the velocity and yaw reference, and publishes the robot's velocities and yaw to a ROS topic named "vehicle_state" of Float32MultiArray type.

3. ram_log package:
This is a logging package that records all the data published by the above two packages in one logging file named "falcon.txt", which you can load to Matlab workspace using "load("falcon.txt")".

The details of the data:
time, vx_ref, vy_ref, vz_ref, yaw_ref, px, py, pz, vx, vy, vz;

You need to modify the path to the log director to record properly. 


