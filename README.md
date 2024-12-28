# FaultTolerantControl-PX4

This code is meant to be added as a PX4 internal module, to eliminate the need for onboard computing. 

# Failure Detection

This is achieved by detecting the resultant sudden jerk in the drone frame. A threshold for angular acceleration was found after extensive testing in multiple flight conditions, including failure at high-altitude flight, low-altitude flight, high-speed straight-line flight, and orbit motion.

# Failure Isolation

The module finds which motor has failed, using the direction of the resultant sudden jerk. In theory, just the sign of angular velocities along the 2 axes on the drone's body would tell us the direction of the jerk, but to eliminate false detections due to small vibrations, a threshold for angular velocities was developed after extensive testing in the above mentioned flight conditions. 

# Controlled Landing

Once the module detects a motor failure, it automatically calls the controlled-landing subroutine. 

> **_YAW CONTROL:_**  When one motor fails, the control over one degree of freedom must be given up, so in our case, we're giving up yaw control in order to control the rate of descent of the drone.

The subroutine sets the controller gains for yaw-control to zero, and then regulates the thrust of the other three motors. The thrust of the motor opposite the failed motor is regulated according to a research paper we found, by Mark W Mueller, on Fault-Tolerant Control of Quadcopters. It suggests that the ratio between the thrust of the motor opposite the failed motor, to that of the other 2 motors, should be low, to prevent flipping of the drone. 

Using the equations in the mentioned research paper, and some testing with motor failure from different altitudes, we developed a function for the thrust of the motor opposite the failed motor, as a function of the drone's altitude. 
