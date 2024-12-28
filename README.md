# FaultTolerantControl-PX4

This code is meant to be added as a PX4 internal module, to eliminate the need for onboard computing. 

# Failure Detection

Motor failure detection is achieved by detecting the resultant sudden jerk in the drone frame. A threshold for angular acceleration was found after extensive testing in multiple flight conditions, including failure at high-altitude flight, low-altitude flight, high-speed straight-line flight, and orbit motion.

