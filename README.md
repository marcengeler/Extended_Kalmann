# Extended Kalman Filter Project Starter Code

This code implements an extended Kalman Filter in C++ to analyze RADAR and LIDAR Data from a simulation in order to estimate the correct position of a bycicle.


## Setting up the code

To start off, I reviewed the course material and implemented the correct matrix sizes to initialized the Kalman filter. The RADAR and LIDAR measurements consists of different variables in different dimensions, that's why they have to be initialized seperately.

## First Tests

The first tests showed, that the model was off.

[capture]: .capture.PNG "Prediciton Error"
![alt text][capture]

From the behavior, i saw, that the measurement data in direction of the curve was off, and that probably the filter didn't update the matrices after initialization.