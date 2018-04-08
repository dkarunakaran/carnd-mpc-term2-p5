# CarND-Controls-MPC
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
The MPC considers the task of following a trajectory as an optimization problem in which the solution is the path the car should take. The idea is to simulate different actuator inputs (steering, acceleration and braking) and predict a resulting trajectory by selecting the one with the minimum cost. The car follows that trajectory and gets new input to calculate a new set of trajectories to optimize. 

## Model

The MPC model is based on kinematic bicycle model defined by a state of six parameters:
* x, y — position.
* psi(ψ) — orientation.
* v — velocity.
* cte — cross-track error. The difference between the trajectory defined by the waypoints and the current vehicle position y in the coordinate space of the vehicle.
* epsi (eψ) — orientation error.

And the vehicle model equation is:

<img src= "images/equation1.png">

Actuators values calculated by the MPC controller were steer angle and throttle. All the states and coefficients of the polynomial drawn on center of the road were made input to the MPC solve function. This function gave throttle and steering value as the output. Actuator values were decided at each time step by MPC controller such that it minimizes the cost function defined.

### Actuator constraints
These are limiting parameters defined by the design of the vehicle and fundamental physics — e.g. a car never makes a hard 90° turn. This is called a nonholonomic model. In our case:

* a — acceleration is in the range [-1, 1] = [full brake, full throttle]
* delta — steering angle is in the range [-25°, 25°]

