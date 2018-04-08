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

* a - acceleration is in the range [-1, 1] = [full brake, full throttle]
* delta - steering angle is in the range [-25°, 25°]

## Timestep Length and Elapsed Duration (N & dt)

The prediction horizon is the duration over which future predictions are made. This is referred as T.

T is the product of two other variables, N and dt.

N is the number of timesteps in the horizon. dt is how much time elapses between actuations. 

N, dt, and T are hyperparameters that need to be tuned for each model predictive controller we build. However, there are some general guidelines. T should be as large as possible, while dt should be as small as possible.

I tried different combinations of N and dt, including (N=20, dt=0.05), (N=15, dt=0.05), (N=10, dt=0.05), (N=20, dt=0.1) and so on. With higher N value, if the vehicle overshot the reference trajectory, it would begin to oscillate greately and drive off the track. With lower value of N, the vehicle may drive straight off the track.

Finally (N = 10, dt = .1) performs the best result.

## Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system.
```
// converting to vehicle coordinates
for(int i=0;i<ptsx.size();i++){
  double diffx = ptsx[i]-px;
  double diffy = ptsy[i]-py;
  ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
  ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
}

Eigen::VectorXd ptsxV = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
Eigen::VectorXd ptsyV = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
```
Then a 3rd-degree polynomial is fitted to the transformed waypoints. 

```
auto coeffs = polyfit(ptsxV, ptsyV, 3);
```

These polynomial coefficients are used to calculate the cte and epsi later on. 
```
double cte_l =  polyeval(coeffs, 0) + v * CppAD::sin(-atan(coeffs[1])) * dt;
double epsi_l = -atan(coeffs[1])+psi_l;
```

They are used by the solver as well to create a reference trajectory.

```
 std::vector<double> r;
 r = mpc.Solve(state, coeffs);
```

## Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

Latency of 0.1 seconds was added to the system so that our model will give actuations to the vehicle after 0.1 seconds, so our initial state should be after 0.1 seconds to get the correct optimum control values.



